/*
 * cam: camera payload command surface (see cam_app.h for the contract).
 *
 * Validation lives here because refusal must be a flight-software decision
 * with an event line and a counter, not a sim-side shrug: the ground hears
 * "rejected, here is why" on EVS whether or not the frame would have made it
 * across the airgap.
 */

#include <math.h>
#include <stdio.h> /* snprintf, for the manifest's CSV lines */
#include <string.h>

#include "cfe.h"
#include "cf_msgids.h"
#include "cam_app.h"
#include "cam_events.h"
#include "cam_version.h"

CAM_AppData_t CAM_AppData;

/* -------------------------------------------------------------------------- */

static void CAM_SendHk(void)
{
    /* The four-number report: the commanded pair this app
     * earned, RECORDED from the sim's own CamState, STORED from bsk_link's
     * ledger, and TRANSFERRED counted by CF's own end-of-transaction
     * telemetry - a counter moves when its mechanism exists, never before.
     * Truncation to the 32-bit HK fields is safe by arithmetic:
     * one frame per render interval cannot reach 4 billion this century. */
    CAM_AppData.HkTlm.Payload.FramesRecorded = (uint32)CAM_AppData.FramesRecorded;
    CAM_AppData.HkTlm.Payload.FramesStored   = (uint32)CAM_AppData.FilesStored;
    CAM_AppData.HkTlm.Payload.FramesTransferred = (uint32)CAM_AppData.FilesTransferred;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(CAM_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(CAM_AppData.HkTlm.TelemetryHeader), true);
}

/* -------------------------------------------------------------------------- */

/**
 * Egress one accepted config to the sim (via bsk_link) and account for it.
 *
 * The stamp is the newest sensor frame's sim time and frame number: "this
 * config was commanded while the vehicle believed it was at t=..." The sim
 * applies at the next frame boundary, so the stamp is attribution, not a
 * deadline - the command surface owns no delivery guarantee, and honesty about that is the
 * whole reason CamState exists as a separate witness.
 */
static void CAM_ForwardConfig(void)
{
    BskCamConfigTlm_t Config;
    CFE_Status_t      Status;

    CFE_MSG_Init(CFE_MSG_PTR(Config.TelemetryHeader), CFE_SB_ValueToMsgId(BSK_LINK_CAM_CONFIG_TLM_MID), sizeof(Config));
    Config.Payload.SimTimeS   = CAM_AppData.LastSimTimeS;
    Config.Payload.Frame      = CAM_AppData.LastFrame;
    Config.Payload.CameraId   = CAM_AppData.CameraId;
    Config.Payload.Shutter    = CAM_AppData.Shutter;
    Config.Payload.RenderRateS = CAM_AppData.RenderRateCmd;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Config.TelemetryHeader));
    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Config.TelemetryHeader), true);
    if (Status == CFE_SUCCESS)
    {
        CAM_AppData.ConfigFramesForwarded++;
        CAM_AppData.HkTlm.Payload.ConfigFramesForwarded = CAM_AppData.ConfigFramesForwarded;
        CAM_AppData.HkTlm.Payload.LastConfigSimTimeS    = CAM_AppData.LastSimTimeS;
    }
    else if (CAM_AppData.ConfigFwdFailCount == 0)
    {
        /* Say it once: the config is commanded but the sim will never see
         * it, and HK-vs-CamState divergence will start accruing against a
         * healthy-looking ground. This event names the real culprit. */
        CFE_EVS_SendEvent(CAM_CONFIG_ERR_EID, CFE_EVS_EventType_ERROR, "cam config publish rejected: 0x%08X",
                          (unsigned int)Status);
        CAM_AppData.ConfigFwdFailCount++;
    }
    else
    {
        CAM_AppData.ConfigFwdFailCount++;
    }
}

/* -------------------------------------------------------------------------- */

static void CAM_HandleSetConfig(const CFE_SB_Buffer_t *BufPtr)
{
    const CamSetConfigCmd_t *Cmd  = (const CamSetConfigCmd_t *)BufPtr;
    size_t                   Size;
    uint8                    Shutter;
    uint8                    CameraId;
    double                   Rate;

    CFE_MSG_GetSize(&BufPtr->Msg, &Size);
    if (Size != sizeof(CamSetConfigCmd_t))
    {
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_SET_CONFIG_ERR_EID, CFE_EVS_EventType_ERROR, "SET_CONFIG length %u, expected %u: ignored",
                          (unsigned int)Size, (unsigned int)sizeof(CamSetConfigCmd_t));
        return;
    }

    Shutter   = Cmd->Payload.Shutter;
    CameraId  = Cmd->Payload.CameraId;
    Rate      = Cmd->Payload.RenderRateS;

    /* Three refusals, each its own failure mode:
     * - shutter beyond on/off is a schema disagreement, not a value;
     * - camera id 0 names no eye ([[camera]] ids start at 1);
     * - rate <= 0 or non-finite is a divide-by-zero one layer before the
     *   hardware could take it. The sim door refuses again on the same law:
     *   both sides of the airgap guard the modulo, because the cost of one
     *   side being stale is a dead vehicle. */
    if (Shutter > 1 || CameraId == 0 || !isfinite(Rate) || Rate <= 0.0 || Rate > CAM_RENDER_RATE_MAX)
    {
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_SET_CONFIG_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SET_CONFIG rejected (shutter=%u camera=%u rate=%.6g): need shutter 0|1, camera >= 1, "
                          "finite 0 < rate <= %g s",
                          (unsigned int)Shutter, (unsigned int)CameraId, Rate, CAM_RENDER_RATE_MAX);
        return;
    }

    CAM_AppData.Shutter             = Shutter;
    CAM_AppData.CameraId            = CameraId;
    CAM_AppData.RenderRateCmd       = Rate;
    CAM_AppData.HkTlm.Payload.Shutter       = Shutter;
    CAM_AppData.HkTlm.Payload.CameraId      = CameraId;
    CAM_AppData.HkTlm.Payload.RenderRateCmd = Rate;
    CAM_AppData.HkTlm.Payload.CommandCounter++;

    CAM_ForwardConfig();

    CFE_EVS_SendEvent(CAM_SET_CONFIG_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "camera %u configured: shutter %s, render rate %.3f s (forwarded at t=%.1fs frame=%llu)",
                      (unsigned int)CameraId, (Shutter ? "OPEN" : "CLOSED"), Rate, CAM_AppData.LastSimTimeS,
                      (unsigned long long)CAM_AppData.LastFrame);
}

/* -------------------------------------------------------------------------- */

/* One inventory line waiting to be sorted: the stat we already paid for,
 * and the path to print. Static, not stack - the store can legitimately
 * hold hundreds of frames, and a CFE task stack has no such room. */
#define CAM_MANIFEST_MAX_FILES 512

typedef struct
{
    char   Path[OS_MAX_PATH_LEN];
    uint64 Size;
    int64  When; /* mtime seconds since the Unix epoch; ties broken by Path */
} CamStoreEntry;

static CamStoreEntry CamManifestWalk[CAM_MANIFEST_MAX_FILES];

/* Seconds since the Unix epoch -> "YYYY-MM-DDTHH:MM:SSZ". Hand-rolled civil
 * calendar arithmetic (the days-from-civil algorithm) instead of libc's
 * gmtime(): flight code has no business consulting the host's timezone
 * table, and the integer form is exact. The clock this names is the
 * container's wall clock - on the bench, the host's - not mission time:
 * it records when the store PHYSICALLY gained a file. */
static void CAM_FormatUtc(int64 Seconds, char *Buf, size_t Len)
{
    int64 Days  = Seconds / 86400;
    int64 Secs  = Seconds % 86400;
    int64 Era, Doe, Yoe, Year, Doy, Mp, Day, Month;

    if (Secs < 0)
    {
        Secs += 86400;
        Days -= 1; /* floor, so a pre-1970 stamp still names its true day */
    }
    Days += 719468;
    Era   = (Days >= 0 ? Days : Days - 146096) / 146097;
    Doe   = Days - Era * 146097;                     /* day of era  [0,146096] */
    Yoe   = (Doe - Doe / 1460 + Doe / 36524 - Doe / 146096) / 365;
    Year  = Yoe + Era * 400;
    Doy   = Doe - (365 * Yoe + Yoe / 4 - Yoe / 100); /* day of year   [0,365] */
    Mp    = (5 * Doy + 2) / 153;                     /* month probe   [0,11]  */
    Day   = Doy - (153 * Mp + 2) / 5 + 1;
    Month = Mp < 10 ? Mp + 3 : Mp - 9;
    Year += (Month <= 2);
    snprintf(Buf, Len, "%04llu-%02llu-%02lluT%02llu:%02llu:%02lluZ",
             (unsigned long long)Year, (unsigned long long)Month, (unsigned long long)Day,
             (unsigned long long)(Secs / 3600), (unsigned long long)((Secs / 60) % 60),
             (unsigned long long)(Secs % 60));
}

/* Insert one entry keeping CamManifestWalk ordered by creation time, so a
 * reader meets the frames in the order the vehicle captured them. A directory
 * walk answers in filesystem hash order - tmpfs and ext4 return entries in
 * whatever order their internal tables hold, which is why the old manifest
 * listed frame_1_50 above frame_1_2 and read as random. Mtimes carry one
 * second of resolution, and a multi-eye bird captures several files inside
 * one second, so equal timestamps order by path: ties resolve to the same
 * list on every walk, and a table a reader can diff is worth more than a
 * table that merely looks sorted. */
static void CAM_ManifestInsert(uint32 *Count, const char *Path, uint64 Size, int64 When)
{
    uint32 At = *Count;

    while (At > 0 && (CamManifestWalk[At - 1].When > When ||
                      (CamManifestWalk[At - 1].When == When && strcmp(CamManifestWalk[At - 1].Path, Path) > 0)))
    {
        CamManifestWalk[At] = CamManifestWalk[At - 1];
        At--;
    }
    snprintf(CamManifestWalk[At].Path, sizeof(CamManifestWalk[At].Path), "%s", Path);
    CamManifestWalk[At].Size = Size;
    CamManifestWalk[At].When = When;
    *Count += 1;
}

/* The inventory: walk the store, draft the manifest whole, swap it in.
 *
 * The file is CSV - header path,size_bytes,created_utc, then one line per
 * stored file, FULL path first: a reader parsing this should not have to
 * reconstruct where the file lives from a bare name, and a name shaped like
 * frame_<camera>_<capture>.png is bsk_link's storage convention, not a table
 * of columns to segment. No comment lines: they are exactly the character a
 * strict CSV reader chokes on, and the totals they used to carry are already
 * on EVS in the completion event, loud where a machine will not trip.
 *
 * Names starting with '.' are store internals (bsk_link's .part files): the
 * inventory lists what the ground may ask for, and a half-written part is
 * not that. Sizes and times come from OS_stat per name - this OSAL's dirent
 * carries a name only. A file that cannot be stat-ed - vanished between the
 * read and the stat, or a name over the store's path budget - is skipped and
 * named in an event: an inventory line for a file nobody can request is the
 * kind of line that makes liars of good tables. A store larger than the
 * sort budget refuses the manifest whole rather than print a truncated
 * inventory dressed as a complete one.
 *
 * The draft-and-swap is not ceremony: a CFDP read already in flight holds
 * the OLD file whole while the new manifest lands, so a manifest rewrite
 * mid-downlink corrupts neither generation. */
static void CAM_HandleManifest(bool FromGround)
{
    osal_id_t   DirId;
    os_dirent_t Entry;
    os_fstat_t  Stats;
    int32       Status;
    osal_id_t   Fd;
    uint32      Files   = 0;
    uint64      Bytes   = 0;
    uint32      Walked  = 0;
    bool        WriteOk = true;
    char        Line[160];

    Status = OS_DirectoryOpen(&DirId, CAM_IMG_DIR);
    if (Status != OS_SUCCESS)
    {
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR,
                          "manifest refused: %s unreadable (status %d) - has the store ever run?", CAM_IMG_DIR,
                          (int)Status);
        return;
    }

    Status = OS_OpenCreate(&Fd, CAM_MANIFEST_TMP, OS_FILE_FLAG_CREATE | OS_FILE_FLAG_TRUNCATE, OS_WRITE_ONLY);
    if (Status != OS_SUCCESS)
    {
        OS_DirectoryClose(DirId);
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR, "manifest refused: %s unopenable (status %d)",
                          CAM_MANIFEST_TMP, (int)Status);
        return;
    }

    snprintf(Line, sizeof(Line), "path,size_bytes,created_utc\n");
    if (OS_write(Fd, Line, strlen(Line)) < 0)
    {
        WriteOk = false;
    }

    while (WriteOk && OS_DirectoryRead(DirId, &Entry) == OS_SUCCESS)
    {
        char        Path[OS_MAX_PATH_LEN];
        const char *Name = OS_DIRENTRY_NAME(Entry);

        if (Name[0] == '\0' || Name[0] == '.')
        {
            continue;
        }
        snprintf(Path, sizeof(Path), "%s/%s", CAM_IMG_DIR, Name);
        Status = OS_stat(Path, &Stats);
        if (Status != OS_SUCCESS)
        {
            /* A file nobody can request has no business on the inventory,
             * but dropping it in silence moves the lie to the reader. */
            CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR,
                              "manifest skipped %s: status %d", Name, (int)Status);
            continue;
        }
        if (Walked >= CAM_MANIFEST_MAX_FILES)
        {
            /* The sort budget is the store's size in files; a fuller store
             * gets a refusal, never a shorter list pretending to be all. */
            OS_DirectoryClose(DirId);
            OS_close(Fd);
            OS_remove(CAM_MANIFEST_TMP);
            CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
            CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR,
                              "manifest refused: store holds over %d files (sort budget) - previous manifest kept",
                              CAM_MANIFEST_MAX_FILES);
            return;
        }
        CAM_ManifestInsert(&Walked, Path, (uint64)OS_FILESTAT_SIZE(Stats), (int64)OS_FILESTAT_TIME(Stats));
    }
    OS_DirectoryClose(DirId);

    for (uint32 At = 0; WriteOk && At < Walked; At++)
    {
        char When[32];

        CAM_FormatUtc(CamManifestWalk[At].When, When, sizeof(When));
        snprintf(Line, sizeof(Line), "%s,%llu,%s\n", CamManifestWalk[At].Path,
                 (unsigned long long)CamManifestWalk[At].Size, When);
        if (OS_write(Fd, Line, strlen(Line)) != (int32)strlen(Line))
        {
            WriteOk = false; /* a partial line is worse than no manifest */
            break;
        }
        Files++;
        Bytes += CamManifestWalk[At].Size;
    }

    OS_close(Fd);

    if (!WriteOk)
    {
        OS_remove(CAM_MANIFEST_TMP); /* the old manifest, if any, stays valid */
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR,
                          "manifest write failed at file %lu of the walk - previous manifest kept",
                          (unsigned long)Files);
        return;
    }

    Status = OS_mv(CAM_MANIFEST_TMP, CAM_MANIFEST_PATH);
    if (Status != OS_SUCCESS)
    {
        OS_remove(CAM_MANIFEST_TMP);
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_MANIFEST_ERR_EID, CFE_EVS_EventType_ERROR, "manifest swap failed (status %d)",
                          (int)Status);
        return;
    }

    CAM_AppData.ManifestWrites++;
    CAM_AppData.ManifestWritten = 1; /* the inventory now exists: the store owes it news */
    if (FromGround)
    {
        CAM_AppData.HkTlm.Payload.CommandCounter++;
    }
    CFE_EVS_SendEvent(CAM_MANIFEST_INF_EID, CFE_EVS_EventType_INFORMATION, "manifest written: %s - %lu files, %llu bytes",
                      CAM_MANIFEST_PATH, (unsigned long)Files, (unsigned long long)Bytes);
}

/* -------------------------------------------------------------------------- */

static void CAM_HandleCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t FcnCode = 0;
    size_t            Size;

    CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);
    CFE_MSG_GetSize(&BufPtr->Msg, &Size);

    /* All CAM_CMD function codes are header-only by contract (cam.xml):
     * a nonzero payload means the sender and this app disagree about the
     * schema, which is a defect to report, not to interpret. */
    if (Size != sizeof(CamNoopCmd_t))
    {
        CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(CAM_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "CMD FC=%u bad length %u: ignored",
                          (unsigned int)FcnCode, (unsigned int)Size);
        return;
    }

    switch (FcnCode)
    {
        case 0: /* NOOP */
            CAM_AppData.HkTlm.Payload.CommandCounter++;
            CFE_EVS_SendEvent(CAM_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "CAM No-op command, Version %u.%u.%u, camera %u, configs forwarded %lu",
                              CAM_APP_VERSION, CAM_APP_REV_MAJOR, CAM_APP_REV_MINOR, (unsigned int)CAM_AppData.CameraId,
                              (unsigned long)CAM_AppData.ConfigFramesForwarded);
            break;

        case 1: /* MANIFEST */
            CAM_HandleManifest(true);
            break;

        default:
            CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
            CFE_EVS_SendEvent(CAM_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "invalid function code %u",
                              (unsigned int)FcnCode);
            break;
    }
}

/* -------------------------------------------------------------------------- */

static int32 CAM_AppInit(void)
{
    CFE_Status_t Status;

    memset(&CAM_AppData, 0, sizeof(CAM_AppData));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&CAM_AppData.Pipe, CAM_PIPE_DEPTH, "CAM.Pipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(CAM_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe create failed, status=%08X",
                          (unsigned int)Status);
        return Status;
    }

    struct
    {
        uint32     Mid;
        const char *What;
    } Subscriptions[] = {
        {BSK_LINK_SENSOR_FRAME_TLM_MID, "sensor frame (stamping)"},
        {CAM_CMD_MID, "commands"},
        {CAM_SET_CONFIG_MID, "config upload"},
        {CAM_SEND_HK_MID, "housekeeping request"},
        {BSK_LINK_CAM_STATE_TLM_MID, "cam state (recorded witness)"},
        {BSK_LINK_CAM_STORAGE_TLM_MID, "image ledger (stored witness)"},
        {CF_EOT_TLM_MID, "file transactions (transferred witness)"},
    };

    for (size_t i = 0; i < sizeof(Subscriptions) / sizeof(Subscriptions[0]); ++i)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(Subscriptions[i].Mid), CAM_AppData.Pipe);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(CAM_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "subscribe %s (0x%04X) failed, status=%08X",
                              Subscriptions[i].What, (unsigned int)Subscriptions[i].Mid, (unsigned int)Status);
            return Status;
        }
    }

    CFE_MSG_Init(CFE_MSG_PTR(CAM_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(CAM_HK_TLM_MID),
                 sizeof(CAM_AppData.HkTlm));

    CFE_EVS_SendEvent(CAM_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "CAM App Initialized. Version %u.%u.%u, camera unconfigured (CameraId 0)", CAM_APP_VERSION,
                      CAM_APP_REV_MAJOR, CAM_APP_REV_MINOR);

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void CAM_AppMain(void)
{
    uint32          RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_Status_t    Status;

    Status = CAM_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, CAM_AppData.Pipe, CAM_SB_TIMEOUT);
            if (Status != CFE_SUCCESS)
            {
                continue; /* timeout: the normal case between 1 Hz frames */
            }

            CFE_SB_MsgId_t       MsgId;
            CFE_SB_MsgId_Atom_t  MsgIdValue;
            size_t               Size;

            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            MsgIdValue = CFE_SB_MsgIdToValue(MsgId);
            CFE_MSG_GetSize(&BufPtr->Msg, &Size);

            switch (MsgIdValue)
            {
                case BSK_LINK_SENSOR_FRAME_TLM_MID:
                    /* Stamping only: the newest sim instant this vehicle has
                     * evidence for. A wrong length is a schema drift, not a
                     * reason to stamp a config with a stale time - keep the
                     * old stamp and count nothing: bsk_link already audits
                     * malformed link frames. */
                    if (Size == sizeof(BskSensorFrameTlm_t))
                    {
                        const BskSensorFrameTlm_t *Frame = (const BskSensorFrameTlm_t *)BufPtr;
                        CAM_AppData.LastSimTimeS        = Frame->Payload.SimTimeS;
                        CAM_AppData.LastFrame           = Frame->Payload.Frame;
                    }
                    break;

                case CAM_CMD_MID:
                    CAM_HandleCmd(BufPtr);
                    break;

                case CAM_SET_CONFIG_MID:
                    CAM_HandleSetConfig(BufPtr);
                    break;

                case CAM_SEND_HK_MID:
                    CAM_SendHk();
                    break;

                case BSK_LINK_CAM_STATE_TLM_MID:
                    /* RECORDED witness: the sim's own tally of what its
                     * store outlet sent. Wrong length is schema drift -
                     * keep the last honest value, count nothing. */
                    if (Size == sizeof(BskCamStateTlm_t))
                    {
                        CAM_AppData.FramesRecorded =
                            ((const BskCamStateTlm_t *)BufPtr)->Payload.FramesRecorded;
                    }
                    break;

                case BSK_LINK_CAM_STORAGE_TLM_MID:
                    /* STORED witness: bsk_link emits its ledger on every
                     * store mutation - a file landed, or a proven one was
                     * recycled. The inventory law lives here: once a
                     * manifest exists this boot, every mutation re-derives
                     * it from the directory. A store that moves while the
                     * inventory keeps listing yesterday's files makes
                     * liars of both. */
                    if (Size == sizeof(BskCamStorageTlm_t))
                    {
                        const BskCamStorageTlm_t *Storage = (const BskCamStorageTlm_t *)BufPtr;
                        CAM_AppData.FilesStored = Storage->Payload.FilesStored;
                        if (Storage->Payload.Generation != CAM_AppData.StoreGeneration)
                        {
                            CAM_AppData.StoreGeneration = Storage->Payload.Generation;
                            if (CAM_AppData.ManifestWritten)
                            {
                                CAM_HandleManifest(false);
                            }
                        }
                    }
                    break;

                case CF_EOT_TLM_MID:
                    /* TRANSFERRED witness: CF ends every closed file
                     * transaction with one EOT packet - the only per-file
                     * testimony CF gives. Count only what crossed: TX
                     * direction, clean status (wire values documented on
                     * the mirror struct); every failure stays visible as a
                     * divergence against STORED instead of being laundered
                     * into the counter. Wrong length is schema drift -
                     * count nothing, keep the last honest value. */
                    if (Size == sizeof(CfEotTlm_t))
                    {
                        const CfEotTlm_t *Eot = (const CfEotTlm_t *)BufPtr;
                        if (Eot->Payload.Direction == 1u && Eot->Payload.TxnStat == 0u)
                        {
                            CAM_AppData.FilesTransferred++;
                        }
                    }
                    break;

                default:
                    /* Nothing else is subscribed on this pipe; a wrong MsgId
                     * reaching here is a routing defect worth one counter,
                     * not an event storm. */
                    CAM_AppData.HkTlm.Payload.CommandErrorCounter++;
                    break;
            }
        }
    }

    CFE_ES_WriteToSysLog("CAM exit: cmds=%lu errs=%lu camera=%u shutter=%u rate=%.3f fwd=%lu/%lu\n",
                         (unsigned long)CAM_AppData.CommandCounter, (unsigned long)CAM_AppData.CommandErrorCounter,
                         (unsigned int)CAM_AppData.CameraId, (unsigned int)CAM_AppData.Shutter,
                         CAM_AppData.RenderRateCmd, (unsigned long)CAM_AppData.ConfigFramesForwarded,
                         (unsigned long)CAM_AppData.ConfigFwdFailCount);

    CFE_ES_ExitApp(RunStatus);
}
