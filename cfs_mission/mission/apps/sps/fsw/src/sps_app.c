/*
 * sps: time-tagged command execution (see sps_app.h for the contract).
 *
 * Two clocks exist on this vehicle and only one of them decides physics:
 * wall time runs the schedulers and sockets, sim time runs the world. A
 * schedule is a statement about sim time, so it is evaluated against the
 * sensor frame's own timecode - the same number the lockstep gate advanced
 * - never against CFE_TIME, which merely mirrors it one task hop behind.
 */

#include <math.h>
#include <string.h>

#include "cfe.h"
#include "sps_app.h"
#include "sps_events.h"
#include "sps_version.h"

SPS_AppData_t SPS_AppData;

/* -------------------------------------------------------------------------- */

static const SpsCmdEntry_t *SPS_LookupCmd(uint16 MsgId, uint8 Fc)
{
    for (size_t i = 0; i < SPS_CMD_COUNT; ++i)
    {
        if (SPS_CMD_TABLE[i].MsgId == MsgId && SPS_CMD_TABLE[i].Fc == Fc)
        {
            return &SPS_CMD_TABLE[i];
        }
    }
    return NULL;
}

static uint32 SPS_QueueCount(void)
{
    return SPS_AppData.QueueCount;
}

/* Insert sorted by ExecTimeS; ties keep upload order (a stable queue is a
 * testable queue: two schedules stamped at the same T fire in the order the
 * ground sent them, every boot, by construction not by luck). */
static bool SPS_QueueInsert(const Sps_Entry_t *Entry)
{
    if (SPS_AppData.QueueCount >= SPS_SLOTS)
    {
        return false;
    }

    uint32 Pos = SPS_AppData.QueueCount;
    while (Pos > 0 && SPS_AppData.Queue[Pos - 1].ExecTimeS > Entry->ExecTimeS)
    {
        SPS_AppData.Queue[Pos] = SPS_AppData.Queue[Pos - 1];
        --Pos;
    }
    SPS_AppData.Queue[Pos]  = *Entry;
    SPS_AppData.QueueCount++;
    if (SPS_AppData.QueueCount > SPS_AppData.QueuePeak)
    {
        SPS_AppData.QueuePeak = SPS_AppData.QueueCount;
    }
    return true;
}

/* Re-emit one stored entry onto the bus as the command it captured. */
static void SPS_Fire(const Sps_Entry_t *Entry, double FrameTimeS)
{
    Sps_FireMsg_t Out;

    memset(&Out, 0, sizeof(Out));
    CFE_MSG_Init(CFE_MSG_PTR(Out.Hdr), CFE_SB_ValueToMsgId(Entry->MsgId),
                 sizeof(CFE_MSG_CommandHeader_t) + Entry->Len);
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Out.Hdr), Entry->Fc);
    if (Entry->Len > 0)
    {
        memcpy(Out.Data, Entry->Data, Entry->Len);
    }

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Out.Hdr));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(Out.Hdr), true) == CFE_SUCCESS)
    {
        SPS_AppData.FiredTotal++;
        SPS_AppData.LastFiredMsgId     = Entry->MsgId;
        SPS_AppData.LastExecTimeS      = Entry->ExecTimeS;
        SPS_AppData.LastFiredSimTimeS  = FrameTimeS;
        CFE_EVS_SendEvent(SPS_FIRE_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "scheduled command fired: mid=0x%04X fc=%u len=%u (T=%.3f s, frame t=%.3f s)",
                          (unsigned int)Entry->MsgId, (unsigned int)Entry->Fc, (unsigned int)Entry->Len,
                          Entry->ExecTimeS, FrameTimeS);
    }
    else if (SPS_AppData.FiredTotal == 0)
    {
        /* Publish rejection for a fired command is an app bug, not a
         * condition: report once per run rather than storm a frozen sim. */
        CFE_EVS_SendEvent(SPS_FIRE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "scheduled fire publish rejected: mid=0x%04X fc=%u", (unsigned int)Entry->MsgId,
                          (unsigned int)Entry->Fc);
    }
}

/* Drop the front entry (fired or not) and shift the rest toward it. */
static void SPS_QueuePopFront(void)
{
    for (uint32 i = 1; i < SPS_AppData.QueueCount; ++i)
    {
        SPS_AppData.Queue[i - 1] = SPS_AppData.Queue[i];
    }
    SPS_AppData.QueueCount--;
}

/* -------------------------------------------------------------------------- */

static void SPS_EvaluateQueue(double FrameTimeS)
{
    SPS_AppData.FramesEvaluated++;
    SPS_AppData.LastFrameSimTimeS = FrameTimeS;

    if (!SPS_AppData.Enabled)
    {
        return; /* armed-but-frozen: the clock moves, the queue does not */
    }

    /* Queue is sorted, so due entries are a prefix. Fire while due: a single
     * frame may release several entries (all "first frame >= T"), and the
     * prefix walk fires them in time order. */
    while (SPS_AppData.QueueCount > 0 && SPS_AppData.Queue[0].ExecTimeS <= FrameTimeS)
    {
        Sps_Entry_t Due = SPS_AppData.Queue[0];
        SPS_QueuePopFront();
        SPS_Fire(&Due, FrameTimeS);
    }
}

/* -------------------------------------------------------------------------- */

static void SPS_HandleSchedule(const CFE_SB_Buffer_t *BufPtr)
{
    const SpsScheduleCmd_t *Cmd  = (const SpsScheduleCmd_t *)BufPtr;
    const SpsSchedule_Payload_t *Req = &Cmd->Payload;
    Sps_Entry_t                   Entry;
    const SpsCmdEntry_t          *Dict;

    size_t Size;
    CFE_MSG_GetSize(&BufPtr->Msg, &Size);
    if (Size != sizeof(SpsScheduleCmd_t))
    {
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "SCHEDULE bad length %u, expected %u: ignored",
                          (unsigned int)Size, (unsigned int)sizeof(SpsScheduleCmd_t));
        return;
    }

    /* Self-schedule: a queue that can feed itself needs no operator. */
    if (Req->CmdMsgId == SPS_SCHEDULE_CMD_MID)
    {
        SPS_AppData.RejectedParamTotal++;
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_REJ_PARAM_ERR_EID, CFE_EVS_EventType_ERROR,
                          "schedule refused: SPS may not schedule its own SCHEDULE port");
        return;
    }

    /* The dictionary is the EDS: if the ground can send it, the flight can
     * judge it - and if flight cannot name it, it does not fly. */
    Dict = SPS_LookupCmd(Req->CmdMsgId, Req->CmdFc);
    if (Dict == NULL)
    {
        SPS_AppData.RejectedUnknownTotal++;
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_REJ_UNKNOWN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "schedule refused: mid=0x%04X fc=%u is not in the command dictionary",
                          (unsigned int)Req->CmdMsgId, (unsigned int)Req->CmdFc);
        return;
    }

    if (!isfinite(Req->ExecTimeS) || Req->ExecTimeS < 0.0)
    {
        SPS_AppData.RejectedParamTotal++;
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_REJ_PARAM_ERR_EID, CFE_EVS_EventType_ERROR,
                          "schedule refused: T=%g is not a usable sim time (need finite, >= 0)", Req->ExecTimeS);
        return;
    }

    /* Length must equal the dictionary's own size for this (MsgId, Fc):
     * a short image would fire as a truncated command the target app
     * (rightly) drops at dispatch; a long one cannot exist on this wire. */
    if (Req->CmdDataLen != Dict->PayloadBytes || Req->CmdDataLen > SPS_DATA_BYTES)
    {
        SPS_AppData.RejectedParamTotal++;
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_REJ_PARAM_ERR_EID, CFE_EVS_EventType_ERROR,
                          "schedule refused: %s needs %u payload bytes, envelope carries %u", Dict->Name,
                          (unsigned int)Dict->PayloadBytes, (unsigned int)Req->CmdDataLen);
        return;
    }

    memset(&Entry, 0, sizeof(Entry));
    Entry.ExecTimeS = Req->ExecTimeS;
    Entry.MsgId     = Req->CmdMsgId;
    Entry.Fc        = Req->CmdFc;
    Entry.Len       = Req->CmdDataLen;
    if (Entry.Len > 0)
    {
        memcpy(Entry.Data, Req->CmdData, Entry.Len);
    }

    if (!SPS_QueueInsert(&Entry))
    {
        SPS_AppData.RejectedFullTotal++;
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_REJ_FULL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "schedule refused: queue full (%u entries), mid=0x%04X T=%.3f dropped",
                          (unsigned int)SPS_SLOTS, (unsigned int)Req->CmdMsgId, Req->ExecTimeS);
        return;
    }

    /* Late means the upload arrived after the world had already passed T -
     * the entry still fires on the next frame (frame-quantized as ever),
     * but the ground is told its timing was optimistic, not pretended away. */
    bool Late = SPS_AppData.FramesEvaluated > 0 && Req->ExecTimeS < SPS_AppData.LastFrameSimTimeS;
    if (Late)
    {
        SPS_AppData.LateFiredTotal++;
    }

    SPS_AppData.ScheduledTotal++;
    SPS_AppData.HkTlm.Payload.CommandCounter++;
    CFE_EVS_SendEvent(SPS_SCHEDULE_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "scheduled %s: mid=0x%04X fc=%u len=%u at T=%.3f s%s (queue %u/%u, %s)", Dict->Name,
                      (unsigned int)Req->CmdMsgId, (unsigned int)Req->CmdFc, (unsigned int)Entry.Len, Req->ExecTimeS,
                      Late ? " LATE: T already passed, fires next frame" : "", (unsigned int)SPS_QueueCount(),
                      (unsigned int)SPS_SLOTS, SPS_AppData.Enabled ? "ARMED" : "FROZEN");
}

/* -------------------------------------------------------------------------- */

static void SPS_SendHk(void)
{
    SpsHkTlm_Payload_t *Hk = &SPS_AppData.HkTlm.Payload;

    Hk->Enabled              = SPS_AppData.Enabled ? 1 : 0;
    Hk->QueueCount           = SPS_QueueCount();
    Hk->QueuePeak            = SPS_AppData.QueuePeak;
    Hk->ScheduledTotal       = SPS_AppData.ScheduledTotal;
    Hk->FiredTotal           = SPS_AppData.FiredTotal;
    Hk->RejectedUnknownTotal = SPS_AppData.RejectedUnknownTotal;
    Hk->RejectedFullTotal    = SPS_AppData.RejectedFullTotal;
    Hk->RejectedParamTotal   = SPS_AppData.RejectedParamTotal;
    Hk->LateFiredTotal       = SPS_AppData.LateFiredTotal;
    Hk->FramesEvaluated      = SPS_AppData.FramesEvaluated;
    Hk->LastFiredMsgId       = SPS_AppData.LastFiredMsgId;
    Hk->LastExecTimeS        = SPS_AppData.LastExecTimeS;
    Hk->LastFiredSimTimeS    = SPS_AppData.LastFiredSimTimeS;
    Hk->LastFrameSimTimeS    = SPS_AppData.LastFrameSimTimeS;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(SPS_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(SPS_AppData.HkTlm.TelemetryHeader), true);
}

/* -------------------------------------------------------------------------- */

static void SPS_HandleCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t FcnCode = 0;
    size_t            Size;

    CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);
    CFE_MSG_GetSize(&BufPtr->Msg, &Size);

    /* All SPS_CMD function codes are header-only by contract (sps.xml):
     * a nonzero payload means the sender and this app disagree about the
     * schema, which is a defect to report, not to interpret. */
    if (Size != sizeof(CFE_MSG_CommandHeader_t))
    {
        SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(SPS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "CMD FC=%u bad length %u: ignored",
                          (unsigned int)FcnCode, (unsigned int)Size);
        return;
    }

    switch (FcnCode)
    {
        case 0: /* NOOP */
            SPS_AppData.HkTlm.Payload.CommandCounter++;
            CFE_EVS_SendEvent(SPS_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "SPS No-op command, Version %u.%u.%u, queue %u/%u %s", SPS_APP_VERSION,
                              SPS_APP_REV_MAJOR, SPS_APP_REV_MINOR, (unsigned int)SPS_QueueCount(),
                              (unsigned int)SPS_SLOTS, SPS_AppData.Enabled ? "ARMED" : "FROZEN");
            break;

        case 1: /* ENABLE (arm the queue) */
            SPS_AppData.HkTlm.Payload.CommandCounter++;
            SPS_AppData.Enabled = true;
            CFE_EVS_SendEvent(SPS_ENABLE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "schedule execution ENABLED: %u pending entr%s will fire on frame times >= their T",
                              (unsigned int)SPS_QueueCount(), SPS_QueueCount() == 1 ? "y" : "ies");
            break;

        case 2: /* DISABLE (freeze the queue) */
            SPS_AppData.HkTlm.Payload.CommandCounter++;
            SPS_AppData.Enabled = false;
            CFE_EVS_SendEvent(SPS_DISABLE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "schedule execution DISABLED: %u pending entries hold regardless of their T",
                              (unsigned int)SPS_QueueCount());
            break;

        case 3: /* FLUSH */
        {
            SPS_AppData.HkTlm.Payload.CommandCounter++;
            uint32 Dropped = SPS_QueueCount();
            memset(SPS_AppData.Queue, 0, sizeof(SPS_AppData.Queue));
            SPS_AppData.QueueCount = 0;
            CFE_EVS_SendEvent(SPS_FLUSH_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "schedule queue flushed: %u pending entries dropped", (unsigned int)Dropped);
            break;
        }

        default:
            SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
            CFE_EVS_SendEvent(SPS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "unknown CMD FC=%u: ignored",
                              (unsigned int)FcnCode);
            break;
    }
}

/* -------------------------------------------------------------------------- */

static int32 SPS_AppInit(void)
{
    CFE_Status_t Status;

    memset(&SPS_AppData, 0, sizeof(SPS_AppData));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&SPS_AppData.Pipe, SPS_PIPE_DEPTH, "SPS.Pipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(SPS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe create failed, status=%08X",
                          (unsigned int)Status);
        return Status;
    }

    /* The frame strobe and everything the ground (or scheduler) can say, on
     * one pipe: schedule processing and evaluation are then serialized by SB
     * delivery order, which is exactly the determinism the queue promises. */
    struct
    {
        uint32     Mid;
        const char *What;
    } Subscriptions[] = {
        {BSK_LINK_SENSOR_FRAME_TLM_MID, "sensor frame"},
        {SPS_CMD_MID, "commands"},
        {SPS_SCHEDULE_CMD_MID, "schedule upload"},
        {SPS_SEND_HK_MID, "housekeeping request"},
    };

    for (size_t i = 0; i < sizeof(Subscriptions) / sizeof(Subscriptions[0]); ++i)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(Subscriptions[i].Mid), SPS_AppData.Pipe);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(SPS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "subscribe %s (0x%04X) failed, status=%08X",
                              Subscriptions[i].What, (unsigned int)Subscriptions[i].Mid, (unsigned int)Status);
            return Status;
        }
    }

    CFE_MSG_Init(CFE_MSG_PTR(SPS_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(SPS_HK_TLM_MID),
                 sizeof(SPS_AppData.HkTlm));

    CFE_EVS_SendEvent(SPS_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "SPS App Initialized. Version %u.%u.%u, queue DISABLED, %u slots, %u schedulable commands",
                      SPS_APP_VERSION, SPS_APP_REV_MAJOR, SPS_APP_REV_MINOR, (unsigned int)SPS_SLOTS,
                      (unsigned int)SPS_CMD_COUNT);

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void SPS_AppMain(void)
{
    uint32          RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_Status_t    Status;

    Status = SPS_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, SPS_AppData.Pipe, SPS_SB_TIMEOUT);
            if (Status != CFE_SUCCESS)
            {
                continue; /* timeout: the normal case between 1 Hz frames */
            }

            CFE_SB_MsgId_t         MsgId;
            CFE_SB_MsgId_Atom_t    MsgIdValue;
            size_t                 Size;

            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            MsgIdValue = CFE_SB_MsgIdToValue(MsgId);
            CFE_MSG_GetSize(&BufPtr->Msg, &Size);

            switch (MsgIdValue)
            {
                case BSK_LINK_SENSOR_FRAME_TLM_MID:
                    if (Size == sizeof(BskSensorFrameTlm_t))
                    {
                        const BskSensorFrameTlm_t *Frame = (const BskSensorFrameTlm_t *)BufPtr;
                        SPS_EvaluateQueue(Frame->Payload.SimTimeS);
                    }
                    else if (SPS_AppData.SensorFramesBadLen++ == 0)
                    {
                        CFE_EVS_SendEvent(SPS_FRAME_ERR_EID, CFE_EVS_EventType_ERROR,
                                          "sensor frame length %u, expected %u: ignoring frames", (unsigned int)Size,
                                          (unsigned int)sizeof(BskSensorFrameTlm_t));
                    }
                    break;

                case SPS_CMD_MID:
                    SPS_HandleCmd(BufPtr);
                    break;

                case SPS_SCHEDULE_CMD_MID:
                    SPS_HandleSchedule(BufPtr);
                    break;

                case SPS_SEND_HK_MID:
                    SPS_SendHk();
                    break;

                default:
                    /* Nothing else is subscribed on this pipe; a wrong MsgId
                     * reaching here is a routing defect worth one counter,
                     * not an event storm. */
                    SPS_AppData.HkTlm.Payload.CommandErrorCounter++;
                    break;
            }
        }
    }

    CFE_ES_WriteToSysLog("SPS exit: scheduled=%lu fired=%lu rej(unknown=%lu full=%lu param=%lu) late=%lu frames=%lu\n",
                         (unsigned long)SPS_AppData.ScheduledTotal, (unsigned long)SPS_AppData.FiredTotal,
                         (unsigned long)SPS_AppData.RejectedUnknownTotal, (unsigned long)SPS_AppData.RejectedFullTotal,
                         (unsigned long)SPS_AppData.RejectedParamTotal, (unsigned long)SPS_AppData.LateFiredTotal,
                         (unsigned long)SPS_AppData.FramesEvaluated);
}
