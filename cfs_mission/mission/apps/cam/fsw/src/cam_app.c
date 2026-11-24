/*
 * cam: camera payload command surface (see cam_app.h for the contract).
 *
 * Validation lives here because refusal must be a flight-software decision
 * with an event line and a counter, not a sim-side shrug: the ground hears
 * "rejected, here is why" on EVS whether or not the frame would have made it
 * across the airgap.
 */

#include <math.h>
#include <string.h>

#include "cfe.h"
#include "cam_app.h"
#include "cam_events.h"
#include "cam_version.h"

CAM_AppData_t CAM_AppData;

/* -------------------------------------------------------------------------- */

static void CAM_SendHk(void)
{
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
 * deadline - v2a owns no delivery guarantee, and honesty about that is the
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
     * - rate <= 0 or non-finite is B1's divide-by-zero one layer before the
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
