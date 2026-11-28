/*
 * obc_hk: the spacecraft's own housekeeping (see obc_hk_app.h).
 *
 * Passive by design: every field is either a count of a packet this app
 * watched on the bus or a mirror of the freshest report. The app asks
 * nothing of the apps it watches, so its liveness is independent of theirs.
 */

#include <math.h>
#include <string.h>

#include "cfe.h"
#include "obc_hk_app.h"
#include "obc_hk_events.h"
#include "obc_hk_version.h"

OBC_HK_AppData_t OBC_HK_AppData;

/* -------------------------------------------------------------------------- */

static void OBC_HK_SendHk(void)
{
    /* The payload struct IS the image: every handler filled it as traffic
     * arrived, so a report is a stamp and a send - nothing to assemble,
     * nothing to drift. MissionClockS belongs to the frame handler: the
     * reading must come from a frame's wake, not from this send. */
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(OBC_HK_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(OBC_HK_AppData.HkTlm.TelemetryHeader), true);
}

/* -------------------------------------------------------------------------- */

static void OBC_HK_HandleCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t FcnCode = 0;
    size_t            Size;

    CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);
    CFE_MSG_GetSize(&BufPtr->Msg, &Size);

    /* All OBC_HK_CMD function codes are header-only by contract (obc.xml). */
    if (Size != sizeof(CFE_MSG_CommandHeader_t))
    {
        OBC_HK_AppData.CommandErrorCounter++;
        OBC_HK_AppData.HkTlm.Payload.CommandErrorCounter = OBC_HK_AppData.CommandErrorCounter;
        CFE_EVS_SendEvent(OBC_HK_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "CMD FC=%u bad length %u: ignored",
                          (unsigned int)FcnCode, (unsigned int)Size);
        return;
    }

    switch (FcnCode)
    {
        case 0: /* NOOP */
            OBC_HK_AppData.CommandCounter++;
            OBC_HK_AppData.HkTlm.Payload.CommandCounter = OBC_HK_AppData.CommandCounter;
            CFE_EVS_SendEvent(OBC_HK_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "OBC HK No-op, Version %u.%u.%u, frames=%lu/%lu onboard, fcs mode %u",
                              OBC_HK_APP_VERSION, OBC_HK_APP_REV_MAJOR, OBC_HK_APP_REV_MINOR,
                              (unsigned long)OBC_HK_AppData.SensorFramesSeen,
                              (unsigned long)OBC_HK_AppData.OnboardFramesSeen, (unsigned int)OBC_HK_AppData.FcsMode);
            break;

        default:
            OBC_HK_AppData.CommandErrorCounter++;
            OBC_HK_AppData.HkTlm.Payload.CommandErrorCounter = OBC_HK_AppData.CommandErrorCounter;
            CFE_EVS_SendEvent(OBC_HK_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "invalid function code %u",
                              (unsigned int)FcnCode);
            break;
    }
}

/* -------------------------------------------------------------------------- */

static int32 OBC_HK_AppInit(void)
{
    CFE_Status_t Status;

    memset(&OBC_HK_AppData, 0, sizeof(OBC_HK_AppData));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&OBC_HK_AppData.Pipe, OBC_HK_PIPE_DEPTH, "OBC.HkPipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(OBC_HK_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe create failed, status=%08X",
                          (unsigned int)Status);
        return Status;
    }

    /* Everything the OBC can hear about its own state, one pipe. The link
     * frames are knowledge the satellite produced, not truth it was handed:
     * the truth stream never crosses this bus. */
    struct
    {
        uint32     Mid;
        const char *What;
    } Subscriptions[] = {
        {BSK_LINK_SENSOR_FRAME_TLM_MID, "sensor frame"},
        {BSK_LINK_ONBOARD_STATE_TLM_MID, "onboard state"},
        {BSK_LINK_TORQUE_COMMAND_TLM_MID, "torque command"},
        {FCS_HK_TLM_MID, "fcs housekeeping"},
        {OBC_HK_CMD_MID, "commands"},
        {OBC_HK_SEND_HK_MID, "housekeeping request"},
    };

    for (size_t i = 0; i < sizeof(Subscriptions) / sizeof(Subscriptions[0]); ++i)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(Subscriptions[i].Mid), OBC_HK_AppData.Pipe);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(OBC_HK_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "subscribe %s (0x%04X) failed, status=%08X",
                              Subscriptions[i].What, (unsigned int)Subscriptions[i].Mid, (unsigned int)Status);
            return Status;
        }
    }

    CFE_MSG_Init(CFE_MSG_PTR(OBC_HK_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(OBC_HK_TLM_MID),
                 sizeof(OBC_HK_AppData.HkTlm));

    CFE_EVS_SendEvent(OBC_HK_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "OBC HK App Initialized. Version %u.%u.%u, listening on 4 MsgIds", OBC_HK_APP_VERSION,
                      OBC_HK_APP_REV_MAJOR, OBC_HK_APP_REV_MINOR);

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void OBC_HK_AppMain(void)
{
    uint32          RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_Status_t    Status;

    Status = OBC_HK_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, OBC_HK_AppData.Pipe, OBC_HK_SB_TIMEOUT);
            if (Status != CFE_SUCCESS)
            {
                continue; /* timeout: the normal case between 1 Hz frames */
            }

            CFE_SB_MsgId_t MsgId;

            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            size_t Size;
            CFE_MSG_GetSize(&BufPtr->Msg, &Size);

            switch (CFE_SB_MsgIdToValue(MsgId))
            {
                case BSK_LINK_SENSOR_FRAME_TLM_MID:
                    if (Size == sizeof(BskSensorFrameTlm_t))
                    {
                        const BskSensorFrameTlm_t *Frame = (const BskSensorFrameTlm_t *)BufPtr;
                        OBC_HK_AppData.SensorFramesSeen++;
                        OBC_HK_AppData.LastSensorSimTimeS              = Frame->Payload.SimTimeS;
                        OBC_HK_AppData.HkTlm.Payload.SensorFramesSeen  = OBC_HK_AppData.SensorFramesSeen;
                        OBC_HK_AppData.HkTlm.Payload.LastSensorSimTimeS = OBC_HK_AppData.LastSensorSimTimeS;
                        /* Wake-time clock: the mission clock as an observer
                         * of sensor frames reads it, in the frame's own wake.
                         * The link slaves the clock BEFORE publishing the
                         * frame that wakes anyone, so this reading minus the
                         * SimTimeS above is the wake-time skew every frame
                         * consumer - a stored-command evaluator included -
                         * inherits. Zero-ish is the ordering law; the clock
                         * bench measures it, one wake per HK sample. */
                        {
                            CFE_TIME_SysTime_t Now = CFE_TIME_GetTime();
                            OBC_HK_AppData.HkTlm.Payload.MissionClockS =
                                (double)Now.Seconds + (double)Now.Subseconds / 4294967296.0;
                        }
                    }
                    else
                    {
                        OBC_HK_AppData.BadLenCount++;
                    }
                    break;

                case BSK_LINK_ONBOARD_STATE_TLM_MID:
                    if (Size == sizeof(BskOnboardStateTlm_t))
                    {
                        const BskOnboardStateTlm_t *State = (const BskOnboardStateTlm_t *)BufPtr;
                        double                      Field = 0.0;
                        double                      Sun   = 0.0;

                        OBC_HK_AppData.OnboardFramesSeen++;
                        OBC_HK_AppData.LastOnboardSimTimeS = State->Payload.SimTimeS;
                        for (uint8 Axis = 0; Axis < 3; ++Axis)
                        {
                            OBC_HK_AppData.WheelSpeeds[Axis] = State->Payload.WheelSpeeds[Axis];
                            Field += State->Payload.MagB[Axis] * State->Payload.MagB[Axis];
                        }
                        for (uint8 Eye = 0; Eye < 4; ++Eye)
                        {
                            if (State->Payload.CssCos[Eye] > Sun)
                            {
                                Sun = State->Payload.CssCos[Eye];
                            }
                        }
                        OBC_HK_AppData.MagFieldNormT = sqrt(Field);

                        OBC_HK_AppData.HkTlm.Payload.OnboardFramesSeen  = OBC_HK_AppData.OnboardFramesSeen;
                        OBC_HK_AppData.HkTlm.Payload.LastOnboardSimTimeS = OBC_HK_AppData.LastOnboardSimTimeS;
                        memcpy(OBC_HK_AppData.HkTlm.Payload.WheelSpeeds, OBC_HK_AppData.WheelSpeeds,
                               sizeof(OBC_HK_AppData.HkTlm.Payload.WheelSpeeds));
                        OBC_HK_AppData.HkTlm.Payload.SunCosine    = Sun;
                        OBC_HK_AppData.HkTlm.Payload.MagFieldNormT = OBC_HK_AppData.MagFieldNormT;
                    }
                    else
                    {
                        OBC_HK_AppData.BadLenCount++;
                    }
                    break;

                case BSK_LINK_TORQUE_COMMAND_TLM_MID:
                    if (Size == sizeof(BskTorqueCmdTlm_t))
                    {
                        OBC_HK_AppData.TorquePacketsSeen++;
                        OBC_HK_AppData.HkTlm.Payload.TorquePacketsSeen = OBC_HK_AppData.TorquePacketsSeen;
                    }
                    else
                    {
                        OBC_HK_AppData.BadLenCount++;
                    }
                    break;

                case FCS_HK_TLM_MID:
                    /* Mirrored, not interrogated: if fcs stops publishing,
                     * these fields freeze and the freeze IS the diagnosis. */
                    if (Size == sizeof(FcsHkTlm_t))
                    {
                        const FcsHkTlm_t *FcsHk = (const FcsHkTlm_t *)BufPtr;
                        OBC_HK_AppData.FcsMode                          = FcsHk->Payload.Mode;
                        OBC_HK_AppData.FcsTorquesIssued                 = FcsHk->Payload.TorqueCommandsIssued;
                        OBC_HK_AppData.HkTlm.Payload.FcsMode            = OBC_HK_AppData.FcsMode;
                        OBC_HK_AppData.HkTlm.Payload.FcsTorquesIssued   = OBC_HK_AppData.FcsTorquesIssued;
                    }
                    else
                    {
                        OBC_HK_AppData.BadLenCount++;
                    }
                    break;

                case OBC_HK_CMD_MID:
                    OBC_HK_HandleCmd(BufPtr);
                    break;

                case OBC_HK_SEND_HK_MID:
                    OBC_HK_SendHk();
                    break;

                default:
                    /* Nothing else is subscribed; a wrong MsgId here is a
                     * routing defect worth one counter, not an event. */
                    OBC_HK_AppData.BadLenCount++;
                    break;
            }
        }
    }

    CFE_ES_WriteToSysLog("OBC_HK exit: sensor=%lu onboard=%lu torque=%lu fcs_mode=%u\n",
                         (unsigned long)OBC_HK_AppData.SensorFramesSeen,
                         (unsigned long)OBC_HK_AppData.OnboardFramesSeen,
                         (unsigned long)OBC_HK_AppData.TorquePacketsSeen, (unsigned int)OBC_HK_AppData.FcsMode);

    CFE_ES_ExitApp(RunStatus);
}
