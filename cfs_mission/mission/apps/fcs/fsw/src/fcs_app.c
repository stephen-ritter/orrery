/*
 * fcs: rate-damping flight control (see fcs_app.h for the contract).
 *
 * Event-driven where it matters: the control step runs the moment a sensor
 * frame arrives, not on a timer, so the loop delay is one link hop plus one
 * SB hop regardless of the scheduler phase.
 */

#include <math.h>
#include <string.h>

#include "cfe.h"
#include "cf_msgids.h"
#include "fcs_app.h"
#include "fcs_events.h"
#include "fcs_version.h"

/* Default gains mirror KP_RADS in sim/detumble_scenario.py: per-axis closed
 * loop tau = I/Kp near 12 s on this spacecraft, far under the Kp at which a
 * 1 Hz update goes unstable. Ground can retune live with FCS_SET_GAIN; the
 * defaults exist so an ENABLE with no upload is a known, tested state. */
static const double FCS_DEFAULT_KP[FCS_AXES] = {60.0, 50.0, 45.0};

FCS_AppData_t FCS_AppData;

/* -------------------------------------------------------------------------- */

static void FCS_SendHk(void)
{
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(FCS_AppData.HkTlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(FCS_AppData.HkTlm.TelemetryHeader), true);
}

/* -------------------------------------------------------------------------- */

static const char *FCS_ModeName(FCS_Mode_t Mode)
{
    switch (Mode)
    {
        case FCS_MODE_ENABLED:
            return "ENABLED";
        case FCS_MODE_SAFE:
            return "SAFE";
        default:
            return "DISABLED";
    }
}

/**
 * Is the onboard knowledge fresh enough to judge the actuators NOW?
 *
 * Fresh means: at least one snapshot, and its frame within a few sensor
 * frames of the one being processed (the rig publishes ONBOARD right after
 * SENSOR, so a live link sits at age 1). A plant that never publishes
 * onboard state - the hand-written witness rig - keeps this permanently
 * false: the supervisor stays unarmed, and old knowledge about a restarted
 * sim (frame watermarks reset) never trips a modern law.
 */
static bool FCS_OnboardFresh(uint64 SensorFrame)
{
    if (FCS_AppData.OnboardFramesReceived == 0)
    {
        return false;
    }
    if (SensorFrame < FCS_AppData.OnboardFrame)
    {
        return false; /* snapshot from a newer session: not evidence about now */
    }
    return (SensorFrame - FCS_AppData.OnboardFrame) <= FCS_ONBOARD_MAX_AGE;
}

/** Largest |wheel speed| in the current knowledge snapshot. */
static double FCS_WorstWheelAbs(void)
{
    double Worst = 0.0;

    for (uint8 Axis = 0; Axis < FCS_AXES; ++Axis)
    {
        double Magnitude = fabs(FCS_AppData.OnboardWheelSpeeds[Axis]);
        if (Magnitude > Worst)
        {
            Worst = Magnitude;
        }
    }
    return Worst;
}

static bool FCS_WheelsAtLimit(void)
{
    return FCS_WorstWheelAbs() >= FCS_AppData.WheelLimit;
}

/* -------------------------------------------------------------------------- */

/**
 * The lockstep barrier ack (D5 amendment): every processed sensor frame gets
 * one, whatever the mode. The sim blocks its next Basilisk step until the ack
 * carrying the frame number it just published arrives on the link, so the ack
 * is where "the satellite saw frame N" stops being an assumption.
 */
static void FCS_StepAck(const BskSensorFrame_Payload_t *Sensor)
{
    BskStepAckTlm_t Ack;
    CFE_Status_t    Status;

    CFE_MSG_Init(CFE_MSG_PTR(Ack.TelemetryHeader), CFE_SB_ValueToMsgId(BSK_LINK_STEP_ACK_TLM_MID), sizeof(Ack));
    Ack.Payload.SimTimeS      = Sensor->SimTimeS;
    Ack.Payload.Frame         = Sensor->Frame;
    Ack.Payload.Mode          = (uint8)FCS_AppData.Mode;
    Ack.Payload.TorquesIssued = FCS_AppData.TorqueCommandsIssued;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Ack.TelemetryHeader));
    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Ack.TelemetryHeader), true);
    if (Status == CFE_SUCCESS)
    {
        FCS_AppData.StepAcksIssued++;
    }
    else if (FCS_AppData.StepAckFailCount == 0)
    {
        /* Say it once: the sim is about to hang at its gate, and this event
         * is the difference between "why did the scenario abort?" and a
         * five-minute read of the roadmap. */
        CFE_EVS_SendEvent(FCS_STEP_ACK_ERR_EID, CFE_EVS_EventType_ERROR, "step ack publish rejected: 0x%08X",
                          (unsigned int)Status);
        FCS_AppData.StepAckFailCount++;
    }
    else
    {
        FCS_AppData.StepAckFailCount++;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * One control step: measured rate in, commanded torque out, barrier ack last.
 *
 * SimTimeS and Frame pass through from the sensor frame, so the sim can match
 * every applied torque to the knowledge that produced it and stale commands
 * are visible in data rather than assumed away.
 */
static void FCS_ControlStep(const BskSensorFrame_Payload_t *Sensor)
{
    BskTorqueCmdTlm_t Torque;
    CFE_Status_t      Status;

    /* Barrier idempotency (D5 amendment): the sim retransmits sensor{N} while
     * it waits, because a fresh ZMQ PUB/SUB pair eats the first published
     * frame - and a lockstep sim that loses frame 1 must not deadlock, it
     * must shout until the shout lands. An already-processed frame number is
     * therefore re-acked WITHOUT re-running the law: the control law and
     * every counter here stay exactly once-per-frame. Frame numbers rise by
     * one within a sim session, so a LOWER number is no link accident - it
     * means the sim restarted against a warm mission; the watermark resets
     * and the new session runs on its own terms. */
    if (Sensor->Frame == FCS_AppData.LastFrameNumber)
    {
        FCS_AppData.StepAckResends++;
        FCS_StepAck(Sensor);
        return;
    }
    if (Sensor->Frame < FCS_AppData.LastFrameNumber)
    {
        if (FCS_AppData.SensorFramesReceived != 0)
        {
            CFE_EVS_SendEvent(FCS_SESSION_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "basilisk session restarted: frame %llu after %llu",
                              (unsigned long long)Sensor->Frame,
                              (unsigned long long)FCS_AppData.LastFrameNumber);
        }
        FCS_AppData.SensorFramesReceived = 0;
        FCS_AppData.TorqueCommandsIssued = 0;
        FCS_AppData.HkTlm.Payload.SensorFramesReceived = 0;
        FCS_AppData.HkTlm.Payload.TorqueCommandsIssued = 0;
    }
    FCS_AppData.LastFrameNumber = Sensor->Frame;

    FCS_AppData.SensorFramesReceived++;
    FCS_AppData.LastSimTimeS = Sensor->SimTimeS;

    /* Supervisor first (P4b-2): the frame that proves a wheel is at its
     * limit must not also receive torque from a law that pushed it there.
     * Tripping stops the law, not the ack below - the sim's gate and the
     * control authority are deliberately independent. */
    if (FCS_AppData.Mode == FCS_MODE_ENABLED && FCS_OnboardFresh(Sensor->Frame) && FCS_WheelsAtLimit())
    {
        FCS_AppData.Mode               = FCS_MODE_SAFE;
        FCS_AppData.ModeTrips++;
        FCS_AppData.HkTlm.Payload.Mode = (uint8)FCS_MODE_SAFE;
        CFE_EVS_SendEvent(FCS_MODE_SAFE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "wheel %+.1f rad/s at limit %.1f (t=%.1fs frame=%llu): ENABLED->SAFE, torque stopped, "
                          "ground recovery required",
                          FCS_WorstWheelAbs(), FCS_AppData.WheelLimit, Sensor->SimTimeS,
                          (unsigned long long)Sensor->Frame);
    }

    if (FCS_AppData.Mode == FCS_MODE_ENABLED)
    {
        CFE_MSG_Init(CFE_MSG_PTR(Torque.TelemetryHeader), CFE_SB_ValueToMsgId(BSK_LINK_TORQUE_COMMAND_TLM_MID),
                     sizeof(Torque));
        Torque.Payload.SimTimeS = Sensor->SimTimeS;
        Torque.Payload.Frame    = Sensor->Frame;
        for (uint8 Axis = 0; Axis < FCS_AXES; ++Axis)
        {
            Torque.Payload.TorqueSumB[Axis] = -FCS_AppData.Kp[Axis] * Sensor->OmegaBnB[Axis];
            FCS_AppData.LastTorqueB[Axis]   = Torque.Payload.TorqueSumB[Axis];
        }

        CFE_SB_TimeStampMsg(CFE_MSG_PTR(Torque.TelemetryHeader));
        Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Torque.TelemetryHeader), true);
        if (Status == CFE_SUCCESS)
        {
            FCS_AppData.TorqueCommandsIssued++;
            FCS_AppData.HkTlm.Payload.TorqueCommandsIssued = FCS_AppData.TorqueCommandsIssued;
        }
        else if (FCS_AppData.PublishFailCount == 0)
        {
            CFE_EVS_SendEvent(FCS_TORQUE_ERR_EID, CFE_EVS_EventType_ERROR, "torque publish rejected: 0x%08X",
                              (unsigned int)Status);
        }
        if (Status != CFE_SUCCESS)
        {
            FCS_AppData.PublishFailCount++;
        }
    }

    /* The gate answers disabled runs too: an inert satellite still proves it
     * saw the frame, which is exactly what lets a lockstep sim keep stepping. */
    FCS_StepAck(Sensor);
}

/* -------------------------------------------------------------------------- */

static void FCS_HandleSetGain(const CFE_SB_Buffer_t *BufPtr)
{
    const FcsSetGainCmd_t *Cmd = (const FcsSetGainCmd_t *)BufPtr;
    size_t                 Size;
    double                 Candidate[FCS_AXES];
    bool                   Acceptable = true;

    CFE_MSG_GetSize(&BufPtr->Msg, &Size);
    if (Size != sizeof(FcsSetGainCmd_t))
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SET_GAIN length %u, expected %u: ignored", (unsigned int)Size,
                          (unsigned int)sizeof(FcsSetGainCmd_t));
        return;
    }

    for (uint8 Axis = 0; Axis < FCS_AXES; ++Axis)
    {
        Candidate[Axis] = Cmd->Payload.Kp[Axis];
        /* Reject NaN/Inf and nonsense magnitudes: a NaN gain would silently
         * poison every future torque frame, and an NaN in a float telemetry
         * field is a miserable thing to debug from the ground. */
        if (!isfinite(Candidate[Axis]) || Candidate[Axis] <= 0.0 || Candidate[Axis] > FCS_KP_MAX)
        {
            Acceptable = false;
        }
    }

    if (!Acceptable)
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_SET_GAIN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SET_GAIN rejected (%+.3g %+.3g %+.3g): need finite, 0 < Kp <= %g", Candidate[0],
                          Candidate[1], Candidate[2], FCS_KP_MAX);
        return;
    }

    memcpy(FCS_AppData.Kp, Candidate, sizeof(FCS_AppData.Kp));
    memcpy(FCS_AppData.HkTlm.Payload.Kp, FCS_AppData.Kp, sizeof(FCS_AppData.Kp));
    FCS_AppData.HkTlm.Payload.CommandCounter++;
    CFE_EVS_SendEvent(FCS_SET_GAIN_INF_EID, CFE_EVS_EventType_INFORMATION, "gain set: Kp=%+.3g %+.3g %+.3g",
                      FCS_AppData.Kp[0], FCS_AppData.Kp[1], FCS_AppData.Kp[2]);
}

/* -------------------------------------------------------------------------- */

static void FCS_HandleSetWheelLimit(const CFE_SB_Buffer_t *BufPtr)
{
    const FcsSetWheelLimitCmd_t *Cmd = (const FcsSetWheelLimitCmd_t *)BufPtr;
    size_t                       Size;
    double                       Candidate;

    CFE_MSG_GetSize(&BufPtr->Msg, &Size);
    if (Size != sizeof(FcsSetWheelLimitCmd_t))
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "SET_WHEEL_LIMIT length %u, expected %u: ignored",
                          (unsigned int)Size, (unsigned int)sizeof(FcsSetWheelLimitCmd_t));
        return;
    }

    Candidate = Cmd->Payload.WheelLimit;
    /* Same poison logic as gains: the threshold is compared against floats
     * off the link every frame, and a NaN would make the supervisor fire
     * never - or always - with no visible cause. */
    if (!isfinite(Candidate) || Candidate <= 0.0 || Candidate > FCS_WHEEL_LIMIT_MAX)
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_SET_LIMIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "SET_WHEEL_LIMIT rejected (%+.3g): need finite, 0 < limit <= %g rad/s", Candidate,
                          FCS_WHEEL_LIMIT_MAX);
        return;
    }

    FCS_AppData.WheelLimit               = Candidate;
    FCS_AppData.HkTlm.Payload.WheelLimit = Candidate;
    FCS_AppData.HkTlm.Payload.CommandCounter++;
    CFE_EVS_SendEvent(FCS_SET_LIMIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "wheel trip threshold set: %.1f rad/s (supervisor active once onboard state flows)",
                      Candidate);
}

/* -------------------------------------------------------------------------- */

/* CFDP downlink request: relay to CF's own TX_FILE, native layout
 * (measured law: CF reads its command payload in machine endianness, and the
 * CF-side build of the PDU is all CF's). FCS never learns CFDP - it carries
 * one file's worth of bytes into a struct CF published, keeps its own file
 * law (keep=1: a request may not delete craft files), and counts what it
 * handed over. Whether the file exists or transfers is CF's testimony, not
 * a second opinion here. */
static void FCS_HandleCfdpDown(const CFE_SB_Buffer_t *BufPtr)
{
    const FcsCfdpDownCmd_t *Cmd = (const FcsCfdpDownCmd_t *)BufPtr;
    CfTxFileCmd_t              Tx;
    CFE_Status_t               Status;
    size_t                     Size;

    /* the request's fixed names and CF's fixed names are the same law: if
     * either mirror is resized, this build fails here, not on the wire. */
    _Static_assert(sizeof(((const FcsCfdpDownCmd_t *)NULL)->Payload.SrcPath) ==
                       sizeof(((const CfTxFileCmd_t *)NULL)->Payload.SrcFilename),
                   "CFDP_DOWN path width drifted from the CF TX_FILE mirror");

    CFE_MSG_GetSize(&BufPtr->Msg, &Size);
    if (Size != sizeof(FcsCfdpDownCmd_t))
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CFDP_DOWN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "CFDP_DOWN length %u, expected %u: ignored", (unsigned int)Size,
                          (unsigned int)sizeof(FcsCfdpDownCmd_t));
        return;
    }
    if (Cmd->Payload.CfdpClass > 1 || Cmd->Payload.SrcPath[0] == '\0' || Cmd->Payload.DstName[0] == '\0')
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CFDP_DOWN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "CFDP_DOWN rejected: class %u, empty path or name", (unsigned int)Cmd->Payload.CfdpClass);
        return;
    }

    memset(&Tx, 0, sizeof(Tx));
    CFE_MSG_Init(CFE_MSG_PTR(Tx.CommandHeader), CFE_SB_ValueToMsgId(CF_CMD_MID), sizeof(Tx));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Tx.CommandHeader), CF_TX_FILE_CC);
    Tx.Payload.CfdpClass = Cmd->Payload.CfdpClass;
    Tx.Payload.Keep      = 1; /* the craft keeps its own files: always */
    Tx.Payload.ChanNum   = 0;
    Tx.Payload.Priority  = 0;
    Tx.Payload.DestId    = Cmd->Payload.DestEntityId;
    memcpy(Tx.Payload.SrcFilename, Cmd->Payload.SrcPath, sizeof(Tx.Payload.SrcFilename));
    memcpy(Tx.Payload.DstFilename, Cmd->Payload.DstName, sizeof(Tx.Payload.DstFilename));

    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Tx.CommandHeader), true);
    if (Status != CFE_SUCCESS)
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CFDP_DOWN_ERR_EID, CFE_EVS_EventType_ERROR, "CFDP_DOWN relay rejected: 0x%08X",
                          (unsigned int)Status);
        return;
    }

    FCS_AppData.HkTlm.Payload.CommandCounter++;
    CFE_EVS_SendEvent(FCS_CFDP_DOWN_INF_EID, CFE_EVS_EventType_INFORMATION, "CFDP down: %s as %s -> entity %lu, class %u",
                      Tx.Payload.SrcFilename, Tx.Payload.DstFilename, (unsigned long)Cmd->Payload.DestEntityId,
                      (unsigned int)Cmd->Payload.CfdpClass);
}

static void FCS_HandleCmd(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t FcnCode = 0;
    size_t            Size;

    CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);
    CFE_MSG_GetSize(&BufPtr->Msg, &Size);

    /* All FCS_CMD function codes are header-only by contract (fcs.xml):
     * a nonzero payload means the sender and this app disagree about the
     * schema, which is a defect to report, not to interpret. */
    if (Size != sizeof(FcsNoopCmd_t))
    {
        FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
        CFE_EVS_SendEvent(FCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "CMD FC=%u bad length %u: ignored",
                          (unsigned int)FcnCode, (unsigned int)Size);
        return;
    }

    switch (FcnCode)
    {
        case 0: /* NOOP */
            FCS_AppData.HkTlm.Payload.CommandCounter++;
            CFE_EVS_SendEvent(FCS_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "FCS No-op command, Version %u.%u.%u, mode %s", FCS_APP_VERSION, FCS_APP_REV_MAJOR,
                              FCS_APP_REV_MINOR, FCS_ModeName(FCS_AppData.Mode));
            break;

        case 1: /* ENABLE */
            /* A SAFE latch is a statement about the actuators, not a mode to
             * cycle out of: while the freshest knowledge still shows a wheel
             * at the limit, ENABLE is refused. Ground raises the limit or
             * waits for momentum management - recovery is a decision, not a
             * race with the supervisor. (A stale snapshot never refuses:
             * absence of knowledge is not knowledge of a fault.) */
            if (FCS_AppData.Mode == FCS_MODE_SAFE && FCS_OnboardFresh(FCS_AppData.LastFrameNumber) &&
                FCS_WheelsAtLimit())
            {
                FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
                CFE_EVS_SendEvent(FCS_ENABLE_REJ_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "ENABLE refused in SAFE: wheel %+.1f rad/s still at limit %.1f; raise "
                                  "FCS_SET_WHEEL_LIMIT or manage momentum first",
                                  FCS_WorstWheelAbs(), FCS_AppData.WheelLimit);
                break;
            }
            FCS_AppData.HkTlm.Payload.CommandCounter++;
            FCS_AppData.Mode               = FCS_MODE_ENABLED;
            FCS_AppData.HkTlm.Payload.Mode = (uint8)FCS_MODE_ENABLED;
            CFE_EVS_SendEvent(FCS_ENABLE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "control loop ENABLED: torque = -Kp * omega, Kp=%+.3g %+.3g %+.3g", FCS_AppData.Kp[0],
                              FCS_AppData.Kp[1], FCS_AppData.Kp[2]);
            break;

        case 2: /* DISABLE */
            FCS_AppData.HkTlm.Payload.CommandCounter++;
            FCS_AppData.Mode               = FCS_MODE_DISABLED;
            FCS_AppData.HkTlm.Payload.Mode = (uint8)FCS_MODE_DISABLED;
            CFE_EVS_SendEvent(FCS_DISABLE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "control loop DISABLED: sensor frames still ingested, no torque commanded");
            break;

        default:
            FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
            CFE_EVS_SendEvent(FCS_CMD_ERR_EID, CFE_EVS_EventType_ERROR, "invalid function code %u",
                              (unsigned int)FcnCode);
            break;
    }
}

/* -------------------------------------------------------------------------- */

static int32 FCS_AppInit(void)
{
    CFE_Status_t Status;

    memset(&FCS_AppData, 0, sizeof(FCS_AppData));
    FCS_AppData.Mode       = FCS_MODE_DISABLED;
    FCS_AppData.WheelLimit = FCS_WHEEL_LIMIT_DEFAULT;
    memcpy(FCS_AppData.Kp, FCS_DEFAULT_KP, sizeof(FCS_AppData.Kp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&FCS_AppData.Pipe, FCS_PIPE_DEPTH, "FCS.Pipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(FCS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "pipe create failed, status=%08X",
                          (unsigned int)Status);
        return Status;
    }

    /* Control input and everything the ground (or scheduler) can say, on one
     * pipe: at this message rate separation would buy latency, not clarity. */
    struct
    {
        uint32     Mid;
        const char *What;
    } Subscriptions[] = {
        {BSK_LINK_SENSOR_FRAME_TLM_MID, "sensor frame"},
        {BSK_LINK_ONBOARD_STATE_TLM_MID, "onboard state"},
        {FCS_CMD_MID, "commands"},
        {FCS_SET_GAIN_MID, "gain upload"},
        {FCS_SET_WHEEL_LIMIT_MID, "wheel limit upload"},
        {FCS_CFDP_DOWN_MID, "file downlink request"},
        {FCS_SEND_HK_MID, "housekeeping request"},
    };

    for (size_t i = 0; i < sizeof(Subscriptions) / sizeof(Subscriptions[0]); ++i)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(Subscriptions[i].Mid), FCS_AppData.Pipe);
        if (Status != CFE_SUCCESS)
        {
            CFE_EVS_SendEvent(FCS_PIPE_ERR_EID, CFE_EVS_EventType_ERROR, "subscribe %s (0x%04X) failed, status=%08X",
                              Subscriptions[i].What, (unsigned int)Subscriptions[i].Mid, (unsigned int)Status);
            return Status;
        }
    }

    /* Housekeeping: initialized once, like the stock apps, so the sequence
     * count flows; counters live in the payload so they cannot drift from
     * what the beacon shows. */
    CFE_MSG_Init(CFE_MSG_PTR(FCS_AppData.HkTlm.TelemetryHeader), CFE_SB_ValueToMsgId(FCS_HK_TLM_MID),
                 sizeof(FCS_AppData.HkTlm));
    FCS_AppData.HkTlm.Payload.Mode = (uint8)FCS_AppData.Mode;
    memcpy(FCS_AppData.HkTlm.Payload.Kp, FCS_AppData.Kp, sizeof(FCS_AppData.Kp));
    FCS_AppData.HkTlm.Payload.WheelLimit = FCS_AppData.WheelLimit;

    CFE_EVS_SendEvent(FCS_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "FCS App Initialized. Version %u.%u.%u, mode DISABLED, Kp=%+.3g %+.3g %+.3g", FCS_APP_VERSION,
                      FCS_APP_REV_MAJOR, FCS_APP_REV_MINOR, FCS_AppData.Kp[0], FCS_AppData.Kp[1], FCS_AppData.Kp[2]);

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void FCS_AppMain(void)
{
    uint32          RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_Status_t    Status;

    Status = FCS_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, FCS_AppData.Pipe, FCS_SB_TIMEOUT);
            if (Status != CFE_SUCCESS)
            {
                continue; /* timeout: the normal case between 1 Hz frames */
            }

            CFE_SB_MsgId_t    MsgId;
            CFE_SB_MsgId_Atom_t MsgIdValue;
            size_t            Size;

            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            MsgIdValue = CFE_SB_MsgIdToValue(MsgId);
            CFE_MSG_GetSize(&BufPtr->Msg, &Size);

            switch (MsgIdValue)
            {
                case BSK_LINK_SENSOR_FRAME_TLM_MID:
                    if (Size == sizeof(BskSensorFrameTlm_t))
                    {
                        const BskSensorFrameTlm_t *Frame = (const BskSensorFrameTlm_t *)BufPtr;
                        FCS_ControlStep(&Frame->Payload);
                        FCS_AppData.HkTlm.Payload.SensorFramesReceived = FCS_AppData.SensorFramesReceived;
                        FCS_AppData.HkTlm.Payload.LastSimTimeS         = FCS_AppData.LastSimTimeS;
                        memcpy(FCS_AppData.HkTlm.Payload.LastTorqueB, FCS_AppData.LastTorqueB,
                               sizeof(FCS_AppData.HkTlm.Payload.LastTorqueB));
                    }
                    else
                    {
                        FCS_AppData.SensorFramesBadLen++;
                    }
                    break;

                case BSK_LINK_ONBOARD_STATE_TLM_MID:
                    /* Knowledge snapshot for the supervisor + HK window into
                     * what the controller knows. Sampled, never law input:
                     * the rate law stays on SensorFrame (frame-exact law
                     * acceptance untouched). */
                    if (Size == sizeof(BskOnboardStateTlm_t))
                    {
                        const BskOnboardStateTlm_t *State = (const BskOnboardStateTlm_t *)BufPtr;
                        double                      Sun   = 0.0;

                        FCS_AppData.OnboardFramesReceived++;
                        FCS_AppData.OnboardFrame = State->Payload.Frame;
                        memcpy(FCS_AppData.OnboardWheelSpeeds, State->Payload.WheelSpeeds,
                               sizeof(FCS_AppData.OnboardWheelSpeeds));
                        for (uint8 Eye = 0; Eye < 4; ++Eye)
                        {
                            /* Largest cosine wins: "is there sun anywhere"
                             * is the question HK should answer, and shadows
                             * on three eyes are not news. */
                            if (State->Payload.CssCos[Eye] > Sun)
                            {
                                Sun = State->Payload.CssCos[Eye];
                            }
                        }
                        FCS_AppData.OnboardSunCosine = Sun;

                        FCS_AppData.HkTlm.Payload.OnboardFramesReceived = FCS_AppData.OnboardFramesReceived;
                        memcpy(FCS_AppData.HkTlm.Payload.WheelSpeeds, FCS_AppData.OnboardWheelSpeeds,
                               sizeof(FCS_AppData.HkTlm.Payload.WheelSpeeds));
                        FCS_AppData.HkTlm.Payload.SunCosine = Sun;
                    }
                    else
                    {
                        FCS_AppData.OnboardBadLen++;
                    }
                    break;

                case FCS_CMD_MID:
                    FCS_HandleCmd(BufPtr);
                    break;

                case FCS_SET_GAIN_MID:
                    FCS_HandleSetGain(BufPtr);
                    break;

                case FCS_SET_WHEEL_LIMIT_MID:
                    FCS_HandleSetWheelLimit(BufPtr);
                    break;

                case FCS_CFDP_DOWN_MID:
                    FCS_HandleCfdpDown(BufPtr);
                    break;

                case FCS_SEND_HK_MID:
                    FCS_SendHk();
                    break;

                default:
                    /* Nothing else is subscribed on this pipe; a wrong MsgId
                     * reaching here is a routing defect worth one counter,
                     * not an event storm. */
                    FCS_AppData.HkTlm.Payload.CommandErrorCounter++;
                    break;
            }
        }
    }

    CFE_ES_WriteToSysLog("FCS exit: frames=%lu torque=%lu acks=%lu resends=%lu onboard=%lu trips=%u mode=%u\n",
                         (unsigned long)FCS_AppData.SensorFramesReceived,
                         (unsigned long)FCS_AppData.TorqueCommandsIssued,
                         (unsigned long)FCS_AppData.StepAcksIssued,
                         (unsigned long)FCS_AppData.StepAckResends, (unsigned long)FCS_AppData.OnboardFramesReceived,
                         (unsigned)FCS_AppData.ModeTrips, (unsigned int)FCS_AppData.Mode);

    CFE_ES_ExitApp(RunStatus);
}
