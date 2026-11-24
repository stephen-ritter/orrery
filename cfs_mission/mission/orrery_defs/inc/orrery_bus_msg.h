/*
 * orrery bus structs for the Basilisk link frames and fcs telemetry.
 *
 * Byte layout is owned by the EDS packages (orrery_defs/eds/bsk_link.xml,
 * fcs.xml): COSMOS defs and the EdsLib database are generated from them, and
 * bsk_link moves these exact payload bytes across the ZMQ link. That makes
 * sizeof() a wire contract, so every payload here is packed and pinned:
 * an edit that introduces implicit padding fails this header, not the
 * spacecraft. sim/wire_audit.py re-checks the same lengths on the live wire.
 */
#ifndef ORRERY_BUS_MSG_H
#define ORRERY_BUS_MSG_H

#include <string.h>

#include "cfe_msg_hdr.h"

/* BSK_LINK/SensorFrame: what the satellite's instruments report. */
typedef struct
{
    double   SimTimeS;   /* sim time [s]                        */
    double   OmegaBnB[3]; /* body rate [rad/s]                  */
    uint64   Frame;      /* sim frame counter                   */
} __attribute__((packed)) BskSensorFrame_Payload_t;

/* BSK_LINK/OnboardState: everything else the OBC reads off its buses (P4b).
 * Wheel axes fixed at three, CSS at four - the schema, this struct and the
 * sim's provider all state the same mission hardware. */
typedef struct
{
    double   SimTimeS;     /* sim time [s]                           */
    uint64   Frame;        /* sim frame counter                      */
    double   WheelSpeeds[3]; /* wheel speeds, body-axis set [rad/s]  */
    double   StQuatBn[4];  /* ST quaternion, scalar-first            */
    double   CssCos[4];    /* CSS cosine outputs [-]                 */
    double   MagB[3];      /* measured field, body frame [T]         */
} __attribute__((packed)) BskOnboardState_Payload_t;

/* BSK_LINK/NavState: the satellite's OPINION of its orbit (KN) — a modelled
 * GNSS receiver's fix produced world-side with a seeded error model, the
 * honest sibling of TruthState: the pair must differ (receiver scale) and
 * agree (one world), and only this one may ever touch the flight bus. */
typedef struct
{
    double   SimTimeS;        /* sim time [s]                        */
    uint64   Frame;           /* sim frame counter                   */
    double   RBnN[3];         /* receiver position fix, inertial [m] */
    double   VBnN[3];         /* receiver velocity fix, inertial [m] */
} __attribute__((packed)) BskNavState_Payload_t;

/* BSK_LINK/TorqueCommand: what the flight software commands. */
typedef struct
{
    double   SimTimeS;      /* sim time of the source frame [s] */
    double   TorqueSumB[3]; /* commanded body torque [N m]      */
    uint64   Frame;         /* source frame counter             */
} __attribute__((packed)) BskTorqueCmd_Payload_t;

/* BSK_LINK/StepAck: lockstep barrier ack (D5 amendment). fcs answers every
 * sensor frame it processes; the sim holds the next Basilisk step until the
 * ack for the frame it just published arrives. */
typedef struct
{
    double   SimTimeS;   /* sim time of the acked frame [s]      */
    uint64   Frame;      /* acked sim frame counter              */
    uint8    Mode;       /* fcs mode when acking (0/1/2, P4b-2 adds SAFE)  */
    uint8    AlignPad[3]; /* explicit EDS pad, not a compiler pad */
    uint32   TorquesIssued; /* issued-so-far counter (debug)      */
} __attribute__((packed)) BskStepAck_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    BskStepAck_Payload_t      Payload;
} BskStepAckTlm_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    BskSensorFrame_Payload_t  Payload;
} BskSensorFrameTlm_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t    TelemetryHeader;
    BskOnboardState_Payload_t    Payload;
} BskOnboardStateTlm_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t    TelemetryHeader;
    BskNavState_Payload_t        Payload;
} BskNavStateTlm_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    BskTorqueCmd_Payload_t    Payload;
} BskTorqueCmdTlm_t;

/* FCS ground commands (FC in FCS_CMD_MID, payload in the separate
 * FCS_SET_GAIN_MID - see fcs.xml for why gain upload is its own packet). */
typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} FcsNoopCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} FcsEnableCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} FcsDisableCmd_t;

typedef struct
{
    double Kp[3]; /* per-axis proportional gain [N m per rad/s] */
} __attribute__((packed)) FcsSetGain_Payload_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
    FcsSetGain_Payload_t    Payload;
} FcsSetGainCmd_t;

/* FCS_SET_WHEEL_LIMIT (P4b-2): the supervisor threshold, own packet for the
 * same schema-honest reason as SetGain - an argument needs a real payload. */
typedef struct
{
    double WheelLimit; /* per-wheel |speed| trip threshold [rad/s] */
} __attribute__((packed)) FcsSetWheelLimit_Payload_t;

typedef struct
{
    CFE_MSG_CommandHeader_t    CommandHeader;
    FcsSetWheelLimit_Payload_t Payload;
} FcsSetWheelLimitCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} FcsSendHkCmd_t;

/* FCS housekeeping. Field order and padding mirror FCS/HkTlm_Payload,
 * including the explicit alignment pad the EDS declares (the hk.xml idiom):
 * the wire is sequential, so the pad is a field, not a compiler decision. */
typedef struct
{
    uint8  CommandCounter;
    uint8  CommandErrorCounter;
    uint8  Mode; /* 0 disabled, 1 enabled, 2 safe (P4b-2 supervisor latch) */
    uint8  Unused;
    uint32 SensorFramesReceived;
    uint32 TorqueCommandsIssued;
    uint8  AlignPad[4];
    double LastSimTimeS;
    double LastTorqueB[3];
    double Kp[3];
    uint32 OnboardFramesReceived; /* knowledge frames ingested (P4b-2)   */
    uint8  AlignPad2[4];          /* explicit EDS pad before the doubles */
    double WheelLimit;            /* supervisor trip threshold [rad/s]   */
    double WheelSpeeds[3];        /* latest wheel feedback [rad/s]       */
    double SunCosine;             /* max CSS cosine seen [-]             */
} __attribute__((packed)) FcsHkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    FcsHkTlm_Payload_t        Payload;
} FcsHkTlm_t;

/* CAM ground commands (CAM v2a, cam_app). SET_CONFIG is its own packet on
 * the SetGain law; RenderRateS arrives in the GROUND's seconds and must be
 * > 0 - the app refuses otherwise, because vizInterface's image-request
 * modulo divides by it (B1: zero is a hardware fault, not a config). */
typedef struct
{
    uint8  Shutter;    /* 1 open (request images), 0 closed       */
    uint8  CameraId;   /* the [[camera]] to configure (>= 1)      */
    uint8  AlignPad[6]; /* explicit EDS pad before the double     */
    double RenderRateS; /* image interval [s]; must be > 0         */
} __attribute__((packed)) CamSetConfig_Payload_t;

typedef struct
{
    CFE_MSG_CommandHeader_t  CommandHeader;
    CamSetConfig_Payload_t   Payload;
} CamSetConfigCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} CamNoopCmd_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
} CamSendHkCmd_t;

/* CAM housekeeping: the COMMANDED view only - the sim's confirmed state
 * rides the CamState row, and the ground screens the pair. */
typedef struct
{
    uint8  CommandCounter;
    uint8  CommandErrorCounter;
    uint8  Shutter;             /* last commanded (1 open)              */
    uint8  CameraId;            /* 0 until the first accepted command   */
    uint32 ConfigFramesForwarded;
    uint8  AlignPad[4];         /* explicit EDS pad before the doubles  */
    double LastConfigSimTimeS;  /* sim time of the last frame forwarded */
    double RenderRateCmd;       /* last commanded image interval [s]    */
    uint32 FramesRecorded;      /* sim store outlet sent (CamState, v2b)    */
    uint32 FramesStored;        /* bsk_link completed /cf/img files (v2b)   */
    uint32 FramesTransferred;   /* v2c promise: 0 until that phase lands */
} __attribute__((packed)) CamHkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    CamHkTlm_Payload_t        Payload;
} CamHkTlm_t;

/* BSK_LINK/CamConfig: one accepted camera command, egressed by bsk_link to
 * the sim on the cmd channel under its own topic - never a barrier frame. */
typedef struct
{
    double   SimTimeS;   /* sim time at the command [s]       */
    uint64   Frame;      /* sim frame the command rode         */
    uint64   CameraId;   /* the [[camera]] to configure        */
    uint64   Shutter;    /* 1 open, 0 closed                   */
    double   RenderRateS;/* image interval [s], ground unit    */
} __attribute__((packed)) BskCamConfig_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    BskCamConfig_Payload_t    Payload;
} BskCamConfigTlm_t;

/* BSK_LINK/CamStorage: the onboard image ledger (CAM v2b). bsk_link owns
 * this truth - it is the one that reassembles bytes into /cf/img files -
 * and cam app merely reports it. Divergence from CamState.FramesRecorded
 * (what the sim's outlet sent) is the airgap eating a frame. */
typedef struct
{
    double  SimTimeS;    /* sim time of the newest sensor frame [s] */
    uint64  Frame;       /* sim frame at that stamp                 */
    uint64  FilesStored; /* complete files under /cf/img since boot  */
    uint64  BytesStored; /* payload bytes across those files         */
} __attribute__((packed)) BskCamStorage_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TelemetryHeader;
    BskCamStorage_Payload_t    Payload;
} BskCamStorageTlm_t;

/* BSK_LINK/CamState as C reads it (CAM v2b): the registry row is decoded by
 * EdsLib on the generic path, but cam app wants the store tally on the bus,
 * and a hand-written twin with a size pin beats linking the whole DB for
 * one field. Drift vs bsk_link.xml is a compile error, not a screen lie. */
typedef struct
{
    double  SimTimeS;
    uint64  Frame;
    uint64  CameraId;
    uint64  Shutter;
    double  RenderRateS;
    uint64  Captured;
    uint64  FramesRecorded; /* store outlet sends (CAM v2b) */
} __attribute__((packed)) BskCamState_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    BskCamState_Payload_t     Payload;
} BskCamStateTlm_t;

/* IMG chunk header (CAM v2b): orrery's OWN framing for the image byte
 * stream, deliberately outside the EDS telemetry model (the STEP_ACK
 * precedent for non-ground frames) - packed 24 B header, then the binary
 * slice. The sim's struct.pack("<QQII") mirror lives in sim/forge_scenario;
 * this assert is where the two agree or the build fails. */
typedef struct
{
    uint64 CamId;    /* the [[camera]] that captured it          */
    uint64 Capture;  /* capture ordinal since boot (1-based)     */
    uint32 Part;     /* 0-based index within this capture        */
    uint32 Total;    /* parts in this capture                    */
} __attribute__((packed)) BskImageChunkHeader_t;

/* OBC housekeeping (P4b-3, hk_app). Field order mirrors OBC/HkTlm_Payload,
 * pads included: everything here is bus-observed counts and freshest bus
 * reports - the OBC's view of itself, never navigator truth. */
typedef struct
{
    uint8  CommandCounter;
    uint8  CommandErrorCounter;
    uint8  FcsMode;           /* from the latest FCS_HK_TLM (0/1/2)          */
    uint8  Unused;
    uint32 SensorFramesSeen;  /* frames hk_app itself watched on the bus    */
    uint32 OnboardFramesSeen;
    uint32 TorquePacketsSeen;
    uint32 FcsTorquesIssued;  /* issued counter mirrored from fcs HK         */
    uint8  AlignPad[4];
    double LastSensorSimTimeS;
    double LastOnboardSimTimeS;
    double WheelSpeeds[3];    /* latest wheel feedback [rad/s]               */
    double SunCosine;         /* max CSS cosine seen [-]                     */
    double MagFieldNormT;     /* |B| from the magnetometer [T]               */
} __attribute__((packed)) ObcHk_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    ObcHk_Payload_t           Payload;
} ObcHkTlm_t;

/* Time-tagged command execution (P6, sps app). The schedule envelope is a
 * complete command image - target MsgId + function code + verbatim payload
 * bytes - so anything the EDS dictionary knows is schedulable. */
typedef struct
{
    double ExecTimeS;   /* execute on the first sensor frame with time >= T [s] */
    uint16 CmdMsgId;    /* command to execute (validated against the EDS table) */
    uint8  CmdFc;       /* its function code                                    */
    uint8  CmdDataLen;  /* payload bytes carried in CmdData                     */
    double CmdData[6];  /* command payload verbatim (unused slots zero)         */
} __attribute__((packed)) SpsSchedule_Payload_t;

typedef struct
{
    CFE_MSG_CommandHeader_t CommandHeader;
    SpsSchedule_Payload_t   Payload;
} SpsScheduleCmd_t;

/* sps housekeeping: queue state, lifetime counters, firing evidence.
 * Field order mirrors SPS/HkTlm_Payload including the EDS alignment pad. */
typedef struct
{
    uint8  CommandCounter;
    uint8  CommandErrorCounter;
    uint8  Enabled; /* 0 armed-but-frozen, 1 evaluating on frames */
    uint8  Unused;
    uint32 QueueCount;
    uint32 QueuePeak;
    uint32 ScheduledTotal;
    uint32 FiredTotal;
    uint32 RejectedUnknownTotal; /* MsgId/Fc not in the command dictionary */
    uint32 RejectedFullTotal;    /* queue at capacity                      */
    uint32 RejectedParamTotal;   /* bad length or self-schedule            */
    uint32 LateFiredTotal;       /* T already passed at upload             */
    uint32 FramesEvaluated;
    uint32 LastFiredMsgId;
    uint8  AlignPad[4];
    double LastExecTimeS;    /* requested T of the most recent firing   */
    double LastFiredSimTimeS; /* sim time of the frame that carried it   */
    double LastFrameSimTimeS; /* sim time of the latest evaluated frame  */
} __attribute__((packed)) SpsHkTlm_Payload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    SpsHkTlm_Payload_t        Payload;
} SpsHkTlm_t;

/* Compile-time wire contract: payload sizes exactly as codegen emits them in
 * the COSMOS defs (sensor/torque 40 B payloads, fcs hk 120 B with its pads). */
_Static_assert(sizeof(BskSensorFrame_Payload_t) == 40, "SensorFrame payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskOnboardState_Payload_t) == 128, "OnboardState payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskNavState_Payload_t) == 64, "NavState payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskTorqueCmd_Payload_t) == 40, "TorqueCommand payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskStepAck_Payload_t) == 24, "StepAck payload drift vs bsk_link.xml");
_Static_assert(sizeof(FcsSetGain_Payload_t) == 24, "SetGain payload drift vs fcs.xml");
_Static_assert(sizeof(FcsSetWheelLimit_Payload_t) == 8, "SetWheelLimit payload drift vs fcs.xml");
_Static_assert(sizeof(FcsHkTlm_Payload_t) == 120, "FCS HK payload drift vs fcs.xml");
_Static_assert(sizeof(ObcHk_Payload_t) == 80, "OBC HK payload drift vs obc.xml");
_Static_assert(sizeof(SpsSchedule_Payload_t) == 60, "Schedule payload drift vs sps.xml");
_Static_assert(sizeof(SpsHkTlm_Payload_t) == 72, "SPS HK payload drift vs sps.xml");
_Static_assert(sizeof(CamSetConfig_Payload_t) == 16, "SetConfig payload drift vs cam.xml");
_Static_assert(sizeof(CamHkTlm_Payload_t) == 40, "CAM HK payload drift vs cam.xml");
_Static_assert(sizeof(BskCamConfig_Payload_t) == 40, "CamConfig payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskCamStorage_Payload_t) == 32, "CamStorage payload drift vs bsk_link.xml");
_Static_assert(sizeof(BskCamState_Payload_t) == 56, "CamState payload drift vs bsk_link.xml row");
_Static_assert(sizeof(BskImageChunkHeader_t) == 24, "IMG chunk header drift vs sim framing");

#endif /* ORRERY_BUS_MSG_H */
