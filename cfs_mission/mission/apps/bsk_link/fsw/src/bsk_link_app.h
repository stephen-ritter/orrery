/*
 * bsk_link application global data and constants.
 */

#ifndef BSK_LINK_APP_H
#define BSK_LINK_APP_H

#include <stdbool.h>

#include "cfe.h"
#include "bsk_link_eds_defines.h"
#include "orrery_bus_msg.h"
#include "orrery_frames.h"

/**
 * \name Configuration
 * \{
 */

/** Basilisk scenario ZMQ sensor feed (Basilisk binds PUB; see protos/README.md).
 *  The mission container shares the host network namespace, so loopback reaches
 *  a scenario running on the host. */
#define BSK_LINK_ZMQ_ENDPOINT "tcp://127.0.0.1:5580"

/** Basilisk scenario command sink (Basilisk binds PULL, sim/detumble_scenario.py).
 *  This app is the only writer: fcs torque commands leave cFS through here. */
#define BSK_LINK_ZMQ_CMD_ENDPOINT "tcp://127.0.0.1:5581"

/** ZMQ topic of the sensor feed (first multipart frame; see sim/detumble_scenario.py).
 *  The sensor frame is the CONTROL path (D5 barrier witness), so it stays
 *  hand-wired here; every OBSERVABILITY frame (ONBOARD, NAVSTATE, ...) is a
 *  registry row in orrery_frames.h and needs no define in this file. */
#define BSK_LINK_SENSOR_TOPIC "SENSOR"
/** Boot handshake (OB): the world's declared part fit, not telemetry.
 *  Its container has no TopicId on purpose - no MsgId, no ground definition,
 *  no beacon line. A handshake that reached the ground would tempt someone to
 *  grade the ground's opinion of the fit instead of the spacecraft's. */
#define BSK_LINK_FIT_TOPIC "FIT"

/** ZMQ topic of the commanded torque (egress; the sim decodes it with the same DB) */
#define BSK_LINK_CMD_TOPIC "CMD"

/** ZMQ topic of the lockstep barrier ack (egress; D5 amendment). fcs answers
 *  every processed frame; the sim holds its next Basilisk step for this. */
#define BSK_LINK_ACK_TOPIC "ACK"

/** ZMQ topic of the camera config (egress; CAM v2a). Fire-and-forget like
 *  CMD, unlike ACK it never gates: a lost config is ground-diagnosed by the
 *  CAM_HK-vs-CamState divergence pair, not by a barrier the physics waits on. */
#define BSK_LINK_CAM_TOPIC "CAM"

/**
 * Sensor frame to decode off the link.
 *
 * These are EDS identities from the mission's own XML, compiled into
 * orrery_eds_db by edslink/ and linked into this app: the simulator and the
 * flight software are literaly reading the same compiled schema, so a changed
 * field order is a link/build failure rather than silently wrong floats.
 * \{ */
#define BSK_LINK_SENSOR_TYPE EdsContainer_BSK_LINK_SensorFrame_DATADICTIONARY
#define BSK_LINK_SENSOR_TYPE_NAME "BSK_LINK/SensorFrame"
#define BSK_LINK_FIELD_SIMTIME "SimTimeS"
#define BSK_LINK_FIT_TYPE_NAME "BSK_LINK/FitManifest"
#define BSK_LINK_FIELD_DECLARED_BITS "DeclaredBits"
#define BSK_LINK_FIELD_REGISTRY_HASH "RegistryHash"
#define BSK_LINK_FIELD_OMEGA "OmegaBnB"
#define BSK_LINK_FIELD_FRAME "Frame"
#define BSK_LINK_OMEGA_AXES 3
/** \} */

/**
 * Onboard state (P4b) and GNSS fix (KN) used to declare their field walks
 * here; the REG refactor moved every observability identity into the
 * generated registry table (orrery_frames.h). The payload structs they
 * decode INTO remain the bus contract in orrery_bus_msg.h - fcs and obc_hk
 * still cast the bus image to them - but the generic consumer produces
 * that image from raw container bytes, so no field name lives in C per
 * observability part anymore.
 */

/** Command pipe depth. Lockstep puts TWO egress messages per sensor frame on
 *  this pipe (torque when enabled, always the ack), so the depth counts in
 *  pairs, not frames. */
#define BSK_LINK_PIPE_DEPTH 8

/** SB receive timeout in ms (bounds ZMQ drain latency) */
#define BSK_LINK_SB_TIMEOUT 500

/** Emit one EVS event per N decoded sensor frames (plus the first) */
#define BSK_LINK_SENSOR_EVENT_EVERY 100

/** Emit one EVS event per N forwarded torque frames (plus the first) */
#define BSK_LINK_TORQUE_EVENT_EVERY 100

/** queued-while-disconnected cap for the egress PUSH socket (bytes bounded
 *  by frame size; when full, sends fail EAGAIN and are counted as drops --
 *  a dead sim must never grow cFS memory without limit) */
#define BSK_LINK_EGRESS_HWM 32

/** Max link frames consumed per main-loop pass (keeps the SB pipe responsive) */
#define BSK_LINK_DRAIN_BUDGET 16

/** Ticks to wait before judging claimed-but-silent rows (OB). The world
 *  publishes every declared part every tick, so five is generous; judging
 *  earlier would call a device missing before its first frame could land,
 *  and a verdict that flickers is a verdict nobody reads. */
#define BSK_LINK_FIT_GRACE_TICKS 5

/** \} */

/** \} */

/* -------------------------------------------------------------------------- */

/**
 * Generic world-frame message (REG): header + raw container bytes.
 *
 * One slot per registry row, sized for the largest observability payload.
 * The image is byte-identical to the typed messages this app emitted per
 * frame before the registry refactor (16 B telemetry header + plain LE
 * container), which is why fcs/obc_hk/ground defs need nothing new.
 */
typedef struct
{
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint8                     Payload[BSK_LINK_FRAME_MAX_PAYLOAD];
} BSK_Link_WorldFrameTlm_t;

/**
 * Fit verdict (OB): one of three agreements, or the one disagreement that
 * stops the mission.
 *
 * DEGRADED and DENIED are reported and flown on: a channel that is off is a
 * channel that is off, and the FSW never invents the missing data (KN2's
 * absence rule). STALE is different in kind - the bytes may decode into the
 * wrong fields entirely - and no amount of flying on knowledge that may be
 * mislabelled is worth the ambiguity, so the link refuses the barrier.
 */
typedef enum
{
    BSK_LINK_FIT_UNKNOWN = 0, /* no manifest heard (a world without the frame) */
    BSK_LINK_FIT_MATCHED,     /* declared == built == streaming                */
    BSK_LINK_FIT_SILENT,      /* the manifest claims rows the bus never sent   */
    BSK_LINK_FIT_DENIED,      /* the bus sends rows the manifest denies        */
    BSK_LINK_FIT_STALE        /* registry digest disagrees: refuse to fly      */
} BSK_Link_FitVerdict_t;

typedef struct
{
    CFE_SB_PipeId_t CmdPipe;

    uint32 SensorMsgCount;     /* sensor frames decoded from the feed            */
    uint32 MalformedMsgCount;  /* frames that failed EDS decode                  */
    uint32 TopicSkipCount;     /* frames on a topic other than SENSOR            */
    uint32 MsgRecvCount;       /* multipart messages taken off the socket        */
    uint32 ZmqConnectAttempts;
    uint32 SensorTlmPublishFail; /* SB publishes rejected (reported once)        */
    uint32 TorqueRecvCount;    /* torque commands taken off the bus              */
    uint32 TorqueFwdCount;     /* torque frames written to the sim               */
    uint32 TorqueDropCount;    /* egress queue full / send failed                */
    uint32 AckRecvCount;       /* step acks taken off the bus                    */
    uint32 AckFwdCount;        /* step acks written to the sim (opens its gate)  */
    uint32 AckDropCount;       /* ack egress failed: the sim will hang and abort */
    uint32 CamRecvCount;       /* cam configs taken off the bus (CAM v2a)        */
    uint32 CamFwdCount;        /* cam configs written to the sim                 */
    uint32 CamDropCount;       /* wrong length or egress send failed             */
    uint32 TimeSyncCount;      /* SET_MET sent: mission clock slaved to the sim  */
    uint32 WorldFrameCounts[BSK_LINK_FRAME_ROWS];      /* per registry row      */
    uint32 WorldFramePublishFail[BSK_LINK_FRAME_ROWS]; /* rejected, reported once */

    /* Boot handshake (OB): three views of the same question - what the
     * scenario's TOML CLAIMS, what this binary was BUILT with
     * (BSK_LINK_FIT_REGISTRY_HASH and the generated table), and what the bus
     * actually STREAMS. The declared pair arrives from the world; the built
     * pair is compiled in; the streaming mask is derived from the row counters
     * above. Nothing here is telemetry: the report is the event bus. */
    uint32 FitFrameCount;    /* manifests heard since boot                     */
    uint32 FitDeclaredBits;  /* last declared fit mask                         */
    uint32 FitStreamingBits; /* last derived streaming mask                    */
    uint32 FitSilentBits;    /* claimed, never seen                            */
    uint32 FitDeniedBits;    /* seen, denied by the manifest                   */
    uint32 FitDeclaredHash;  /* registry digest the world started with         */
    uint8  FitVerdict;       /* BSK_Link_FitVerdict_t, for transitions only    */
    bool   FitAbort;         /* stale firmware: refuse to fly (see FitFrame)   */
    uint32 FitRefusedTicks;  /* barrier ticks refused because of that abort    */

    /* Last decoded frame, for events, the sensor-frame MsgId on the bus, and
     * (P4) housekeeping. */
    double LastSimTimeS;
    double OmegaBnB[BSK_LINK_OMEGA_AXES];
    uint64 LastFrame;

    /* Telemetry this app puts on the bus for fcs and the beacon. Header is
     * initialized once (like the stock apps) so sequence counts flow. The
     * observability frames are one generic slot per registry row: the payload
     * is the raw EDS container image, byte-identical to the typed messages
     * this app emitted before the REG refactor (plain LE payload, no padding),
     * so every bus consumer keeps casting the same struct. */
    BskSensorFrameTlm_t SensorFrameTlm;
    BSK_Link_WorldFrameTlm_t WorldFrameTlm[BSK_LINK_FRAME_ROWS];
} BSK_LinkAppData_t;

extern BSK_LinkAppData_t BSK_LinkAppData;

/**
 * \brief Application entry point (startup table: BSK_LINK_AppMain).
 */
void BSK_LINK_AppMain(void);

#endif /* BSK_LINK_APP_H */
