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
 *  The sensor frame is the CONTROL path, the barrier's witness, so it stays
 *  hand-wired here; every OBSERVABILITY frame (ONBOARD, NAVSTATE, ...) is a
 *  registry row in orrery_frames.h and needs no define in this file. */
#define BSK_LINK_SENSOR_TOPIC "SENSOR"
/** Boot handshake: the world's declared part fit, not telemetry.
 *  Its container has no TopicId on purpose - no MsgId, no ground definition,
 *  no beacon line. A handshake that reached the ground would tempt someone to
 *  grade the ground's opinion of the fit instead of the spacecraft's. */
#define BSK_LINK_FIT_TOPIC "FIT"

/** ZMQ topic of the commanded torque (egress; the sim decodes it with the same DB) */
#define BSK_LINK_CMD_TOPIC "CMD"

/** ZMQ topic of the lockstep barrier ack (egress). fcs answers
 *  every processed frame; the sim holds its next Basilisk step for this. */
#define BSK_LINK_ACK_TOPIC "ACK"

/** ZMQ topic of the camera config (egress). Fire-and-forget like
 *  CMD, unlike ACK it never gates: a lost config is ground-diagnosed by the
 *  CAM_HK-vs-CamState divergence pair, not by a barrier the physics waits on. */
#define BSK_LINK_CAM_TOPIC "CAM"

/** ZMQ topic of the per-tick burn frame (egress). One frame per sensor frame
 *  while fcs holds an open quota, and apply-ONCE downstream: each frame buys
 *  exactly one tick of thrust, so a craft that stops egressing stops pushing.
 *  Fire-and-forget like CAM — a dropped frame costs one tick of honest
 *  under-thrust, visible as claim-vs-applied divergence at the bench. */
#define BSK_LINK_BURN_TOPIC "BURN"

/* Store outlet: image bytes ride the sensor feed as their own topic
 * (fire-and-forget like every other sim frame), sliced so one frame never
 * has to hold a whole PNG. */
#define BSK_LINK_IMG_TOPIC      "IMG"
#define BSK_LINK_IMG_SLICE_MAX  16384

/* The recorder (the scenario's [[store]] row for this vehicle).
 *
 * The bound is compiled in, so the only flight-time question is what the store
 * holds right now. The watermark is the point where a full recorder is still
 * avoidable from the ground: warn there, refuse only at the bound. */
#define BSK_LINK_STORE_DIR        "/cf/img"
#define BSK_LINK_STORE_CATALOG     BSK_LINK_STORE_DIR "/.catalog"
#define BSK_LINK_STORE_CATALOG_PART BSK_LINK_STORE_DIR "/.catalog.part"

/** The ground's inventory, published by the store itself beside its catalog
 *  (boot census + every mutation). It lives OUTSIDE /cf/img - the store
 *  directory holds only requestable frames - and the ground requests
 *  /cf/manifest.txt through FCS_CFDP_DOWN by this exact name; one name, one
 *  law, both sides. Draft-and-swap like the catalog: a reader mid-fetch
 *  holds the whole old inventory, never a half line. */
#define BSK_LINK_STORE_MANIFEST      "/cf/manifest.txt"
#define BSK_LINK_STORE_MANIFEST_PART "/cf/manifest.part"
                                 /* dot names are store internals: the boot
                                  * walk and the ground's inventory both skip
                                  * them, so the index is never its own file */
#define BSK_LINK_STORE_PATH_MAX   96 /* enough for the longest frame path + headroom */
#define BSK_LINK_STORE_WATERMARK  80 /* percent of either bound that warns ground */
#define BSK_LINK_STORE_NOMINAL    0
#define BSK_LINK_STORE_NEAR_FULL  1
#define BSK_LINK_STORE_FULL       2
#define BSK_LINK_STORE_UNCLAIMED  3 /* this vehicle declares no recorder: the
                                     * store is written, never enforced          */

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
 * Onboard state and GNSS fix used to declare their field walks
 * here; the generated registry table (orrery_frames.h) now carries every
 * observability identity. The payload structs they
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

/** Ticks to wait before judging claimed-but-silent rows. The world
 *  publishes every declared part every tick, so five is generous; judging
 *  earlier would call a device missing before its first frame could land,
 *  and a verdict that flickers is a verdict nobody reads. */
#define BSK_LINK_FIT_GRACE_TICKS 5

/** \} */

/** \} */

/* -------------------------------------------------------------------------- */

/**
 * Generic world-frame message: header + raw container bytes.
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
 * Fit verdict: one of three agreements, or the one disagreement that
 * stops the mission.
 *
 * DEGRADED and DENIED are reported and flown on: a channel that is off is a
 * channel that is off, and the FSW never invents the missing data (the
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
    uint32 CamRecvCount;       /* cam configs taken off the bus                  */
    uint32 CamFwdCount;        /* cam configs written to the sim                 */
    uint32 CamDropCount;       /* wrong length or egress send failed             */
    uint32 BurnRecvCount;      /* burn frames taken off the bus (1 Hz while open)*/
    uint32 BurnFwdCount;       /* burn frames written to the sim (each = 1 tick) */
    uint32 BurnDropCount;      /* wrong length or egress failed: one tick lost   */
    /* Store outlet: reassembly of the sim's IMG chunk stream into
     * /cf/img files. The fd belongs to the ONE open capture; ordering is
     * TCP's, so anything out of sequence is damage to count, not to chase. */
    osal_id_t StoreFd;         /* OSAL id of the open .part, UNDEFINED idle    */
    uint64 StoreCam;           /* capture identity of the open stream            */
    uint64 StoreCapture;
    uint32 StoreNextPart;      /* part index the socket must produce next        */
    uint32 StoreTotal;
    uint64 StoreBytes;         /* bytes written to the open .part                */
    uint32 ImgChunkCount;      /* IMG frames taken off the socket                */
    uint32 ImgDropCount;       /* refused chunks (ordering, size, file errors)   */
    uint32 StorePublishFail;   /* CamStorage TLM rejected (reported once)        */
    uint64 FilesStored;        /* complete files under /cf/img since boot        */
    uint64 BytesStored;        /* payload bytes across those files               */
    /* The recorder's bound, compiled in from the flying scenario's [[store]]
     * rows (codegen/emit_store). Occupancy is deliberately SEPARATE from
     * FilesStored/BytesStored above: those are the boot's history, and a file
     * that leaves the store - recycled or refused - must not rewrite it. */
    uint8  StoreClaimed;       /* 1: this vehicle ships a declared recorder      */
    uint64 StoreCapacityBytes;
    uint32 StoreCapacityFiles;
    uint32 StoreFilesNow;      /* complete files occupying the store right now   */
    uint64 StoreUsedBytes;
    uint64 StoreGeneration;    /* store mutations (landed or recycled): the
                                * manifest's turn to follow the store's          */
    uint64 FilesEvicted;       /* proven-received files recycled to make room    */
    uint64 CapturesLost;       /* captures refused because the store was full    */
    uint64 TransfersAcked;     /* files an acknowledged transfer proved received */
    uint8  StoreState;         /* BSK_LINK_STORE_{NOMINAL,NEAR,FULL}             */
    uint8  RefusedActive;      /* 1 while the stream below is refused: the slices
                                * of a refused capture must not sneak back       */
    uint64 RefusedCam;         /* in through the out-of-order branch             */
    uint64 RefusedCapture;
    BskCamStorageTlm_t CamStorageTlm; /* the ledger as the bus sees it           */
    uint32 TimeSyncCount;      /* SET_MET sent: mission clock slaved to the sim  */
    uint32 WorldFrameCounts[BSK_LINK_FRAME_ROWS];      /* per registry row      */
    uint32 WorldFramePublishFail[BSK_LINK_FRAME_ROWS]; /* rejected, reported once */

    /* Boot handshake: three views of the same question - what the
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
    /* Instance attribution: slot k counts the k-th id-carrying frame row.
     * The world ships the declared ids in the manifest; the streamed twin
     * is this spacecraft's own observation, read from each arriving
     * frame's id field by name. A verdict on ids names the instance that
     * stalled, not just the part. */
    uint32 FitDeclaredIds[BSK_LINK_FIT_ID_SLOTS]; /* ids claimed per slot    */
    uint32 FitStreamedIds[BSK_LINK_FIT_ID_SLOTS]; /* ids seen in frames      */
    uint8  FitIdSlot;        /* slot of the current id-level verdict           */
    uint32 FitIdBits;        /* id mask of the current id-level verdict        */
    bool   FitIdDenied;      /* the id-level verdict denies a stream           */
    uint32 FitDeclaredHash;  /* registry digest the world started with         */
    uint8  FitVerdict;       /* BSK_Link_FitVerdict_t, for transitions only    */
    bool   FitAbort;         /* stale firmware: refuse to fly (see FitFrame)   */
    uint32 FitRefusedTicks;  /* barrier ticks refused because of that abort    */

    /* Last decoded frame, for events, the sensor-frame MsgId on the bus, and
     * housekeeping. */
    double LastSimTimeS;
    double OmegaBnB[BSK_LINK_OMEGA_AXES];
    uint64 LastFrame;

    /* Telemetry this app puts on the bus for fcs and the beacon. Header is
     * initialized once (like the stock apps) so sequence counts flow. The
     * observability frames are one generic slot per registry row: the payload
     * is the raw EDS container image, byte-identical to the typed messages
     * this app emitted before the registry table (plain LE payload, no
     * padding),
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
