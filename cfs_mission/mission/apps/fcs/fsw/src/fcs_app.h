/*
 * fcs: orrery flight control system (P3c).
 *
 * One control law, honestly small: direct rate damping. Every sensor frame
 * bsk_link ingests off the Basilisk link lands on the bus as
 * BSK_LINK_SENSOR_FRAME_TLM; fcs multiplies the measured body rate by -Kp and
 * publishes BSK_LINK_TORQUE_COMMAND_TLM, which bsk_link forwards to the sim's
 * effector. The satellite only ever acts on what it senses: the truth stream
 * exists on other ports and this app never touches the link, only the bus.
 *
 * Ground owns authority over the loop (commands arrive on the :5051 uplink):
 *   FCS_CMD             FC 0 NOOP, FC 1 ENABLE, FC 2 DISABLE
 *   FCS_SET_GAIN        upload the per-axis gain (payload command, see fcs.xml)
 *   FCS_SET_WHEEL_LIMIT upload the supervisor threshold (P4b-2)
 *   FCS_SEND_HK         housekeeping on the scheduler tick
 * The loop starts DISABLED: sensing a tumble must not silently start burning
 * momentum without the ground telling this spacecraft to fly.
 *
 * Mode supervisor (P4b-2, knowledge-driven): fcs ingests its own actuator and
 * instrument state (BSK_LINK_ONBOARD_STATE_TLM, the P4b-1 frame) and trips
 * ENABLED -> SAFE when a |wheel speed| reaches the threshold. SAFE stops the
 * law - not the acks: the lockstep gate never leans on the control path - and
 * latches: ENABLE while SAFE is refused while the knowledge still shows the
 * wheels at the limit, so recovery is a deliberate ground decision (raise the
 * limit, or manage momentum first), not a mode-machine coincidence. A plant
 * with no onboard-state feed (the hand-written witness rig) leaves the
 * supervisor unarmed: no knowledge, no trip, no behavior change.
 *
 * Lockstep contract (D5 amendment): fcs answers EVERY processed sensor frame
 * with BSK_LINK_STEP_ACK_TLM - enabled or disabled, torqued or not - because
 * the ack means "the satellite has seen frame N", which is the sim's gate for
 * taking step N+1. The ack is the sim's clock dependency on flight software,
 * so its publish failure is counted loudly: the sim will hang at the gate and
 * abort, and the counter says why.
 */

#ifndef FCS_APP_H
#define FCS_APP_H

#include "cfe.h"
#include "orrery_bus_msg.h"
#include "orrery_msgids.h"

/** Command/data pipe depth (1 Hz sensor frames + a handful of commands). */
#define FCS_PIPE_DEPTH 8

/** SB receive timeout in ms. */
#define FCS_SB_TIMEOUT 500

/** Number of control axes. */
#define FCS_AXES 3

/** Gain sanity bound for SET_GAIN uploads (N m per rad/s). */
#define FCS_KP_MAX 2000.0

/**
 * Wheel-supervisor defaults (P4b-2), rad/s. The orrery wheel spec is
 * 628 rad/s (sim/scenarios/chief.toml); 95% of spec is "the machine is
 * telling you it is running out of headroom", and the trip must fire before
 * the plant's own clamp does. The upper bound only rejects nonsense uploads:
 * no reaction wheel on this bird spins near it.
 */
#define FCS_WHEEL_LIMIT_DEFAULT 596.6
#define FCS_WHEEL_LIMIT_MAX     10000.0

/** Knowledge staleness bound, in sensor frames. The rig publishes ONBOARD
 * right after SENSOR, so a live link is at most 1 frame behind; anything
 * older is not evidence about THIS actuator state and must not trip the law. */
#define FCS_ONBOARD_MAX_AGE 5

typedef enum
{
    FCS_MODE_DISABLED = 0,
    FCS_MODE_ENABLED  = 1,
    FCS_MODE_SAFE     = 2 /* supervisor latch: law stopped, ground recovery */
} FCS_Mode_t;

typedef struct
{
    CFE_SB_PipeId_t Pipe;

    FCS_Mode_t Mode;
    double     Kp[FCS_AXES];

    uint32 CommandCounter;
    uint32 CommandErrorCounter;
    uint32 SensorFramesReceived;    uint32 SensorFramesBadLen;   /* counted, never fatal: link audit covers it */
    uint32 TorqueCommandsIssued;
    uint32 StepAcksIssued;       /* lockstep barrier acks (every frame, D5)   */
    uint32 StepAckFailCount;     /* an ack that never left = the sim will hang */
    uint32 StepAckResends;       /* idempotent re-acks of an already-processed
                                  * frame: the sim's barrier retransmits because
                                  * ZMQ ate the first PUB of a fresh connection */
    uint64 LastFrameNumber;      /* highest frame processed this boot; a LOWER
                                  * frame means the sim restarted (counters
                                  * rewind) and the watermark resets          */
    uint32 PublishFailCount;

    double LastSimTimeS;
    double LastTorqueB[FCS_AXES];

    /* Supervisor state (P4b-2): the onboard knowledge snapshot plus the
     * threshold it is judged against. OnboardFrame is the sensor-frame
     * watermark of the newest snapshot: age in frames, not wall clock. */
    double WheelLimit;
    uint32 OnboardFramesReceived;
    uint32 OnboardBadLen;
    uint64 OnboardFrame;
    double OnboardWheelSpeeds[FCS_AXES];
    double OnboardSunCosine;
    uint32 ModeTrips; /* ENABLED -> SAFE transitions this boot */

    FcsHkTlm_t HkTlm;
} FCS_AppData_t;

extern FCS_AppData_t FCS_AppData;

void FCS_AppMain(void);

#endif /* FCS_APP_H */
