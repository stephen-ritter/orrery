/*
 * fcs: orrery flight control system.
 *
 * One control law, honestly small: direct rate damping. Every sensor frame
 * bsk_link ingests off the Basilisk link lands on the bus as
 * BSK_LINK_SENSOR_FRAME_TLM; fcs multiplies the measured body rate by -Kp and
 * publishes BSK_LINK_TORQUE_COMMAND_TLM, which bsk_link forwards to the sim's
 * effector. The satellite only ever acts on what it senses: the truth stream
 * exists on other ports and this app never touches the link, only the bus.
 *
 * Ground owns authority over the loop (commands arrive on the :5051 uplink):
 *   FCS_CMD             FC 0 NOOP, FC 1 ENABLE, FC 2 DISABLE, FC 3 BURNSTOP
 *   FCS_SET_GAIN        upload the per-axis gain (payload command, see fcs.xml)
 *   FCS_SET_WHEEL_LIMIT upload the supervisor threshold
 *   FCS_BURN            open a burn quota (payload command, see fcs.xml)
 *   FCS_SEND_HK         housekeeping on the scheduler tick
 * The loop starts DISABLED: sensing a tumble must not silently start burning
 * momentum without the ground telling this spacecraft to fly.
 *
 * Mode supervisor (knowledge-driven): fcs ingests its own actuator and
 * instrument state (BSK_LINK_ONBOARD_STATE_TLM) and trips
 * ENABLED -> SAFE when a |wheel speed| reaches the threshold. SAFE stops the
 * law - not the acks: the lockstep gate never leans on the control path - and
 * latches: ENABLE while SAFE is refused while the knowledge still shows the
 * wheels at the limit, so recovery is a deliberate ground decision (raise the
 * limit, or manage momentum first), not a mode-machine coincidence. A plant
 * with no onboard-state feed (the hand-written witness rig) leaves the
 * supervisor unarmed: no knowledge, no trip, no behavior change.
 *
 * Burn (Rung 1, quota law): the ground asks for force along a body axis for
 * a duration (FCS_BURN); while a quota is open, EVERY newly processed sensor
 * frame carries one BurnCommand frame out on its own topic - the craft owns
 * the pulse pattern, the world owns the physics. Apply-ONCE is the law:
 * each forwarded frame buys exactly one tick, so a craft that stops talking
 * stops thrusting, and BURNSTOP (or the clock) closes the quota the same
 * tick it lands. One burn at a time: a second FCS_BURN while a quota is
 * open is refused, never queued - overlapping quotas have no honest meaning.
 *
 * Lockstep contract: fcs answers EVERY processed sensor frame
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
 * Wheel-supervisor defaults, rad/s. The orrery wheel spec is
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

/**
 * Burn doors, Rung 1 (FCS_BURN quota). The firmware owns the DOORS, never
 * the plant: it cannot know this spacecraft's cluster rating, so the caps
 * reject nonsense (a NaN, an ocean, a week-long burn) while honest
 * over-demand is answered honestly DOWNSTREAM - the sim's world saturates
 * against the real nozzles and its truth ledger records what was ACHIEVED.
 * The claim ledger here therefore promises force x nominal tick per frame
 * forwarded; it is a claim, and only the bench reads both ledgers.
 */
#define FCS_BURN_MAX_FORCE_N    100.0
#define FCS_BURN_MAX_DURATION_S 900.0

/** Nominal tick promised per forwarded burn frame [s]: one frame buys one
 *  control tick of thrust, and the tick is the plant's control period -
 *  1 s on every orrery world. A world that ticks differently makes the
 *  world's applied ledger authoritative, which is what it is for. */
#define FCS_BURN_TICK_S 1.0

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
    uint32 StepAcksIssued;       /* lockstep barrier acks (every frame)       */
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

    /* Supervisor state: the onboard knowledge snapshot plus the
     * threshold it is judged against. OnboardFrame is the sensor-frame
     * watermark of the newest snapshot: age in frames, not wall clock. */
    double WheelLimit;
    uint32 OnboardFramesReceived;
    uint32 OnboardBadLen;
    uint64 OnboardFrame;
    double OnboardWheelSpeeds[FCS_AXES];
    double OnboardSunCosine;
    uint32 ModeTrips; /* ENABLED -> SAFE transitions this boot */

    /* Burn state (Rung 1): the open quota and the claim ledger. StartSimTimeS
     * is the newest sensor frame at acceptance - sim time is the only clock
     * a burn may keep: wall time would make thrust duration a function of
     * host scheduling. BurnSeq numbers the craft's own burns (the wire's
     * BurnId, and HK's LastBurnId), because the ground ask carries no id. */
    uint8  BurnActive; /* 1 while a quota is open                    */
    double BurnForceN;
    double BurnDurationS;
    double BurnAxisB[FCS_AXES]; /* unit, normalized at acceptance      */
    double BurnStartSimTimeS;
    uint64 BurnSeq;
    uint32 BurnFramesForwarded; /* cumulative this boot                  */
    double BurnImpulseClaimed;  /* force x nominal tick, summed          */

    FcsHkTlm_t HkTlm;
} FCS_AppData_t;

extern FCS_AppData_t FCS_AppData;

void FCS_AppMain(void);

#endif /* FCS_APP_H */
