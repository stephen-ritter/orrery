/*
 * sps: time-tagged command execution - the stored program sequence.
 *
 * The ground uplinks complete command images stamped with a sim execution
 * time (SPS_SCHEDULE, own MsgId); sps validates each against the generated
 * EDS command dictionary (sps_cmd_table.h) AT UPLOAD TIME, queues it, and
 * re-emits the stored command on the bus when its time comes. Evaluation
 * rides the frame strobe, never a wall timer: every sensor frame
 * bsk_link ingests arrives with the sim instant it describes, so a schedule
 * compares against the same timecode the physics advanced by - a frozen sim
 * freezes MET, and pending entries wait with it. "Execute at T" means
 * "fire on the first frame whose time is >= T": execution is frame-
 * quantized exactly like a real major-frame schedule.
 *
 * The authority pattern mirrors fcs: the queue starts DISABLED, so a
 * schedule can sit aboard without the power to move anything until an
 * operator ENABLEs it (a command that fires while nobody is watching is
 * either automation or an accident, and the difference is who armed it).
 *
 *   SPS_CMD          FC 0 NOOP, FC 1 ENABLE, FC 2 DISABLE, FC 3 FLUSH
 *   SPS_SCHEDULE     time-tagged upload (payload, own MsgId - see sps.xml)
 *   SPS_SEND_HK      housekeeping on the scheduler tick (and on request)
 *
 * Rejections are the acceptance criteria, not error paths: MsgId/Fc outside
 * the dictionary, payload length mismatch, queue overflow and self-schedules
 * are refused at the uplink with EVS events and HK counters, because the
 * failure mode being designed against is "the ground thought it scheduled
 * something".
 */
#ifndef SPS_APP_H
#define SPS_APP_H

#include "cfe.h"
#include "orrery_bus_msg.h"
#include "orrery_msgids.h"
#include "sps_cmd_table.h"

#define SPS_PIPE_DEPTH 16          /* uploads arrive in bursts; frames at 1 Hz */
#define SPS_SB_TIMEOUT 1000        /* ms: the normal case between frames       */
#define SPS_SLOTS      8           /* queue capacity (operator-visible in HK)  */
#define SPS_DATA_BYTES 48          /* stored payload room = 6 doubles          */

typedef struct
{
    double ExecTimeS;
    uint16 MsgId;
    uint8  Fc;
    uint8  Len;                     /* stored payload bytes                   */
    uint8  Data[SPS_DATA_BYTES];
} Sps_Entry_t;

/* Re-emission buffer: a command header plus the stored payload verbatim.
 * The wire size at fire time is sizeof(Hdr) + Len, the same total the EDS
 * dictionary pinned at schedule time. */
typedef struct
{
    CFE_MSG_CommandHeader_t Hdr;
    uint8                   Data[SPS_DATA_BYTES];
} __attribute__((packed)) Sps_FireMsg_t;

typedef struct
{
    CFE_SB_PipeId_t Pipe;
    SpsHkTlm_t      HkTlm;
    Sps_Entry_t     Queue[SPS_SLOTS]; /* sorted by ExecTimeS; ties keep order  */
    uint32          QueueCount;

    bool     Enabled;               /* authority: queue evaluates only when 1 */
    uint32   ScheduledTotal;
    uint32   FiredTotal;
    uint32   RejectedUnknownTotal;  /* MsgId/Fc absent from the dictionary    */
    uint32   RejectedFullTotal;
    uint32   RejectedParamTotal;    /* wrong length, or self-schedule         */
    uint32   LateFiredTotal;        /* T already passed when uploaded         */
    uint32   FramesEvaluated;
    uint32   QueuePeak;
    double   LastExecTimeS;
    double   LastFiredSimTimeS;
    double   LastFrameSimTimeS;
    uint16   LastFiredMsgId;
    uint32   SensorFramesBadLen;    /* schema drift guard, reported in EVS    */
} SPS_AppData_t;

extern SPS_AppData_t SPS_AppData;

#endif /* sps_APP_H */
