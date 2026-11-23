/*
 * sps event IDs. Numbers are stable: COSMOS decodes them through the
 * generated EVS defs, so renaming a comment is free while renumbering is not.
 */
#ifndef SPS_EVENTS_H
#define SPS_EVENTS_H

#define SPS_INIT_INF_EID         1 /* app initialized (queue DISABLED)      */
#define SPS_NOOP_INF_EID         2 /* NOOP acknowledged                     */
#define SPS_ENABLE_INF_EID       3 /* queue armed: schedules may fire       */
#define SPS_DISABLE_INF_EID      4 /* queue frozen: nothing fires           */
#define SPS_FLUSH_INF_EID        5 /* queue emptied on request              */
#define SPS_SCHEDULE_INF_EID     6 /* schedule accepted onto the queue      */
#define SPS_FIRE_INF_EID         7 /* stored command executed on a frame    */
#define SPS_REJ_UNKNOWN_ERR_EID  8 /* schedule refused: not in dictionary   */
#define SPS_REJ_FULL_ERR_EID     9 /* schedule refused: queue at capacity   */
#define SPS_REJ_PARAM_ERR_EID   10 /* schedule refused: length/self/nonfinite */
#define SPS_CMD_ERR_EID         11 /* bad function code / length / mid      */
#define SPS_PIPE_ERR_EID        12 /* pipe/subscribe failure at init        */
#define SPS_FIRE_ERR_EID        13 /* firing publish rejected (reported 1x) */
#define SPS_FRAME_ERR_EID       14 /* sensor frame length drift             */

#endif /* SPS_EVENTS_H */
