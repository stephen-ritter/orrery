/*
 * fcs event IDs. Numbers are stable: COSMOS decodes them through the
 * generated EVS defs, so renaming a comment is free while renumbering is not.
 */
#ifndef FCS_EVENTS_H
#define FCS_EVENTS_H

#define FCS_INIT_INF_EID         1 /* app initialized                       */
#define FCS_NOOP_INF_EID         2 /* NOOP acknowledged                     */
#define FCS_ENABLE_INF_EID       3 /* control loop enabled                  */
#define FCS_DISABLE_INF_EID      4 /* control loop disabled                 */
#define FCS_SET_GAIN_INF_EID     5 /* gain uploaded                         */
#define FCS_CMD_ERR_EID          6 /* bad function code / bad length / mid  */
#define FCS_SET_GAIN_ERR_EID     7 /* gain rejected (non-finite, out of range)*/
#define FCS_TORQUE_ERR_EID       8 /* torque publish rejected (reported 1x) */
#define FCS_PIPE_ERR_EID         9 /* pipe/subscribe failure at init        */
#define FCS_STEP_ACK_ERR_EID    10 /* step ack publish rejected (sim hangs)  */
#define FCS_SESSION_INF_EID     11 /* basilisk frame counter rewound: new run */
#define FCS_MODE_SAFE_ERR_EID   12 /* supervisor trip: ENABLED -> SAFE        */
#define FCS_ENABLE_REJ_ERR_EID  13 /* ENABLE refused: SAFE, wheels at limit   */
#define FCS_SET_LIMIT_INF_EID   14 /* wheel trip threshold uploaded           */
#define FCS_SET_LIMIT_ERR_EID   15 /* threshold rejected (non-finite, absurd) */
#define FCS_CFDP_DOWN_INF_EID   16 /* file handed to CF for downlink          */
#define FCS_CFDP_DOWN_ERR_EID   17 /* request malformed or relay rejected     */
#define FCS_BURN_INF_EID        18 /* burn accepted: quota open, per-frame fwd */
#define FCS_BURN_REJ_ERR_EID    19 /* burn refused: door (argument, overlap)   */
#define FCS_BURN_TLM_ERR_EID    20 /* burn frame publish rejected (reported 1x)*/
#define FCS_BURNSTOP_INF_EID    21 /* BURNSTOP accepted: quota closed by ground*/
#define FCS_BURN_DONE_INF_EID   22 /* quota elapsed: burn closed by duration   */

#endif /* FCS_EVENTS_H */
