/*
 * bsk_link event identifiers.
 */

#ifndef BSK_LINK_EVENTS_H
#define BSK_LINK_EVENTS_H

#define BSK_LINK_INIT_INF_EID         1 /* app initialized                   */
#define BSK_LINK_INIT_ERR_EID         2 /* init failure                      */
#define BSK_LINK_ZMQ_ERR_EID          3 /* zmq resource failure              */
#define BSK_LINK_ZMQ_CONNECT_INF_EID  4 /* feed connected                    */
#define BSK_LINK_SENSOR_INF_EID       5 /* basilisk sensor frame (throttled) */
#define BSK_LINK_DECODE_ERR_EID       6 /* EDS decode failure (reported once)*/
#define BSK_LINK_TOPIC_ERR_EID        7 /* link frame on an unknown topic    */
#define BSK_LINK_TORQUE_INF_EID       8 /* torque forwarded to sim (throttled)*/
#define BSK_LINK_SUBSCRIBE_ERR_EID    9 /* SB subscribe rejected at init     */
#define BSK_LINK_PUBLISH_ERR_EID     10 /* SB publish rejected (reported once)*/
#define BSK_LINK_EGRESS_ERR_EID      11 /* torque egress send failed (once)  */
#define BSK_LINK_ACK_ERR_EID         12 /* step ack egress failed (sim hangs)*/
#define BSK_LINK_TIME_INF_EID        13 /* MET slaved to link timecode     */
#define BSK_LINK_ONBOARD_INF_EID     14 /* basilisk onboard state (throttled)*/
#define BSK_LINK_NAV_INF_EID         15 /* basilisk GNSS fix (throttled)      */
#define BSK_LINK_ATTSTATE_INF_EID    16 /* basilisk ST attitude (throttled)      */
#define BSK_LINK_FIT_INF_EID           17 /* fit verdict (on change only)     */
#define BSK_LINK_FIT_ERR_EID           18 /* stale firmware vs manifest: aborts */
#define BSK_LINK_FIT_REPORT_EID        19 /* rows claimed-or-denied, named      */
#define BSK_LINK_OPNAV_INF_EID         20 /* basilisk OpNav knowledge (CAM)     */
#define BSK_LINK_CAM_STATE_INF_EID     21 /* basilisk CamState row               */
#define BSK_LINK_CAM_INF_EID           22 /* cam config forwarded to sim         */
#define BSK_LINK_STORE_INF_EID         23 /* image file completed under /cf/img  */
#define BSK_LINK_STORE_ERR_EID         24 /* store open/write/rename refused     */
#define BSK_LINK_STORE_WATERMARK_INF_EID 25 /* recorder crossed the warn line    */
#define BSK_LINK_STORE_FULL_ERR_EID      26 /* capture refused: store at its bound */
#define BSK_LINK_STORE_EVICT_INF_EID     27 /* proven-received file recycled       */
#define BSK_LINK_STORE_ACKWRN_INF_EID    28 /* ack-pending overflow: proof         */
                                            /* capacity, not transfer capacity     */
#define BSK_LINK_BURN_INF_EID            29 /* burn frame forwarded to the sim     */

#endif /* BSK_LINK_EVENTS_H */
