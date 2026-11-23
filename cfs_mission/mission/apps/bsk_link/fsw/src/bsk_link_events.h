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
#define BSK_LINK_TIME_INF_EID        13 /* MET slaved to link timecode (D8)  */
#define BSK_LINK_ONBOARD_INF_EID     14 /* basilisk onboard state (throttled)*/
#define BSK_LINK_NAV_INF_EID         15 /* basilisk GNSS fix (throttled, KN) */
#define BSK_LINK_ATTSTATE_INF_EID    16 /* basilisk ST attitude (throttled, REG) */
#define BSK_LINK_FIT_INF_EID           17 /* fit verdict (OB, on change only)  */
#define BSK_LINK_FIT_ERR_EID           18 /* stale firmware vs manifest: aborts */
#define BSK_LINK_FIT_REPORT_EID        19 /* rows claimed-or-denied, named      */

#endif /* BSK_LINK_EVENTS_H */
