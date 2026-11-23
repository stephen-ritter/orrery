/*
 * gs_tcp event identifiers.
 */

#ifndef GS_TCP_EVENTS_H
#define GS_TCP_EVENTS_H

#define GS_TCP_INIT_INF_EID        1 /* app initialized, port listening      */
#define GS_TCP_INIT_ERR_EID        2 /* init failure (socket/bind/pipe)      */
#define GS_TCP_SUBSCRIBE_ERR_EID   3 /* a table MsgId failed to subscribe    */
#define GS_TCP_CLIENT_CONNECT_INF  4 /* ground client accepted               */
#define GS_TCP_CLIENT_DISCONNECT   5 /* ground client gone                  */
#define GS_TCP_CMD_INJECT_ERR_EID  6 /* received frame failed SB inject      */
#define GS_TCP_NO_CLIENT_INF_EID   7 /* first telemetry byte with no peer    */
#define GS_TCP_STRAY_RX_ERR_EID    8 /* command bytes seen on the beacon port*/

#endif /* GS_TCP_EVENTS_H */
