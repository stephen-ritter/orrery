/* cf_tcp event ids: the link's testimony, one line per kind of news. */
#ifndef CF_TCP_EVENTS_H
#define CF_TCP_EVENTS_H

#define CF_TCP_INIT_INF_EID          1 /* app initialized, port named            */
#define CF_TCP_INIT_ERR_EID          2 /* pipe/subscribe failure at init         */
#define CF_TCP_SOCKET_ERR_EID        9 /* socket/listen failure                  */
#define CF_TCP_CLIENT_INF_EID        3 /* ground CFDP engine connected           */
#define CF_TCP_CLIENT_LOST_INF_EID   4 /* ground peer went away                  */
#define CF_TCP_TX_ERR_EID            5 /* PDU write refused after connect        */
#define CF_TCP_FRAME_ERR_EID         6 /* inbound segment refused (framing law)  */
#define CF_TCP_REFUSE_INF_EID        7 /* second dialer refused: one peer per pipe */
#define CF_TCP_PUBLISH_ERR_EID       8 /* inbound PDU rejected by the bus        */

#endif /* CF_TCP_EVENTS_H */
