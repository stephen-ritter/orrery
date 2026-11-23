/*
 * obc_hk event IDs. Numbers are stable, like every other app's: COSMOS
 * decodes them through the generated EVS defs.
 */
#ifndef OBC_HK_EVENTS_H
#define OBC_HK_EVENTS_H

#define OBC_HK_INIT_INF_EID 1 /* app initialized                        */
#define OBC_HK_NOOP_INF_EID 2 /* NOOP acknowledged                      */
#define OBC_HK_CMD_ERR_EID  3 /* bad function code / bad length         */
#define OBC_HK_PIPE_ERR_EID 4 /* pipe/subscribe failure at init         */

#endif /* OBC_HK_EVENTS_H */
