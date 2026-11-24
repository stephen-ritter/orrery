/*
 * cam event IDs. Numbers are stable: COSMOS decodes them through the
 * generated EVS defs, so renaming a comment is free while renumbering is not.
 */
#ifndef CAM_EVENTS_H
#define CAM_EVENTS_H

#define CAM_INIT_INF_EID        1 /* app initialized                       */
#define CAM_NOOP_INF_EID        2 /* NOOP acknowledged                     */
#define CAM_CMD_ERR_EID         3 /* bad function code / bad length        */
#define CAM_SET_CONFIG_INF_EID  4 /* config accepted and forwarded         */
#define CAM_SET_CONFIG_ERR_EID  5 /* config refused (bad value or length)  */
#define CAM_CONFIG_ERR_EID      6 /* config publish rejected (reported 1x) */
#define CAM_PIPE_ERR_EID        7 /* pipe/subscribe failure at init        */

#endif /* CAM_EVENTS_H */
