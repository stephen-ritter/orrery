/* SC message IDs, as this mission seats them (2026-11-28 sequence arc).
 *
 * The command and telemetry seats are the mission map verbatim
 * (eds/cfe-topicids.xml: CMD TC+169, SEND_HK TC+170, HK_TLM TM+170 - the
 * upstream defaults, untouched). What moves is the WAKEUP, and it moves by
 * law, not by numbering: upstream sc paced itself on a stock 1 Hz sch tick,
 * a wall-clock heartbeat with no idea what the physics world is doing. This
 * mission's heartbeat is the bsk_link sensor frame - one frame, one sim
 * second, every consumer woken on the same edge with the mission clock just
 * slaved to the frame's timecode (bsk_link.c, scripts/check_clock.sh). A
 * sequencer that evaluates whole-second ATS tags must decide on that edge,
 * not on a timer beside it, so SC_WAKEUP_MID IS the sensor frame's MsgId:
 * every frame arrival is SC's wakeup, and the wake class the clock bench
 * measured is exactly the class the fire decision inherits.
 *
 * This header is the override seam the build's generated sc_msgid_values.h
 * resolves to (same mechanism that seats CF's channels, cf_topicid_values.h)
 * - the pristine app stays pristine: the routing is here, not in apps/sc.
 * The wake dispatch is by token on purpose: the XML still seats a nominal
 * wakeup command at TC+171 (an inert seat - the ground never wakes sc, the
 * link does), and if a future sc ever asks for a fourth command MID the
 * concatenation below names nothing and the build refuses loudly.
 */
#ifndef ORRERY_SC_MSGID_VALUES_H
#define ORRERY_SC_MSGID_VALUES_H

#include "cfe_core_api_base_msgids.h"
#include "sc_topicids.h"
#include "orrery_msgids.h"

/* CFE_PLATFORM_SC_CMD_MIDVAL / _TLM_MIDVAL are what sc_msgids.h calls to
 * mint SC_CMD_MID, SC_SEND_HK_MID, SC_WAKEUP_MID and SC_HK_TLM_MID. The
 * telemetry side keeps the generic base+topicid rule; the command side
 * dispatches on the token so WAKEUP can leave the command range entirely. */
#define CFE_PLATFORM_SC_TLM_MIDVAL(x) CFE_PLATFORM_TLM_TOPICID_TO_MIDV(CFE_MISSION_SC_##x##_TOPICID)

#define CFE_PLATFORM_SC_CMD_MIDVAL(x) SC_ORRERY_CMD_MID_##x

#define SC_ORRERY_CMD_MID_CMD     CFE_PLATFORM_CMD_TOPICID_TO_MIDV(CFE_MISSION_SC_CMD_TOPICID)
#define SC_ORRERY_CMD_MID_SEND_HK CFE_PLATFORM_CMD_TOPICID_TO_MIDV(CFE_MISSION_SC_SEND_HK_TOPICID)

/* The wake law itself: sc's wakeup pipe subscribes to the sensor frame. */
#define SC_ORRERY_CMD_MID_WAKEUP BSK_LINK_SENSOR_FRAME_TLM_MID

#endif /* ORRERY_SC_MSGID_VALUES_H */
