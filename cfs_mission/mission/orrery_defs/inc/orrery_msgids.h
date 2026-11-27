/*
 * orrery custom-app Software Bus message IDs.
 *
 * Same construction as the mission's sbn_msgids.h: target MID base (from
 * global_core_api_base_msgid_values.h: cpu1 cmd 0x1800, tlm 0x0800) plus the
 * topic ID mirrored from the mission EDS XML. Ground definitions generated
 * by codegen/eds2cosmos carry the identical numbers, pinned by golden tests.
 * Registered part MsgIds are generated from registry/parts.toml into
 * orrery_part_msgids.h and included below - landing a part edits no C.
 */
#ifndef ORRERY_MSGIDS_H
#define ORRERY_MSGIDS_H

#include "cfe_core_api_base_msgids.h"
#include "orrery_topicids.h"
#include "orrery_part_msgids.h"

/** \brief Ingested Basilisk sensor frame (bsk_link -> fcs, beacon) */
#define BSK_LINK_SENSOR_FRAME_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_SENSOR_FRAME_TLM_TOPICID)

/** \brief FCS torque command (fcs -> bsk_link -> sim, beacon) */
#define BSK_LINK_TORQUE_COMMAND_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_TORQUE_COMMAND_TLM_TOPICID)

/** \brief fcs housekeeping telemetry (fcs -> beacon) */
#define FCS_HK_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_FCS_HK_TLM_TOPICID)

/** \brief lockstep step ack (fcs -> bsk_link -> sim; bus-internal, never beacon) */
#define BSK_LINK_STEP_ACK_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_STEP_ACK_TLM_TOPICID)

/** \brief fcs ground commands: NOOP / ENABLE / DISABLE / BURNSTOP */
#define FCS_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_CMD_TOPICID)

/** \brief fcs housekeeping request (scheduler) */
#define FCS_SEND_HK_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_SEND_HK_TOPICID)

/** \brief fcs gain upload with payload (ground only) */
#define FCS_SET_GAIN_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_SET_GAIN_TOPICID)

/** \brief fcs wheel supervisor threshold with payload (ground only) */
#define FCS_SET_WHEEL_LIMIT_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_SET_WHEEL_LIMIT_TOPICID)

/** \brief file downlink request with payload (ground only) */
#define FCS_CFDP_DOWN_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_CFDP_DOWN_TOPICID)

/** \brief fcs burn command with payload (ground only; own packet per the
 * SetGain law - an argument-bearing command is its own interface) */
#define FCS_BURN_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_FCS_BURN_TOPICID)

/** \brief OBC housekeeping telemetry (hk_app -> beacon) */
#define OBC_HK_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_OBC_HK_TLM_TOPICID)

/** \brief OBC hk ground commands: NOOP */
#define OBC_HK_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_OBC_HK_CMD_TOPICID)

/** \brief OBC housekeeping request (scheduler) */
#define OBC_HK_SEND_HK_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_OBC_HK_SEND_HK_TOPICID)

/** \brief sps ground commands: NOOP / ENABLE / DISABLE / FLUSH */
#define SPS_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_SPS_CMD_TOPICID)

/** \brief time-tagged command upload with payload (ground only) */
#define SPS_SCHEDULE_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_SPS_SCHEDULE_CMD_TOPICID)

/** \brief sps housekeeping request (scheduler) */
#define SPS_SEND_HK_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_SPS_SEND_HK_TOPICID)

/** \brief sps housekeeping telemetry (sps -> beacon) */
#define SPS_HK_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_SPS_HK_TLM_TOPICID)

/* Camera payload (cam app): the command surface. SET_CONFIG is a
 * ground-only upload (its payload never reaches the sim by this MID; the app
 * re-emits the accepted command as BSK_LINK_CAM_CONFIG_TLM, the same relay
 * shape as fcs torque). */

/** \brief cam ground commands (COSMOS -> cam) */
#define CAM_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_CAM_CMD_TOPICID)

/** \brief cam housekeeping request (scheduler -> cam) */
#define CAM_SEND_HK_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_CAM_SEND_HK_TOPICID)

/** \brief camera config upload (COSMOS -> cam, own packet per SetGain law) */
#define CAM_SET_CONFIG_MID (SAMPLE_CPU1_CMD_MID_BASE + CFE_MISSION_CAM_SET_CONFIG_TOPICID)

/** \brief cam housekeeping telemetry (cam -> beacon) */
#define CAM_HK_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_CAM_HK_TLM_TOPICID)

/** \brief accepted camera config (cam -> bsk_link -> sim, never gates) */
#define BSK_LINK_CAM_CONFIG_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_CAM_CONFIG_TLM_TOPICID)
#define BSK_LINK_CAM_STORAGE_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_CAM_STORAGE_TLM_TOPICID)

/** \brief one tick of an accepted burn (fcs -> bsk_link -> sim, never gates;
 * apply-once, so a silent craft stops thrusting - see bsk_link.xml) */
#define BSK_LINK_BURN_COMMAND_TLM_MID (SAMPLE_CPU1_TLM_MID_BASE + CFE_MISSION_BSK_LINK_BURN_COMMAND_TLM_TOPICID)

/** \brief cFS TIME service commands (link uses SET_MET to slave MET to the
 * sim timecode) */
#define ORRERY_TIME_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + ORRERY_TIME_CMD_TOPICID)

#endif /* ORRERY_MSGIDS_H */
