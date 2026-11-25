/*
 * orrery custom-app topic IDs (mission-defined block).
 *
 * The authoritative numbers are the <Define> entries in
 * orrery_defs/eds/cfe-topicids.xml - those drive the COSMOS defs and the
 * EdsLib database. This header is the flight-C mirror of the same numbers,
 * following the mission's existing global_sbn_topicids.h pattern. They cannot
 * drift silently: tests/test_eds2cosmos.py parses the XML and this header and
 * fails the build on any mismatch.
 *
 * Registered PART topics are no longer hand-written lines here: the
 * generated mirror below renders them from registry/parts.toml, so landing a
 * part never edits this file. App topics stay - apps are not parts.
 */
#ifndef ORRERY_TOPICIDS_H
#define ORRERY_TOPICIDS_H

#include "orrery_part_topicids.h"

/* Telemetry topics (CFE_MISSION/TELEMETRY_BASE_TOPICID-relative values). */
#define CFE_MISSION_BSK_LINK_SENSOR_FRAME_TLM_TOPICID 200
#define CFE_MISSION_BSK_LINK_TORQUE_COMMAND_TLM_TOPICID 201
#define CFE_MISSION_FCS_HK_TLM_TOPICID 202
#define CFE_MISSION_BSK_LINK_STEP_ACK_TLM_TOPICID 203
#define CFE_MISSION_OBC_HK_TLM_TOPICID 205

/* Command topics (CFE_MISSION/TELECOMMAND_BASE_TOPICID-relative values). */
#define CFE_MISSION_FCS_CMD_TOPICID 200
#define CFE_MISSION_FCS_SEND_HK_TOPICID 201
#define CFE_MISSION_FCS_SET_GAIN_TOPICID 202
#define CFE_MISSION_FCS_SET_WHEEL_LIMIT_TOPICID 203
#define CFE_MISSION_OBC_HK_CMD_TOPICID 204
#define CFE_MISSION_OBC_HK_SEND_HK_TOPICID 205

/* Time-tagged command execution (sps app): the queue is fed on
 * SCHEDULE_CMD and evaluated on the sensor-frame strobe, so execution is
 * frame-quantized like a real major-frame schedule. */
#define CFE_MISSION_SPS_HK_TLM_TOPICID 206
#define CFE_MISSION_SPS_CMD_TOPICID 206
#define CFE_MISSION_SPS_SCHEDULE_CMD_TOPICID 207
#define CFE_MISSION_SPS_SEND_HK_TOPICID 208

/* Camera payload (cam app). SET_CONFIG rides its own topic on the
 * SetGainPkt law (payload-bearing commands lose arguments in the FC-subtype
 * pattern); the CAMSTATE telemetry topic is the registry row, not a line
 * here - apps are hand-written, parts are generated. */
#define CFE_MISSION_CAM_HK_TLM_TOPICID 210
#define CFE_MISSION_BSK_LINK_CAM_CONFIG_TLM_TOPICID 212
#define CFE_MISSION_BSK_LINK_CAM_STORAGE_TLM_TOPICID 213
#define CFE_MISSION_CAM_CMD_TOPICID 210
#define CFE_MISSION_CAM_SEND_HK_TOPICID 211
#define CFE_MISSION_CAM_SET_CONFIG_TOPICID 212

/* File downlink request: own topic per the SetGainPkt law - the
 * argument is the file. FCS relays it to CF's own TX_FILE command; the CFDP
 * transport is CF's, never the mission's. */
#define CFE_MISSION_FCS_CFDP_DOWN_TOPICID 214

/* cFS TIME service command port, block-relative (cfe-topicids.xml:
 * TIME_CMD = TELECOMMAND_BASE_TOPICID + 5). The link slaves mission MET to
 * the sim timecode over this port. */
#define ORRERY_TIME_CMD_TOPICID 5

#endif /* ORRERY_TOPICIDS_H */
