/* CF topic IDs, as this mission seats them.
 *
 * CF's stock values put the two RX channel slots at TC topicids 200/201 -
 * which are this mission's FCS_CMD and FCS_SEND_HK. With the defaults,
 * every fcs command also arrived inside CF as a malformed CFDP PDU ("PDU
 * too short" in every console log) and every real inbound PDU arrived
 * inside fcs as a malformed command: two apps reading each other's
 * private conversation. The mission map (eds/cfe-topicids.xml) already
 * seats CF's channels at TX TM+180/181 and RX TC+182/183; this file makes
 * the firmware actually use that map. The XML is not consulted by CF at
 * compile time - the EDS plumbing is off - so the override header is the
 * one lever, and these numbers are the firmware side of the same contract
 * the ground defs read from the XML.
 */
#ifndef ORRERY_CF_TOPICID_VALUES_H
#define ORRERY_CF_TOPICID_VALUES_H

/* Telecommand side: entity interface. */
#define CFE_MISSION_CF_TIDVAL_CMD      179
#define CFE_MISSION_CF_TIDVAL_SEND_HK  180
#define CFE_MISSION_CF_TIDVAL_WAKE_UP  181

/* Telemetry side: housekeeping and end-of-transaction. */
#define CFE_MISSION_CF_TIDVAL_HK_TLM  176
#define CFE_MISSION_CF_TIDVAL_EOT_TLM 179

/* The file pipe: PDUs out on a telemetry topic, PDUs in on a command one.
 * Chosen clear of fcs (TC 200/201) and of bsk_link (TM 200/201). */
#define CFE_MISSION_CF_TIDVAL_CH0_TX 180
#define CFE_MISSION_CF_TIDVAL_CH1_TX 181
#define CFE_MISSION_CF_TIDVAL_CH0_RX 182
#define CFE_MISSION_CF_TIDVAL_CH1_RX 183

#define CFE_MISSION_CF_TIDVAL(x) CFE_MISSION_CF_TIDVAL_##x

#endif /* ORRERY_CF_TOPICID_VALUES_H */
