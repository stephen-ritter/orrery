/*
 * obc_hk: orrery OBC health and status.
 *
 * The satellite's report on ITSELF, built entirely from what passes its
 * pipe: link frames it watched on the bus, actuator feedback it decoded
 * off the onboard-state stream, the mode its controller last published.
 * No other app's memory, no request/response dependencies - a silent fcs
 * shows up as a frozen mirror, which is a symptom, not a second bug.
 * Navigator truth lives on other ports and never enters this app: the
 * boundary is physical - hk_app could not read truth if it wanted to.
 *
 * Commands (uplink :5051):
 *   OBC_HK_CMD     FC 0 NOOP
 *   OBC_HK_SEND_HK housekeeping on the scheduler tick (1 Hz)
 */

#ifndef OBC_HK_APP_H
#define OBC_HK_APP_H

#include "cfe.h"
#include "orrery_bus_msg.h"
#include "orrery_msgids.h"

/** Subscription pipe depth (1 Hz frames on several MsgIds + commands). */
#define OBC_HK_PIPE_DEPTH 12

/** SB receive timeout in ms. */
#define OBC_HK_SB_TIMEOUT 500

typedef struct
{
    CFE_SB_PipeId_t Pipe;

    uint8 CommandCounter;
    uint8 CommandErrorCounter;

    /* Bus-observed counts: what THIS app saw, independent of the apps that
     * produced the traffic. Divergence between these and the producers' own
     * counters is a routing/size defect, which is worth counting. */
    uint32 SensorFramesSeen;
    uint32 OnboardFramesSeen;
    uint32 TorquePacketsSeen;
    uint32 BadLenCount; /* packets with a size the schema rejects */

    /* Freshest bus reports, mirrored for the HK image. */
    double LastSensorSimTimeS;
    double LastOnboardSimTimeS;
    double WheelSpeeds[3];
    double SunCosine;
    double MagFieldNormT;
    uint8  FcsMode;
    uint32 FcsTorquesIssued;

    ObcHkTlm_t HkTlm;
} OBC_HK_AppData_t;

extern OBC_HK_AppData_t OBC_HK_AppData;

void OBC_HK_AppMain(void);

#endif /* OBC_HK_APP_H */
