/*
 * gs_tcp application global data and constants.
 */

#ifndef GS_TCP_APP_H
#define GS_TCP_APP_H

#include <stdbool.h>

#include "cfe.h"
#include "rs_mid_table.h"

/**
 * \name Configuration
 * \{
 */

/** Downlink (telemetry beacon) TCP endpoint; app binds and accepts multiple */
#define GS_TCP_TLM_PORT 5050

/** Uplink (command receiver) TCP endpoint; commands are accepted ONLY here */
#define GS_TCP_CMD_PORT 5051

/** Max simultaneously served clients per direction */
#define GS_TCP_MAX_CLIENTS 4

/** SB receive timeout in ms (bounds TCP service latency) */
#define GS_TCP_SB_TIMEOUT 250

/** SB pipe depth: OSAL queues cap at 50, which comfortably covers the
 *  1 Hz-class traffic plus EVS bursts (pipe drains every loop pass). */
#define GS_TCP_PIPE_DEPTH 50

/** Read chunk for one non-blocking recv() per client per cycle */
#define GS_TCP_RX_CHUNK 4096

/** CCSDS primary header length */
#define GS_TCP_PRIMARY_HDR 6

/** \} */

/* -------------------------------------------------------------------------- */

typedef struct
{
    int  Fd;      /* -1 = slot free                                        */
    int  Backlog; /* bytes of a partially received packet (uplink only)    */
    uint8 Partial[CFE_MISSION_SB_MAX_SB_MSG_SIZE];

    uint32 TxPackets; /* CCSDS packets streamed to this client (downlink)  */
    uint32 RxPackets; /* CCSDS packets injected from this client (uplink)  */
    uint32 RxBadCount;
} GS_TCP_Client_t;

typedef struct
{
    CFE_SB_PipeId_t TlmPipe;

    /* Fleet-resolved ports (D9): compiled defaults unless the host env
     * names another block; every print and bind uses THESE, not the
     * defines, so the log always tells the truth about this instance. */
    uint16   TlmPort;     /* from ORRERY_TLM_PORT or default */
    uint16   CmdPort;     /* from ORRERY_CMD_PORT or default */

    int      TlmListenFd; /* GS_TCP_TLM_PORT: writes only   */
    int      CmdListenFd; /* GS_TCP_CMD_PORT: reads only    */
    uint32   TlmClientCount;
    uint32   CmdClientCount;
    uint32   ConnectCount;
    uint32   RxTotalCount;      /* commands injected onto the SB (all peers) */
    uint32   RxBadTotal;
    uint32   StrayDownlinkRxBytes; /* bytes written to the beacon port: ignored */
    uint32   DropNoClientCount;    /* telemetry dropped with no downlink peer  */
    GS_TCP_Client_t TlmClients[GS_TCP_MAX_CLIENTS];
    GS_TCP_Client_t CmdClients[GS_TCP_MAX_CLIENTS];
} GS_TCP_AppData_t;

extern GS_TCP_AppData_t GS_TCP_AppData;

/**
 * \brief Application entry point (startup table: GS_TCP_AppMain).
 */
void GS_TCP_AppMain(void);

#endif /* GS_TCP_APP_H */
