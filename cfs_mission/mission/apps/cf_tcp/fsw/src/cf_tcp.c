/* cf_tcp: the CFDP transport bridge.

   CF 4.0 keeps its sockets at home: every CFDP PDU is one SB message on
   the channel TX MID, and every PDU CF must hear arrives on the channel RX
   MID. This app is the wire those messages imagine: it listens, the ground
   CFDP engine dials in, and each direction moves whole PDUs - never
   halves. A frame that would go out broken takes the link down instead
   (half a PDU would desynchronise the peer's framing forever); a frame
   that arrives broken is refused and counted, and the peer reconnects. A
   peer's leaving is visible - a FIN frees the seat on the next tick, and
   keepalive catches the silent deaths - so the one-peer law never strands
   the bridge on a corpse: refusal means a live conversation is not
   displaced, which is the only thing it was ever for. The
   bridge has no opinions about CFDP: no acks, no retransmit, no file
   knowledge - reliability is CF class 2's declared job, and a pipe doing
   reliability quietly would be the second, lying copy of it.

   Delimitation is the one framing duty CCSDS 727.0-B-5 (2.2.5) hands to
   the bearer, and it prescribes no shape, so this is the mission's own:
   4-byte big-endian length covering the stream id and the PDU (value = 4
   + PDU bytes), a 4-byte stream id (value 1), then the raw CFDP PDU. The
   interop flights pin it: real CFDP engines byte-matched through it. On
   the bus side the PDU starts where CF's own struct says it starts -
   after the full telemetry message header, time secondary included
   (measured law: the bridge once stripped the CCSDS primary alone, and
   the engine drowned in 7-byte timestamps). */

#include "cf_tcp_app.h"
#include "cf_tcp_events.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "cfe.h"
#include "cf_msgids.h"

CF_TCP_AppData_t CF_TCP_AppData;

/* Inbound PDUs are stamped into this scratch and written to the bus, the
 * same idiom gs_tcp uses to inject ground commands: the bridge never
 * allocates on the bus, and the CCSDS header is built by hand so the one
 * MID this app speaks is visibly the one from cf_msgids.h. */
static union
{
    CFE_SB_Buffer_t Buf;
    uint8           Bytes[sizeof(CFE_SB_Buffer_t) + CF_TCP_ID_BYTES + CF_TCP_PDU_MAX];
} CF_TCP_RxScratch;

static uint8 CF_TCP_TxFrame[CF_TCP_LEN_BYTES + CF_TCP_ID_BYTES + CF_TCP_PDU_MAX];

/* CF reads and writes PDUs at different offsets depending on the message
 * type, and the bridge must honor each law exactly (both measured, the hard
 * way): CF's TX messages are telemetry - CCSDS primary PLUS the 7-byte CFE
 * time secondary, which is where offsetof(CF_PduTlmMsg_t, ph) points; PDUs
 * arriving on the command RX MID are parsed at offsetof(CF_PduCmdMsg_t, ph)
 * - primary plus the 2-byte function/checksum secondary, the same shape a
 * ground upload has. One type each: if the mission ever changes a message
 * header, the matching side moves with it. */
#define CF_TCP_TX_HDR ((size_t)sizeof(CFE_MSG_TelemetryHeader_t))
#define CF_TCP_RX_HDR ((size_t)sizeof(CFE_MSG_CommandHeader_t))

/* -------------------------------------------------------------------------- */

static uint16 CF_TCP_ResolvePort(const char *EnvName, uint16 CompiledDefault)
{
    const char *Env   = getenv(EnvName);
    long        Value;

    if (Env == NULL || Env[0] == '\0')
    {
        return CompiledDefault;
    }
    Value = strtol(Env, NULL, 10);
    if (Value <= 0 || Value > 65535)
    {
        return CompiledDefault; /* a garbage env degrades, never misroutes */
    }
    return (uint16)Value;
}

static int CF_TCP_BindListener(uint16 Port)
{
    int                Fd;
    int                One = 1;
    int                Flags;
    struct sockaddr_in Addr;

    Fd = socket(AF_INET, SOCK_STREAM, 0);
    if (Fd < 0)
    {
        CFE_EVS_SendEvent(CF_TCP_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR, "socket failed: %s",
                          strerror(errno));
        return -1;
    }
    setsockopt(Fd, SOL_SOCKET, SO_REUSEADDR, &One, sizeof(One));

    memset(&Addr, 0, sizeof(Addr));
    Addr.sin_family      = AF_INET;
    Addr.sin_addr.s_addr = htonl(INADDR_ANY);
    Addr.sin_port        = htons(Port);
    if (bind(Fd, (struct sockaddr *)&Addr, sizeof(Addr)) != 0)
    {
        CFE_EVS_SendEvent(CF_TCP_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR, "bind :%u failed: %s",
                          (unsigned int)Port, strerror(errno));
        close(Fd);
        return -1;
    }
    if (listen(Fd, CF_TCP_MAX_CLIENTS) != 0)
    {
        CFE_EVS_SendEvent(CF_TCP_SOCKET_ERR_EID, CFE_EVS_EventType_ERROR, "listen :%u failed: %s",
                          (unsigned int)Port, strerror(errno));
        close(Fd);
        return -1;
    }
    Flags = fcntl(Fd, F_GETFL, 0);
    if (Flags >= 0)
    {
        fcntl(Fd, F_SETFL, Flags | O_NONBLOCK);
    }
    return Fd;
}

/* -------------------------------------------------------------------------- */

/**
 * Drop the peer and clear the slot.
 *
 * Closing is the only repair the bridge knows: CF's own timers notice the
 * stalled transaction (inactivity/NAK limits) and the engine reconnects,
 * so the pipe never pretends to have retransmitted something it dropped.
 */
static void CF_TCP_CloseClient(CF_TCP_Client_t *Client, const char *Why)
{
    CFE_EVS_SendEvent(CF_TCP_CLIENT_LOST_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "cfdp link closed (%s): rx_pdus=%lu rx_bad=%lu", Why,
                      (unsigned long)Client->RxPdus, (unsigned long)Client->RxDropCount);
    close(Client->Fd);
    memset(Client, 0, sizeof(*Client));
    Client->Fd  = -1;
    Client->Used = false;
    CF_TCP_AppData.LinkEvents++;
}

/**
 * Write one whole frame: length (stream id + PDU), stream id, PDU.
 *
 * Partial writes are link death, not a queue: the peer's framing cannot
 * resynchronise from half a PDU, so the frame that cannot be completed
 * takes the conversation down and the counters say which.
 */
static void CF_TCP_SendFrame(CF_TCP_Client_t *Client, const uint8 *Pdu, size_t PduLen)
{
    size_t LenField = CF_TCP_ID_BYTES + PduLen;
    size_t Total    = CF_TCP_LEN_BYTES + LenField;
    size_t Sent     = 0;

    CF_TCP_TxFrame[0] = (uint8)(LenField >> 24);
    CF_TCP_TxFrame[1] = (uint8)(LenField >> 16);
    CF_TCP_TxFrame[2] = (uint8)(LenField >> 8);
    CF_TCP_TxFrame[3] = (uint8)LenField;
    CF_TCP_TxFrame[4] = 0;
    CF_TCP_TxFrame[5] = 0;
    CF_TCP_TxFrame[6] = 0;
    CF_TCP_TxFrame[7] = CF_TCP_STREAM_ID;
    memcpy(&CF_TCP_TxFrame[CF_TCP_LEN_BYTES + CF_TCP_ID_BYTES], Pdu, PduLen);

    while (Sent < Total)
    {
        ssize_t Written = send(Client->Fd, &CF_TCP_TxFrame[Sent], Total - Sent, MSG_NOSIGNAL);
        if (Written <= 0)
        {
            CF_TCP_AppData.TxErrCount++;
            CFE_EVS_SendEvent(CF_TCP_TX_ERR_EID, CFE_EVS_EventType_ERROR,
                              "PDU write refused after %u/%u B (%s): link reset",
                              (unsigned)Sent, (unsigned)Total, strerror(errno));
            CF_TCP_CloseClient(Client, "partial frame");
            return;
        }
        Sent += (size_t)Written;
    }
    CF_TCP_AppData.TxPdus++;
}

/* -------------------------------------------------------------------------- */

/**
 * Hand one complete inbound PDU to the bus on the CF channel RX MID.
 *
 * The message must look exactly like a ground upload: CF parses command-type
 * messages at CF_PduCmdMsg_t's ph offset, so the primary is stamped by hand
 * from the compile-time MID and the 2-byte command secondary is zero-filled;
 * the PDU begins where CF's command struct says it begins.
 */
static void CF_TCP_InjectPdu(const uint8 *Pdu, size_t PduLen)
{
    CFE_Status_t Status;

    memset(CF_TCP_RxScratch.Bytes, 0, CF_TCP_RX_HDR);
    CF_TCP_RxScratch.Bytes[0] = (uint8)(CF_CH0_RX_MID >> 8);
    CF_TCP_RxScratch.Bytes[1] = (uint8)(CF_CH0_RX_MID & 0xFF);
    CF_TCP_RxScratch.Bytes[2] = 0xC0; /* unsegmented, sequence 0 */
    memcpy(&CF_TCP_RxScratch.Bytes[CF_TCP_RX_HDR], Pdu, PduLen);
    CFE_MSG_SetSize(&CF_TCP_RxScratch.Buf.Msg, (CFE_MSG_Size_t)(CF_TCP_RX_HDR + PduLen));

    Status = CFE_SB_TransmitMsg(&CF_TCP_RxScratch.Buf.Msg, true);
    if (Status != CFE_SUCCESS)
    {
        CF_TCP_AppData.Client[0].RxDropCount++;
        if (CF_TCP_AppData.Client[0].RxDropCount == 1)
        {
            CFE_EVS_SendEvent(CF_TCP_PUBLISH_ERR_EID, CFE_EVS_EventType_ERROR,
                              "PDU inject rejected, status=%08X len=%u", (unsigned int)Status,
                              (unsigned)PduLen);
        }
    }
}

/**
 * Drain the socket, rebuild whole segments, refuse what cannot be framed.
 *
 * The stream id is checked but not yet load-bearing; the length limits are
 * the law: a declared length outside [id+1, id+PDU_MAX] cannot be trusted
 * to ever deliver a clean byte boundary, so the link resets on it.
 */
static void CF_TCP_ServiceRx(CF_TCP_Client_t *Client)
{
    bool Eof = false;

    for (;;)
    {
        ssize_t Got = recv(Client->Fd, &Client->Rx[Client->RxLen],
                           sizeof(Client->Rx) - Client->RxLen, 0);
        if (Got > 0)
        {
            Client->RxLen += (uint32)Got;
            continue;
        }
        Eof = Got == 0; /* FIN means the peer is gone; EAGAIN means quiet */
        break;
    }

    if (Client->RxLen == 0)
    {
        if (Eof)
        {
            CF_TCP_CloseClient(Client, "peer closed");
        }
        return;
    }

    for (;;)
    {
        uint32 LenField;

        if (Client->RxLen < CF_TCP_LEN_BYTES)
        {
            if (Eof)
            {
                if (Client->RxLen > 0)
                {
                    Client->RxDropCount++; /* a header fragment dies with the peer */
                }
                CF_TCP_CloseClient(Client, "peer closed");
            }
            return; /* still waiting for the length itself */
        }
        LenField = ((uint32)Client->Rx[0] << 24) | ((uint32)Client->Rx[1] << 16) |
                   ((uint32)Client->Rx[2] << 8) | (uint32)Client->Rx[3];
        if (LenField < CF_TCP_ID_BYTES + 1 || LenField > CF_TCP_ID_BYTES + CF_TCP_PDU_MAX)
        {
            Client->RxDropCount++;
            CFE_EVS_SendEvent(CF_TCP_FRAME_ERR_EID, CFE_EVS_EventType_ERROR,
                              "bad segment length %lu (want %u..%u): link reset",
                              (unsigned long)LenField, (unsigned)(CF_TCP_ID_BYTES + 1),
                              (unsigned)(CF_TCP_ID_BYTES + CF_TCP_PDU_MAX));
            CF_TCP_CloseClient(Client, "framing violation");
            return;
        }
        if (Client->RxLen < CF_TCP_LEN_BYTES + LenField)
        {
            if (Eof)
            {
                Client->RxDropCount++; /* the rest of this segment never comes */
                CF_TCP_CloseClient(Client, "peer died mid-segment");
                return;
            }
            Client->RxPartial = CF_TCP_LEN_BYTES + LenField - Client->RxLen;
            return; /* mid-segment: patience, the rest is in flight */
        }

        {
            const uint8 *Body  = &Client->Rx[CF_TCP_LEN_BYTES];
            uint32       Id    = ((uint32)Body[0] << 24) | ((uint32)Body[1] << 16) |
                                   ((uint32)Body[2] << 8) | (uint32)Body[3];
            size_t       PduLen = LenField - CF_TCP_ID_BYTES;

            if (Id != CF_TCP_STREAM_ID)
            {
                Client->RxDropCount++;
                CFE_EVS_SendEvent(CF_TCP_FRAME_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "unknown stream id %lu: link reset", (unsigned long)Id);
                CF_TCP_CloseClient(Client, "stream id violation");
                return;
            }
            CF_TCP_InjectPdu(&Body[CF_TCP_ID_BYTES], PduLen);
            Client->RxPdus++;

            Client->RxLen -= CF_TCP_LEN_BYTES + LenField;
            if (Client->RxLen > 0)
            {
                memmove(Client->Rx, &Client->Rx[CF_TCP_LEN_BYTES + LenField], Client->RxLen);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */

static void CF_TCP_AcquirePeer(void)
{
    int ClientFd;

    while ((ClientFd = accept(CF_TCP_AppData.ListenFd, NULL, NULL)) >= 0)
    {
        int Flags = fcntl(ClientFd, F_GETFL, 0);
        int One   = 1;

        if (Flags >= 0)
        {
            fcntl(ClientFd, F_SETFL, Flags | O_NONBLOCK);
        }
        setsockopt(ClientFd, IPPROTO_TCP, TCP_NODELAY, &One, sizeof(One));
        /* A peer that vanishes without a FIN (killed mid-flight, unreachable
         * network) is detected by keepalive - slowly, but without anyone
         * rebooting the craft. The polite goodbye is handled on the next
         * tick; this covers the impolite one. */
        setsockopt(ClientFd, SOL_SOCKET, SO_KEEPALIVE, &One, sizeof(One));

        if (!CF_TCP_AppData.Client[0].Used)
        {
            CF_TCP_Client_t *Client = &CF_TCP_AppData.Client[0];
            memset(Client, 0, sizeof(*Client));
            Client->Fd   = ClientFd;
            Client->Used = true;
            CF_TCP_AppData.ConnectCount++;
            CF_TCP_AppData.LinkEvents++;
            CFE_EVS_SendEvent(CF_TCP_CLIENT_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "cfdp engine connected (total %lu)",
                              (unsigned long)CF_TCP_AppData.ConnectCount);
        }
        else
        {
            /* One file pipe has one peer: the second dialer is refused
             * loudly rather than displacing the conversation in flight.
             * (INFORMATION, not error: refusing a squatter is the app
             * behaving exactly as designed - and the severity ladder has
             * no WARNING rung anyway.) */
            close(ClientFd);
            CFE_EVS_SendEvent(CF_TCP_REFUSE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "second cfdp dialer refused: link already has a peer");
        }
    }
}

static void CF_TCP_ServiceTcp(void)
{
    CF_TCP_AcquirePeer();
    if (CF_TCP_AppData.Client[0].Used)
    {
        CF_TCP_ServiceRx(&CF_TCP_AppData.Client[0]);
    }
}

/* -------------------------------------------------------------------------- */

static int32 CF_TCP_AppInit(void)
{
    CFE_Status_t Status;
    uint16       Port;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    /* Idempotence against a rerun after an ES restart: the whole ledger
     * starts clean, the fd seats start empty, before anything claims them. */
    memset(&CF_TCP_AppData, 0, sizeof(CF_TCP_AppData));
    CF_TCP_AppData.Client[0].Fd = -1;
    CF_TCP_AppData.ListenFd     = -1;

    Status = CFE_SB_CreatePipe(&CF_TCP_AppData.Pipe, 32, "CF_TCP.Pipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(CF_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "pipe create failed, status=%08X", (unsigned int)Status);
        return Status;
    }

    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(CF_CH0_TX_MID), CF_TCP_AppData.Pipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(CF_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "subscribe CF CH0 TX (0x%04X) failed, status=%08X",
                          (unsigned int)CF_CH0_TX_MID, (unsigned int)Status);
        return Status;
    }

    Port                      = CF_TCP_ResolvePort(CF_TCP_PORT_ENV, CF_TCP_DEFAULT_PORT);
    CF_TCP_AppData.ListenFd   = CF_TCP_BindListener(Port);
    if (CF_TCP_AppData.ListenFd < 0)
    {
        /* The bridge is deaf without the listener, but deaf is not dead:
         * CF still runs and its EVS still testifies. Dying here would
         * hide the one app whose silence IS the link's story. */
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    CFE_EVS_SendEvent(CF_TCP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "cfdp bridge up: listening :%u, channel %04X -> socket, socket -> %04X",
                      (unsigned int)Port, (unsigned int)CF_CH0_TX_MID,
                      (unsigned int)CF_CH0_RX_MID);
    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void CF_TCP_AppMain(void)
{
    uint32           RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_MSG_Size_t   MsgSize;
    CFE_Status_t     Status;

    Status = CF_TCP_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, CF_TCP_AppData.Pipe, CF_TCP_SB_TIMEOUT);
            if (Status == CFE_SUCCESS)
            {
                CFE_MSG_GetSize(&BufPtr->Msg, &MsgSize);
                if (MsgSize > CF_TCP_TX_HDR &&
                    (size_t)(MsgSize - CF_TCP_TX_HDR) <= CF_TCP_PDU_MAX &&
                    CF_TCP_AppData.Client[0].Used)
                {
                    CF_TCP_SendFrame(&CF_TCP_AppData.Client[0],
                                     &((const uint8 *)&BufPtr->Msg)[CF_TCP_TX_HDR],
                                     (size_t)MsgSize - CF_TCP_TX_HDR);
                }
                else if (MsgSize > CF_TCP_TX_HDR)
                {
                    CF_TCP_AppData.TxLinkDownDrops++;
                }
            }

            CF_TCP_ServiceTcp();
        }
    }

        CFE_ES_WriteToSysLog("CF_TCP exit: connects=%lu tx=%lu linkdown_drops=%lu tx_err=%lu "
                         "rx_pdus=%lu rx_bad=%lu link_events=%lu\n",
                         (unsigned long)CF_TCP_AppData.ConnectCount,
                         (unsigned long)CF_TCP_AppData.TxPdus,
                         (unsigned long)CF_TCP_AppData.TxLinkDownDrops,
                         (unsigned long)CF_TCP_AppData.TxErrCount,
                         (unsigned long)CF_TCP_AppData.Client[0].RxPdus,
                         (unsigned long)CF_TCP_AppData.Client[0].RxDropCount,
                         (unsigned long)CF_TCP_AppData.LinkEvents);

    CFE_ES_ExitApp(RunStatus);
}
