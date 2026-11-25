/*
 * gs_tcp: generic groundstation TCP gateway (COSMOS <-> software bus).
 *
 * One gateway for ALL of cFS: subscribes to every MsgId in the EDS-generated
 * table (rs_mid_table.h), streams complete CCSDS packets out over TCP as soon
 * as they hit the bus, and injects complete CCSDS command packets received
 * from the ground straight onto the SB. No per-message code lives here; new
 * messages appear in COSMOS by regenerating the table, not by editing this
 * app, replacing the TO_LAB-UDP + separate ingest-app pattern.
 *
 * Uplink and downlink are SEPARATE ports, the way a real bird separates its
 * command receiver from its telemetry beacon: :5050 streams packets out and
 * accepts nothing in, :5051 accepts command frames and never answers. A
 * ground station wired to the wrong port therefore cannot inject (bytes on
 * the beacon port are counted and discarded), which makes the command path a
 * property of the interface, not of client politeness.
 *
 * Framing on the wire is plain CCSDS: the primary header's data-field length
 * octet gives packet extent, so OpenC3 rides a length protocol. Clients may
 * come and go at any time; telemetry with no downlink peer is counted and
 * dropped.
 */

#include "gs_tcp_app.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "cfe.h"
#include "gs_tcp_events.h"

GS_TCP_AppData_t GS_TCP_AppData;

/* scratch copy of one injected command (never bigger than the SB max) */
static union
{
    CFE_SB_Buffer_t Buf;
    uint8           Bytes[CFE_MISSION_SB_MAX_SB_MSG_SIZE];
} GS_TCP_TxScratch;

/* -------------------------------------------------------------------------- */

static int GS_TCP_SendAll(int Fd, const uint8 *Data, size_t Len)
{
    /* Nonblocking client: a busy link drops whole packets rather than
     * stalling the SB or corrupting a frame. COSMOS tolerates telemetry
     * loss and resynchronizes on the next packet boundary. */
    while (Len > 0)
    {
        ssize_t Sent = send(Fd, Data, Len, MSG_NOSIGNAL);
        if (Sent > 0)
        {
            Data += Sent;
            Len -= (size_t)Sent;
            continue;
        }
        if (Sent < 0 && (errno == EINTR))
        {
            continue;
        }
        return -1; /* EAGAIN (client slow) or error: drop packet */
    }
    return 0;
}

/* -------------------------------------------------------------------------- */

static void GS_TCP_Broadcast(const uint8 *Packet, size_t Len)
{
    bool   HadPeer = false;
    size_t Slot;

    /* Downlink pool only, by construction: an uplink socket never sees a
     * byte of telemetry. */
    for (Slot = 0; Slot < GS_TCP_MAX_CLIENTS; Slot++)
    {
        GS_TCP_Client_t *Client = &GS_TCP_AppData.TlmClients[Slot];
        if (Client->Fd >= 0)
        {
            HadPeer = true;
            if (GS_TCP_SendAll(Client->Fd, Packet, Len) == 0)
            {
                Client->TxPackets++;
            }
        }
    }

    if (!HadPeer)
    {
        GS_TCP_AppData.DropNoClientCount++;
        if (GS_TCP_AppData.DropNoClientCount == 1)
        {
            CFE_EVS_SendEvent(GS_TCP_NO_CLIENT_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "telemetry pending but no downlink client connected");
        }
    }
}

/* -------------------------------------------------------------------------- */

static void GS_TCP_InjectCommand(GS_TCP_Client_t *Client, const uint8 *Packet, size_t Len)
{
    CFE_Status_t Status;

    memcpy(GS_TCP_TxScratch.Bytes, Packet, Len);
    Status = CFE_SB_TransmitMsg(&GS_TCP_TxScratch.Buf.Msg, true);
    if (Status == CFE_SUCCESS)
    {
        Client->RxPackets++;
        GS_TCP_AppData.RxTotalCount++;
    }
    else
    {
        Client->RxBadCount++;
        GS_TCP_AppData.RxBadTotal++;
        CFE_EVS_SendEvent(GS_TCP_CMD_INJECT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "command inject failed, status=%08X msgid=%04X len=%u",
                          (unsigned int)Status,
                          (unsigned int)((Packet[0] << 8) | Packet[1]), (unsigned int)Len);
    }
}

/* CCSDS length field: octets in the packet AFTER the primary header. */
static size_t GS_TCP_PacketLength(const uint8 *Header)
{
    return (((size_t)Header[4] << 8) | (size_t)Header[5]) + GS_TCP_PRIMARY_HDR + 1;
}

/* -------------------------------------------------------------------------- */

/**
 * Drop one client and free its slot.
 *
 * \p Role labels the pool in the EVS text so an operator can tell which half
 * of the ground link went away.
 */
static void GS_TCP_CloseClient(GS_TCP_Client_t *Client, const char *Role, const char *Why,
                               uint32 *PoolCount)
{
    CFE_EVS_SendEvent(GS_TCP_CLIENT_DISCONNECT, CFE_EVS_EventType_INFORMATION,
                      "%s client closed (%s, rx=%lu)", Role, Why, (unsigned long)Client->RxPackets);
    close(Client->Fd);
    Client->Fd      = -1;
    Client->Backlog = 0;
    (*PoolCount)--;
}

/* -------------------------------------------------------------------------- */

/**
 * Service one DOWNLINK client's receive side - which is to say: refuse it.
 *
 * Bytes written to the beacon port cannot be commands by policy, so they are
 * counted and discarded (the connection stays up: the peer is almost
 * certainly still listening for telemetry, and dropping it would blind COSMOS
 * over a typo in an interface file).
 */
static void GS_TCP_ServiceDownlinkRx(GS_TCP_Client_t *Client)
{
    uint8   Chunk[GS_TCP_RX_CHUNK];
    ssize_t Got = recv(Client->Fd, Chunk, sizeof(Chunk), 0);

    if (Got > 0)
    {
        GS_TCP_AppData.StrayDownlinkRxBytes += (uint32)Got;
        if (GS_TCP_AppData.StrayDownlinkRxBytes == (uint32)Got)
        {
            CFE_EVS_SendEvent(GS_TCP_STRAY_RX_ERR_EID, CFE_EVS_EventType_ERROR,
                              "command bytes on the beacon port :%u are ignored; uplink is :%u",
                              (unsigned int)GS_TCP_AppData.TlmPort, (unsigned int)GS_TCP_AppData.CmdPort);
        }
        return;
    }
    if (Got == 0)
    {
        GS_TCP_CloseClient(Client, "downlink", "peer closed", &GS_TCP_AppData.TlmClientCount);
    }
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
        GS_TCP_CloseClient(Client, "downlink", strerror(errno), &GS_TCP_AppData.TlmClientCount);
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Service one UPLINK client: accumulate frames, inject every whole one.
 */
static void GS_TCP_ServiceUplinkRx(GS_TCP_Client_t *Client)
{
    uint8   Chunk[GS_TCP_RX_CHUNK];
    ssize_t Got;
    size_t  Need;
    size_t  Total;

    Got = recv(Client->Fd, Chunk, sizeof(Chunk), 0);
    if (Got > 0)
    {
        if ((size_t)Got + (size_t)Client->Backlog > sizeof(Client->Partial))
        {
            Client->RxBadCount++; /* garbage length or peer flood: reset */
            Client->Backlog = 0;
            return;
        }
        memcpy(&Client->Partial[Client->Backlog], Chunk, (size_t)Got);
        Client->Backlog += (size_t)Got;
    }
    else if (Got == 0)
    {
        GS_TCP_CloseClient(Client, "uplink", "peer closed", &GS_TCP_AppData.CmdClientCount);
        return;
    }
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
        GS_TCP_CloseClient(Client, "uplink", strerror(errno), &GS_TCP_AppData.CmdClientCount);
        return;
    }

    /* peel every complete packet off the accumulation buffer */
    while ((size_t)Client->Backlog >= GS_TCP_PRIMARY_HDR)
    {
        Total = GS_TCP_PacketLength(Client->Partial);
        if (Total > sizeof(Client->Partial))
        {
            Client->RxBadCount++;
            Client->Backlog = 0; /* cannot be a real packet: discard */
            return;
        }
        if ((size_t)Client->Backlog < Total)
        {
            return; /* wait for the rest */
        }
        GS_TCP_InjectCommand(Client, Client->Partial, Total);
        Need = (size_t)Client->Backlog - Total;
        if (Need > 0)
        {
            memmove(Client->Partial, &Client->Partial[Total], Need);
        }
        Client->Backlog = (int)Need;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Drain one listener's accept backlog into a client pool.
 */
static void GS_TCP_AcquireClients(int ListenFd, GS_TCP_Client_t *Pool, const char *Role,
                                  uint32 *PoolCount)
{
    int ClientFd;

    /* accept is nonblocking: loop drains the backlog */
    while ((ClientFd = accept(ListenFd, NULL, NULL)) >= 0)
    {
        size_t Slot;
        int    Flags = fcntl(ClientFd, F_GETFL, 0);
        int    One   = 1;

        if (Flags >= 0)
        {
            fcntl(ClientFd, F_SETFL, Flags | O_NONBLOCK);
        }
        setsockopt(ClientFd, IPPROTO_TCP, TCP_NODELAY, &One, sizeof(One));

        for (Slot = 0; Slot < GS_TCP_MAX_CLIENTS; Slot++)
        {
            if (Pool[Slot].Fd < 0)
            {
                Pool[Slot].Fd      = ClientFd;
                Pool[Slot].Backlog = 0;
                (*PoolCount)++;
                GS_TCP_AppData.ConnectCount++;
                CFE_EVS_SendEvent(GS_TCP_CLIENT_CONNECT_INF, CFE_EVS_EventType_INFORMATION,
                                  "%s client connected on slot %u (total %lu)", Role,
                                  (unsigned int)Slot, (unsigned long)GS_TCP_AppData.ConnectCount);
                break;
            }
        }
        if (Slot == GS_TCP_MAX_CLIENTS)
        {
            close(ClientFd); /* full house */
        }
    }
}

/* -------------------------------------------------------------------------- */

static void GS_TCP_ServiceTcp(void)
{
    size_t Slot;

    GS_TCP_AcquireClients(GS_TCP_AppData.TlmListenFd, GS_TCP_AppData.TlmClients, "downlink",
                          &GS_TCP_AppData.TlmClientCount);
    GS_TCP_AcquireClients(GS_TCP_AppData.CmdListenFd, GS_TCP_AppData.CmdClients, "uplink",
                          &GS_TCP_AppData.CmdClientCount);

    for (Slot = 0; Slot < GS_TCP_MAX_CLIENTS; Slot++)
    {
        if (GS_TCP_AppData.TlmClients[Slot].Fd >= 0)
        {
            GS_TCP_ServiceDownlinkRx(&GS_TCP_AppData.TlmClients[Slot]);
        }
        if (GS_TCP_AppData.CmdClients[Slot].Fd >= 0)
        {
            GS_TCP_ServiceUplinkRx(&GS_TCP_AppData.CmdClients[Slot]);
        }
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Bind one of the two listeners. Returns the fd, or -1 after an EVS error.
 */
/* Fleet addressing: a real unit compiles its ports in and never has an
 * environment; the host override exists so N mission containers can fly on
 * one box. Compiled default unless the env names a sane port - a garbage
 * env must degrade to the default, never to a silent wrong port. */
static uint16 GS_TCP_ResolvePort(const char *EnvName, uint16 CompiledDefault)
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
        return CompiledDefault;
    }
    return (uint16)Value;
}

static int GS_TCP_BindListener(uint16 Port)
{
    int                Fd;
    int                One = 1;
    int                Flags;
    struct sockaddr_in Addr;

    Fd = socket(AF_INET, SOCK_STREAM, 0);
    if (Fd < 0)
    {
        CFE_EVS_SendEvent(GS_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "socket failed: %s",
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
        CFE_EVS_SendEvent(GS_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "bind :%u failed: %s",
                          (unsigned int)Port, strerror(errno));
        close(Fd);
        return -1;
    }
    if (listen(Fd, GS_TCP_MAX_CLIENTS) != 0)
    {
        CFE_EVS_SendEvent(GS_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "listen :%u failed: %s",
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

static int32 GS_TCP_AppInit(void)
{
    CFE_Status_t Status;
    size_t       i;
    uint32       Subscribed = 0;

    memset(&GS_TCP_AppData, 0, sizeof(GS_TCP_AppData));
    GS_TCP_AppData.TlmListenFd = -1;
    GS_TCP_AppData.CmdListenFd = -1;
    for (i = 0; i < GS_TCP_MAX_CLIENTS; i++)
    {
        GS_TCP_AppData.TlmClients[i].Fd = -1;
        GS_TCP_AppData.CmdClients[i].Fd = -1;
    }

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&GS_TCP_AppData.TlmPipe, GS_TCP_PIPE_DEPTH, "GS_TCP.TlmPipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(GS_TCP_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "pipe create failed, status=%08X", (unsigned int)Status);
        return Status;
    }

    for (i = 0; i < ORRERY_MID_COUNT; i++)
    {
        Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(ORRERY_MID_TABLE[i].MsgId),
                                  GS_TCP_AppData.TlmPipe);
        if (Status == CFE_SUCCESS)
        {
            Subscribed++;
        }
        else
        {
            CFE_EVS_SendEvent(GS_TCP_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "subscribe 0x%04X (%s) failed, status=%08X",
                              (unsigned int)ORRERY_MID_TABLE[i].MsgId,
                              ORRERY_MID_TABLE[i].Name, (unsigned int)Status);
        }
    }

    GS_TCP_AppData.TlmPort = GS_TCP_ResolvePort("ORRERY_TLM_PORT", GS_TCP_TLM_PORT);
    GS_TCP_AppData.CmdPort = GS_TCP_ResolvePort("ORRERY_CMD_PORT", GS_TCP_CMD_PORT);

    GS_TCP_AppData.TlmListenFd = GS_TCP_BindListener(GS_TCP_AppData.TlmPort);
    GS_TCP_AppData.CmdListenFd = GS_TCP_BindListener(GS_TCP_AppData.CmdPort);
    if (GS_TCP_AppData.TlmListenFd < 0 || GS_TCP_AppData.CmdListenFd < 0)
    {
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }

    CFE_ES_WaitForStartupSync(true);

    CFE_EVS_SendEvent(GS_TCP_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "gs_tcp downlink :%u uplink :%u, subscribed %lu/%u MsgIds from EDS table",
                      (unsigned int)GS_TCP_AppData.TlmPort, (unsigned int)GS_TCP_AppData.CmdPort,
                      (unsigned long)Subscribed, (unsigned int)ORRERY_MID_COUNT);

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

void GS_TCP_AppMain(void)
{
    uint32           RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_MSG_Size_t   PacketSize;
    CFE_Status_t     Status;

    Status = GS_TCP_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            Status = CFE_SB_ReceiveBuffer(&BufPtr, GS_TCP_AppData.TlmPipe, GS_TCP_SB_TIMEOUT);
            if (Status == CFE_SUCCESS)
            {
                CFE_MSG_GetSize(&BufPtr->Msg, &PacketSize);
                if (PacketSize >= GS_TCP_PRIMARY_HDR &&
                    PacketSize <= CFE_MISSION_SB_MAX_SB_MSG_SIZE)
                {
                    GS_TCP_Broadcast((const uint8 *)&BufPtr->Msg, (size_t)PacketSize);
                }
            }

            GS_TCP_ServiceTcp();
        }
    }

    CFE_ES_WriteToSysLog("GS_TCP exit: connects=%lu injected=%lu bad=%lu stray_beacon=%lu\n",
                         (unsigned long)GS_TCP_AppData.ConnectCount,
                         (unsigned long)GS_TCP_AppData.RxTotalCount,
                         (unsigned long)GS_TCP_AppData.RxBadTotal,
                         (unsigned long)GS_TCP_AppData.StrayDownlinkRxBytes);

    CFE_ES_ExitApp(RunStatus);
}
