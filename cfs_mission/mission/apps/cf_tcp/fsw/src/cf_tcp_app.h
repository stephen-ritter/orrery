/* cf_tcp: the CFDP transport bridge - CF's channel bytes to one ground socket.

   CF 4.0 has no sockets: every CFDP PDU it emits is one SB message on the
   channel's TX MID, and every PDU it must send on arrives as one SB message
   on the channel's RX MID. This app is the other half of that contract: a
   listener the ground CFDP engine dials into, writing each outbound PDU as
   one length-framed segment and turning each inbound segment into one SB
   message. Nothing here speaks CFDP: no acks, no retries, no file
   knowledge - reliability belongs to CF (class 2), and a pipe that
   invents its own reliability would be the second, lying one.

   The framing law is the mission's: delimitation is all CCSDS 727.0-B-5
   (2.2.5) asks of the bearer, and it names no shape, so the mission's is
   a 4-byte big-endian length covering the 4-byte stream id and the PDU,
   that id (value 1), then the raw CFDP PDU. The length math is
   the entire interop risk, so every byte count that disagrees is a counter,
   an EVS line, and a refused frame - never a partial read.
*/
#ifndef CF_TCP_APP_H
#define CF_TCP_APP_H

#include "cfe.h"

/* One ground conversation at a time: a real file pipe has one peer, and a
 * second dialer displacing the first is its own EVS story, not a queue. */
#define CF_TCP_MAX_CLIENTS      1

/* SB receive timeout (ms). The app must service the socket even when the
 * bus is silent: the link's connect/disconnect testimony cannot wait for
 * CF to have something to say. */
#define CF_TCP_SB_TIMEOUT       500

/* The listener's compiled default (fleet law: a unit compiles its ports;
 * the env override exists so the yard can fly N birds on one box). */
#define CF_TCP_DEFAULT_PORT     5060
#define CF_TCP_PORT_ENV         "ORRERY_CF_PORT"

/* Mission framing law: 4-byte length, 4-byte stream id, then the PDU. */
#define CF_TCP_LEN_BYTES        4
#define CF_TCP_ID_BYTES         4
#define CF_TCP_STREAM_ID        1u
#define CF_TCP_PDU_MAX          1024 /* CF_MAX_PDU_SIZE is 512; headroom, not hope */

/* One accepted ground connection and its read fragment ledger. */
typedef struct
{
    int      Fd;
    uint32   RxPdus;         /* well-framed segments handed to the bus        */
    uint32   RxDropCount;    /* malformed or oversized segments refused       */
    uint64   RxPartial;      /* bytes buffered mid-segment (framing witness)  */
    uint8    Rx[CF_TCP_LEN_BYTES + CF_TCP_ID_BYTES + CF_TCP_PDU_MAX];
    uint32   RxLen;          /* bytes currently held in Rx                    */
    uint32   RxWant;         /* total bytes the open segment needs            */
    bool     Used;
} CF_TCP_Client_t;

typedef struct
{
    CFE_SB_PipeId_t Pipe;
    int             ListenFd;
    CF_TCP_Client_t Client[CF_TCP_MAX_CLIENTS];

    uint32 TxPdus;           /* CF PDUs written to the socket                 */
    uint32 TxLinkDownDrops;  /* PDUs dropped because nobody was connected     */
    uint32 TxErrCount;       /* socket writes that failed after connect       */
    uint32 ConnectCount;     /* ground peers accepted since boot              */
    uint32 LinkEvents;       /* up/down transitions, for the throttled witness */
} CF_TCP_AppData_t;

extern CF_TCP_AppData_t CF_TCP_AppData;

#endif /* CF_TCP_APP_H */
