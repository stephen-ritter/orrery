/*
 * bsk_link: Basilisk <-> cFS bridge application.
 *
 * Connects as a ZeroMQ SUB to the Basilisk scenario and decodes the sensor feed with
 * the mission's own EDS schema. The frames are the packed payload of
 * BSK_LINK/SensorFrame, and the database that describes them is the one edslink/
 * compiles from the same XML that codegen/eds2cosmos turns into the ground
 * definitions - so this app, the simulator and COSMOS cannot disagree about a field
 * order without the build failing.
 *
 * Wire format: ZMQ multipart, frame 1 the topic ("SENSOR"), frame 2 the EDS payload.
 * Payload fields are target-native little-endian, which is what the XML declares and
 * what EdsLib therefore reads here (see edslink/README.md for the measurement).
 */

#include "bsk_link_app.h"

#include <errno.h>
#include <stdio.h> /* snprintf, for naming fitted rows in an event line */
#include <stdlib.h>
#include <string.h>
#include <zmq.h>

#include "cfe.h"
#include "cfe_time_msg.h" /* TIME command structs + FC for the link timecode sync */
#include "bsk_link_events.h"
#include "edslib_binding_objects.h"
#include "edslib_displaydb.h"
#include "orrery_bus_msg.h"
#include "orrery_msgids.h"

/* The compiled mission database, linked in whole-archive by this app's CMakeLists
 * (orrery_eds_db.a, produced by edslink/). Declared by name rather than included:
 * the generated database header is a build artifact, the symbol is the contract. */
extern const EdsLib_DatabaseObject_t ORRERY_DATABASE;

BSK_LinkAppData_t BSK_LinkAppData;

static void *BSK_LinkZmqContext  = NULL;
static void *BSK_LinkZmqSocket   = NULL;

/* Fleet addressing: resolved once at init from the host env, else the
 * compiled defaults - a real unit compiles its link in and has no env.
 * Copies, not pointers: the env is untrusted and may change under us. */
#define BSK_LINK_ENDPOINT_MAX 128
static char BSK_LinkSensorEndpoint[BSK_LINK_ENDPOINT_MAX] = BSK_LINK_ZMQ_ENDPOINT;
static char BSK_LinkCmdEndpoint[BSK_LINK_ENDPOINT_MAX]    = BSK_LINK_ZMQ_CMD_ENDPOINT;
static bool  BSK_LinkZmqConnected = false;

/* Egress (cFS -> sim) PUSH socket, kept separate from the SUB socket: a SUB
 * socket cannot write, and the two sides have different lifecycles - the
 * telemetry feed must reconnect invisibly, while a stalled command queue is
 * a condition worth counting. */
static void *BSK_LinkZmqPushSocket = NULL;

/*
 * Identity of BSK_LINK/SensorFrame inside the linked-in database. Resolved at startup
 * from its EDS name and cross-checked against the constant the generated header
 * derives from the same XML: the name is what a human reads in an EVS event, the
 * constant is what the compiler checked, and agreeing proves this app and the schema
 * were built from the same tree.
 */
static EdsLib_Id_t BSK_LinkSensorTypeId = EDSLIB_ID_INVALID;

/*
 * Decode scratch space, allocated once at startup and reused every frame. EdsLib
 * reference-counts managed buffers and has no public release call, so allocating per
 * frame would leak one buffer per frame for the life of the app; a single buffer held
 * for the app's whole life has nothing to release. This app is single threaded.
 */
static EdsLib_Binding_Buffer_Content_t *BSK_LinkFrameBuffer = NULL;

/* Boot handshake: the FitManifest container, resolved like every other
 * inbound schema. The handshake is NOT a registry row (it is the link's own
 * control chatter, with no MsgId and no ground definition), so it has its own
 * id and buffer beside the sensor frame's. */
static EdsLib_Id_t                      BSK_LinkFitTypeId = EDSLIB_ID_INVALID;
static EdsLib_Binding_Buffer_Content_t *BSK_LinkFitBuffer = NULL;
static size_t                           BSK_LinkFitFrameSize = 0;

/* Wire length of one SensorFrame, straight out of the schema: a frame that is not
 * exactly this long cannot be this container, and saying so beats a decode error that
 * looks like corruption. */
static size_t BSK_LinkSensorFrameSize = 0;

/* World frames: observability parts no longer each own a triple of
 * statics - the generated registry table names them, and one array slot per
 * row holds what was once per-part ritual: resolved EDS identity, private
 * scratch buffer (decoding one frame must never scribble on another), and
 * the schema-derived wire length. */
static EdsLib_Id_t                      BSK_LinkWorldTypeIds[BSK_LINK_FRAME_ROWS];
static EdsLib_Binding_Buffer_Content_t *BSK_LinkWorldBuffers[BSK_LINK_FRAME_ROWS];

/* -------------------------------------------------------------------------- */

/**
 * Report a decode failure once, with the reason.
 *
 * A schema mismatch is a build-time fault, not a runtime condition: without this the
 * symptom is a silently dead link, which is the worst possible failure mode for a
 * telemetry path.
 */
static void BSK_Link_ReportDecodeFailure(const char *Where, size_t Size, size_t Expected)
{
    if (BSK_LinkAppData.MalformedMsgCount == 0)
    {
        CFE_EVS_SendEvent(BSK_LINK_DECODE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "link frame decode failed at %s (payload %u bytes, schema says %u)", Where,
                          (unsigned int)Size, (unsigned int)Expected);
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Read one scalar field out of a decoded container.
 *
 * Returns the EDS basic type of what it found, or EDSLIB_BASICTYPE_NONE when the
 * schema has no such field or it is not a scalar. During development a NONE means the
 * XML and this app have drifted apart, which is exactly the failure this decode path
 * exists to make loud.
 */
static EdsLib_BasicType_t BSK_Link_ReadScalar(const EdsLib_Binding_DescriptorObject_t *Frame, const char *Name,
                                              double *Out)
{
    EdsLib_DataTypeDB_EntityInfo_t    MemberInfo;
    EdsLib_Binding_DescriptorObject_t Field;
    EdsLib_GenericValueBuffer_t       Value;
    uint16                            SubIndex;

    if (EdsLib_DisplayDB_GetIndexByName(&ORRERY_DATABASE, Frame->EdsId, Name, &SubIndex) != EDSLIB_SUCCESS ||
        EdsLib_DataTypeDB_GetMemberByIndex(&ORRERY_DATABASE, Frame->EdsId, SubIndex, &MemberInfo) != EDSLIB_SUCCESS)
    {
        return EDSLIB_BASICTYPE_NONE;
    }

    EdsLib_Binding_InitSubObject(&Field, Frame, &MemberInfo);
    memset(&Value, 0, sizeof(Value));
    if (EdsLib_Binding_LoadValue(&Field, &Value) != EDSLIB_SUCCESS)
    {
        return EDSLIB_BASICTYPE_NONE;
    }

    switch (Value.ValueType)
    {
        case EDSLIB_BASICTYPE_FLOAT:
            /* LoadValue stores floats in the union's FloatingPoint member (a long
             * double); reading fpdbl instead reinterprets the x87 mantissa bits as
             * a double - which for 5.0 is exactly -0.0. Verified live. */
            *Out = (double)Value.Value.FloatingPoint;
            return Value.ValueType;
        case EDSLIB_BASICTYPE_SIGNED_INT:
            *Out = (double)Value.Value.SignedInteger;
            return Value.ValueType;
        case EDSLIB_BASICTYPE_UNSIGNED_INT:
            *Out = (double)Value.Value.UnsignedInteger;
            return Value.ValueType;
        default:
            return EDSLIB_BASICTYPE_NONE;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Decode one BSK_LINK/SensorFrame payload.
 *
 * OmegaBnB is a Vector3 array, so it is walked element by element after the container
 * itself has been unpacked.
 */
static bool BSK_Link_DecodeSensorFrame(const void *Payload, size_t Size)
{
    EdsLib_Binding_DescriptorObject_t Frame;
    EdsLib_DataTypeDB_EntityInfo_t    MemberInfo;
    EdsLib_Binding_DescriptorObject_t Omega;
    uint16                            SubIndex;

    /*
     * A descriptor without a backing buffer cannot be unpacked into, which is exactly
     * what the first attempt here got wrong (silently: every frame failed to decode).
     * So attach the startup buffer, apply the EDS-declared defaults, and only then
     * unpack the wire bytes over it - the same order the EdsLib CPython binding uses.
     */
    EdsLib_Binding_InitDescriptor(&Frame, &ORRERY_DATABASE, BSK_LinkSensorTypeId);
    EdsLib_Binding_SetDescBuffer(&Frame, BSK_LinkFrameBuffer);
    EdsLib_Binding_InitStaticFields(&Frame);

    if (Size != BSK_LinkSensorFrameSize)
    {
        BSK_Link_ReportDecodeFailure("size", Size, BSK_LinkSensorFrameSize);
        return false;
    }

    if (EdsLib_Binding_InitFromPackedBuffer(&Frame, Payload, Size) != EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure("unpack", Size, BSK_LinkSensorFrameSize);
        return false;
    }

    if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_SIMTIME, &BSK_LinkAppData.LastSimTimeS) != EDSLIB_BASICTYPE_FLOAT)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_SIMTIME, Size, BSK_LinkSensorFrameSize);
        return false;
    }

    /* Frame is a counter, not a measurement: accept either integer width, reject
     * anything else, and never reinterpret the storage. */
    {
        double FrameValue = 0.0;
        if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_FRAME, &FrameValue) == EDSLIB_BASICTYPE_NONE)
        {
            BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_FRAME, Size, BSK_LinkSensorFrameSize);
            return false;
        }
        BSK_LinkAppData.LastFrame = (uint64)FrameValue;
    }

    if (EdsLib_DisplayDB_GetIndexByName(&ORRERY_DATABASE, Frame.EdsId, BSK_LINK_FIELD_OMEGA, &SubIndex) !=
        EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_OMEGA, Size, BSK_LinkSensorFrameSize);
        return false;
    }
    if (EdsLib_DataTypeDB_GetMemberByIndex(&ORRERY_DATABASE, Frame.EdsId, SubIndex, &MemberInfo) != EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_OMEGA, Size, BSK_LinkSensorFrameSize);
        return false;
    }

    EdsLib_Binding_InitSubObject(&Omega, &Frame, &MemberInfo);
    for (uint16 Axis = 0; Axis < BSK_LINK_OMEGA_AXES; ++Axis)
    {
        EdsLib_DataTypeDB_EntityInfo_t    ElemInfo;
        EdsLib_Binding_DescriptorObject_t Elem;
        EdsLib_GenericValueBuffer_t       Value;

        if (EdsLib_DataTypeDB_GetMemberByIndex(&ORRERY_DATABASE, Omega.EdsId, Axis, &ElemInfo) != EDSLIB_SUCCESS)
        {
            BSK_Link_ReportDecodeFailure("OmegaBnB[]", Size, BSK_LinkSensorFrameSize);
            return false;
        }
        EdsLib_Binding_InitSubObject(&Elem, &Omega, &ElemInfo);
        memset(&Value, 0, sizeof(Value));
        if (EdsLib_Binding_LoadValue(&Elem, &Value) != EDSLIB_SUCCESS ||
            Value.ValueType != EDSLIB_BASICTYPE_FLOAT)
        {
            return false;
        }
        BSK_LinkAppData.OmegaBnB[Axis] = (double)Value.Value.FloatingPoint;
    }

    return true;
}

/* -------------------------------------------------------------------------- */

/**
 * Put the last decoded sensor frame on the bus as BSK_LINK_SENSOR_FRAME_TLM.
 *
 * The payload copies the fields that came out of the EdsLib decode, so this
 * MsgId is byte-for-byte what the link carried (orrery_bus_msg.h pins the
 * layout; sim/wire_audit.py proves it against the generated defs on the
 * wire). One EVS event on the first publish failure, then counting: a
 * rejected publish means the control loop is open, which must never be quiet.
 */
static void BSK_Link_PublishSensorFrame(void)
{
    CFE_Status_t Status;

    BSK_LinkAppData.SensorFrameTlm.Payload.SimTimeS = BSK_LinkAppData.LastSimTimeS;
    memcpy(BSK_LinkAppData.SensorFrameTlm.Payload.OmegaBnB, BSK_LinkAppData.OmegaBnB,
           sizeof(BSK_LinkAppData.SensorFrameTlm.Payload.OmegaBnB));
    BSK_LinkAppData.SensorFrameTlm.Payload.Frame = BSK_LinkAppData.LastFrame;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BSK_LinkAppData.SensorFrameTlm.TelemetryHeader));
    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(BSK_LinkAppData.SensorFrameTlm.TelemetryHeader), true);
    if (Status != CFE_SUCCESS && BSK_LinkAppData.SensorTlmPublishFail == 0)
    {
        CFE_EVS_SendEvent(BSK_LINK_PUBLISH_ERR_EID, CFE_EVS_EventType_ERROR,
                          "sensor frame publish rejected: 0x%08x", (unsigned int)Status);
    }
    if (Status != CFE_SUCCESS)
    {
        BSK_LinkAppData.SensorTlmPublishFail++;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Validate one member of a world frame, descending through composites.
 *
 * Scalars must LOAD (a corrupt buffer fails right here); arrays refuse an
 * atomic load by design, so their elements are validated one level down.
 * The depth cap is generous for anything the EDS tree declares - a schema
 * that nests deeper than this has bigger problems than this walk.
 */
static bool BSK_Link_ValidateMember(const EdsLib_Binding_DescriptorObject_t *Parent, uint16 Index, size_t Size,
                                    uint16 Want, unsigned Depth)
{
    EdsLib_DataTypeDB_EntityInfo_t    Info;
    EdsLib_DataTypeDB_TypeInfo_t      SubInfo;
    EdsLib_Binding_DescriptorObject_t Obj;
    EdsLib_GenericValueBuffer_t       Value;

    if (Depth > 4)
    {
        BSK_Link_ReportDecodeFailure("member depth", Size, Want);
        return false;
    }
    if (EdsLib_DataTypeDB_GetMemberByIndex(&ORRERY_DATABASE, Parent->EdsId, Index, &Info) != EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure("member", Size, Want);
        return false;
    }
    EdsLib_Binding_InitSubObject(&Obj, Parent, &Info);
    memset(&Value, 0, sizeof(Value));
    if (EdsLib_Binding_LoadValue(&Obj, &Value) == EDSLIB_SUCCESS)
    {
        if (Value.ValueType == EDSLIB_BASICTYPE_NONE)
        {
            BSK_Link_ReportDecodeFailure("member value", Size, Want);
            return false;
        }
        return true;
    }

    /* A composite (array) refuses atomic load: descend instead. An ATOMIC
     * member that refuses is a genuine failure, not a shape difference. */
    if (EdsLib_DataTypeDB_GetTypeInfo(&ORRERY_DATABASE, Obj.EdsId, &SubInfo) != EDSLIB_SUCCESS ||
        SubInfo.NumSubElements == 0)
    {
        BSK_Link_ReportDecodeFailure("member value", Size, Want);
        return false;
    }
    for (uint16 Sub = 0; Sub < SubInfo.NumSubElements; ++Sub)
    {
        if (!BSK_Link_ValidateMember(&Obj, Sub, Size, Want, Depth + 1))
        {
            return false;
        }
    }
    return true;
}

/**
 * Generic world-frame consumer: decode, validate, publish.
 *
 * Observability parts used to ship a bespoke decoder and publisher each
 * hand-walking fields into a payload struct, copied to the bus.
 * The registry knows each container now, so ONE function serves every row
 * of orrery_frames.h:
 *
 *   - the size must be exactly what the row declares,
 *   - EdsLib must unpack the packed buffer for the container,
 *   - EVERY top-level member must load - a whole-schema validation pass
 *     with no per-field C, so a container that grew a field still fails
 *     loudly, just not by name,
 *   - SimTimeS and Frame are read by name for the event line (every world
 *     frame carries them; the SENSOR control path keeps its own named walk
 *     because its values drive physics,
 *   - the published payload is the RAW container bytes: the EDS container
 *     IS the bus image (plain LE doubles, no padding), so what this puts
 *     on the bus is byte-identical to the typed messages of old, and fcs,
 *     obc_hk, and the generated ground defs all keep casting the same
 *     structs they always did.
 *
 * Knowledge stays observability: a disabled row (its schema never resolved)
 * or a malformed frame is counted, never fatal - missing knowledge changes
 * the screen, not the physics.
 */
static bool BSK_Link_WorldFrame(unsigned RowIndex, const void *Payload, size_t Size)
{
    const BskLink_WorldFrameDef_t        *Row        = &BskLink_WorldFrames[RowIndex];
    EdsLib_Binding_DescriptorObject_t     Frame;
    EdsLib_DataTypeDB_TypeInfo_t          TypeInfo;
    double                                SimTime    = 0.0;
    double                                FrameValue = 0.0;
    BSK_Link_WorldFrameTlm_t             *Msg        = &BSK_LinkAppData.WorldFrameTlm[RowIndex];
    CFE_Status_t                          Status;

    if (!EdsLib_Is_Valid(BSK_LinkWorldTypeIds[RowIndex]))
    {
        return false; /* init already said so on the event bus: never guess a schema */
    }

    EdsLib_Binding_InitDescriptor(&Frame, &ORRERY_DATABASE, BSK_LinkWorldTypeIds[RowIndex]);
    EdsLib_Binding_SetDescBuffer(&Frame, BSK_LinkWorldBuffers[RowIndex]);
    EdsLib_Binding_InitStaticFields(&Frame);

    if (Size != Row->Size)
    {
        BSK_Link_ReportDecodeFailure("size", Size, Row->Size);
        return false;
    }
    if (EdsLib_Binding_InitFromPackedBuffer(&Frame, Payload, Size) != EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure("unpack", Size, Row->Size);
        return false;
    }

    /* Validation pass over the whole container: every declared member must
     * load out of this buffer (composites descend). Type-agnostic by design
     * - the registry made field-by-field C unnecessary, but a decode that
     * checked nothing would publish garbage as telemetry. */
    if (EdsLib_DataTypeDB_GetTypeInfo(&ORRERY_DATABASE, Frame.EdsId, &TypeInfo) != EDSLIB_SUCCESS ||
        TypeInfo.NumSubElements == 0)
    {
        BSK_Link_ReportDecodeFailure("members", Size, Row->Size);
        return false;
    }
    for (uint16 Index = 0; Index < TypeInfo.NumSubElements; ++Index)
    {
        if (!BSK_Link_ValidateMember(&Frame, Index, Size, Row->Size, 0))
        {
            return false;
        }
    }

    /* The event line reads its two constants by name: SimTimeS and Frame
     * head every world frame (the same convention the sensor walk uses). */
    if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_SIMTIME, &SimTime) != EDSLIB_BASICTYPE_FLOAT)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_SIMTIME, Size, Row->Size);
        return false;
    }
    if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_FRAME, &FrameValue) == EDSLIB_BASICTYPE_NONE)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_FRAME, Size, Row->Size);
        return false;
    }

    memcpy(Msg->Payload, Payload, Row->Size);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(Msg->TelemetryHeader));
    Status = CFE_SB_TransmitMsg(CFE_MSG_PTR(Msg->TelemetryHeader), true);
    if (Status != CFE_SUCCESS)
    {
        if (BSK_LinkAppData.WorldFramePublishFail[RowIndex] == 0)
        {
            CFE_EVS_SendEvent(BSK_LINK_PUBLISH_ERR_EID, CFE_EVS_EventType_ERROR,
                              "%s publish rejected: 0x%08x", Row->Label, (unsigned int)Status);
        }
        BSK_LinkAppData.WorldFramePublishFail[RowIndex]++;
    }

    BSK_LinkAppData.WorldFrameCounts[RowIndex]++;
    if (Row->IdField != NULL)
    {
        /* The streamed twin of the manifest's claim, built from this frame's
         * own words: read the row's id field by name. NULL never lands here,
         * and a field name the container does not carry reads as NONE - the
         * id then never counts, and the handshake's own SILENT verdict names
         * the claim that failed to arrive. The typo polices itself. */
        double Id = 0.0;
        if (BSK_Link_ReadScalar(&Frame, Row->IdField, &Id) != EDSLIB_BASICTYPE_NONE && Id >= 1.0 && Id <= 32.0)
        {
            BSK_LinkAppData.FitStreamedIds[Row->IdSlot] |= 1u << ((unsigned)Id - 1u);
        }
    }
    if ((BSK_LinkAppData.WorldFrameCounts[RowIndex] == 1) ||
        ((BSK_LinkAppData.WorldFrameCounts[RowIndex] % BSK_LINK_SENSOR_EVENT_EVERY) == 0))
    {
        CFE_EVS_SendEvent(Row->EvsId, CFE_EVS_EventType_INFORMATION,
                          "basilisk %s t=%.1fs frame=%llu total=%lu", Row->Label, SimTime,
                          (unsigned long long)FrameValue,
                          (unsigned long)BSK_LinkAppData.WorldFrameCounts[RowIndex]);
    }
    return true;
}

static int BSK_Link_WorldFrameLookup(const char *Topic)
{
    for (unsigned Index = 0; Index < BSK_LINK_FRAME_ROWS; ++Index)
    {
        if (strcmp(Topic, BskLink_WorldFrames[Index].Topic) == 0)
        {
            return (int)Index;
        }
    }
    return -1;
}

/* -------------------------------------------------------------------------- */

/**
 * Slave the mission clock to the link timecode (the sim is the
 * spacecraft's time source - the HIL pattern in software).
 *
 * Every sensor frame carries SimTimeS, the sim instant the frame describes.
 * Promote it to the mission clock with the same TIME ground command the
 * operator uses (SET_MET): on a real bird a timecode unit does this once per
 * tone; here the link does it once per frame - and a frame is one sim second
 * by construction. A frozen sim freezes MET: onboard time advances exactly
 * when sim time does, and the time-tagged schedule waits with the sim.
 * Between frames
 * the TIME task's 1 Hz software tone free-runs - sub-second wall-clock slop
 * on pure observability traffic, which is all a watching link is.
 * Control-path logic reads the frame's
 * SimTimeS (frame-accurate by definition), never the CFE clock.
 *
 * TIME answers every SET_MET with its own EVS event and counts it in its HK:
 * the sync is audible on the console and auditable on the beacon.
 */
static void BSK_Link_SyncMissionTime(void)
{
    CFE_TIME_SetMETCmd_t Cmd;
    double               Frac;

    memset(&Cmd, 0, sizeof(Cmd));
    if (CFE_MSG_Init(CFE_MSG_PTR(Cmd.CommandHeader), CFE_SB_ValueToMsgId(ORRERY_TIME_CMD_MID),
                     sizeof(Cmd)) != CFE_SUCCESS)
    {
        return; /* fixed mid and size: impossible, but never send half a command */
    }
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(Cmd.CommandHeader), CFE_TIME_SET_MET_CC);

    Cmd.Payload.Seconds = (uint32)BSK_LinkAppData.LastSimTimeS;
    Frac                = BSK_LinkAppData.LastSimTimeS - (double)Cmd.Payload.Seconds;
    if (Frac < 0.0 || Frac >= 1.0)
    {
        Frac = 0.0; /* malformed timecode: whole seconds still beat garbage */
    }
    Cmd.Payload.MicroSeconds = (uint32)(Frac * 1000000.0);

    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(Cmd.CommandHeader), true) == CFE_SUCCESS)
    {
        BSK_LinkAppData.TimeSyncCount++;
        if (BSK_LinkAppData.TimeSyncCount == 1)
        {
            CFE_EVS_SendEvent(BSK_LINK_TIME_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "mission MET slaved to link timecode: %u s",
                              (unsigned int)Cmd.Payload.Seconds);
        }
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Write one payload out the egress PUSH as a multipart frame [topic, payload].
 *
 * The bus payload is already the little-endian BSK_LINK wire layout (same
 * schema, same packing, checked by _Static_assert at build and by the sim's
 * EdsLib decode at the far end), so encode and forward are the same memcpy,
 * and neither can drift from the XML.
 *
 * Returns false if there is no socket or either part was rejected (queue
 * full). On the mixed case - topic sent, payload refused - the peer would see
 * a one-frame message; loopback PUSH with HWM headroom has not done that, and
 * both callers count the drop either way.
 */
static bool BSK_Link_EgressFrame(const void *Payload, size_t Size, const char *TopicText)
{
    zmq_msg_t Topic;
    zmq_msg_t Out;

    if (BSK_LinkZmqPushSocket == NULL)
    {
        return false;
    }

    zmq_msg_init_size(&Topic, strlen(TopicText));
    memcpy(zmq_msg_data(&Topic), TopicText, strlen(TopicText));
    zmq_msg_init_size(&Out, Size);
    memcpy(zmq_msg_data(&Out), Payload, Size);

    if (zmq_msg_send(&Topic, BSK_LinkZmqPushSocket, ZMQ_SNDMORE | ZMQ_DONTWAIT) > 0 &&
        zmq_msg_send(&Out, BSK_LinkZmqPushSocket, ZMQ_DONTWAIT) > 0)
    {
        return true;
    }

    /* msg_send consumed the message only on success; on failure the
     * messages are still owned here and must be released. */
    zmq_msg_close(&Topic);
    zmq_msg_close(&Out);
    return false;
}

/* -------------------------------------------------------------------------- */

/**
 * Forward one torque command from the bus out to the Basilisk sim.
 */
static void BSK_Link_ForwardTorque(const CFE_SB_Buffer_t *BufPtr)
{
    const CFE_MSG_Message_t *Msg    = &BufPtr->Msg;
    size_t                   MsgLen = 0;
    BskTorqueCmd_Payload_t   Body;

    BSK_LinkAppData.TorqueRecvCount++;

    /* Take the payload by copying out of the bus buffer: the buffer is only
     * valid during this callback and the send must not alias it. */
    CFE_MSG_GetSize(Msg, &MsgLen);
    if (MsgLen != sizeof(BskTorqueCmdTlm_t))
    {
        /* A wrong length here would be a struct/schema drift; the sim's decode
         * would reject it anyway, so count it here where it is attributable. */
        BSK_LinkAppData.TorqueDropCount++;
        return;
    }
    memcpy(&Body, (const uint8 *)Msg + sizeof(CFE_MSG_TelemetryHeader_t), sizeof(Body));

    if (BSK_Link_EgressFrame(&Body, sizeof(Body), BSK_LINK_CMD_TOPIC))
    {
        BSK_LinkAppData.TorqueFwdCount++;
        if ((BSK_LinkAppData.TorqueFwdCount == 1) ||
            ((BSK_LinkAppData.TorqueFwdCount % BSK_LINK_TORQUE_EVENT_EVERY) == 0))
        {
            CFE_EVS_SendEvent(BSK_LINK_TORQUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "torque forwarded t=%.1fs tx=%+.5f %+.5f %+.5f total=%lu", Body.SimTimeS,
                              Body.TorqueSumB[0], Body.TorqueSumB[1], Body.TorqueSumB[2],
                              (unsigned long)BSK_LinkAppData.TorqueFwdCount);
        }
    }
    else
    {
        if (BSK_LinkAppData.TorqueDropCount == 0)
        {
            CFE_EVS_SendEvent(BSK_LINK_EGRESS_ERR_EID, CFE_EVS_EventType_ERROR,
                              "torque dropped: no egress capacity (%s)", BSK_LinkCmdEndpoint);
        }
        BSK_LinkAppData.TorqueDropCount++;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Forward one lockstep barrier ack from the bus out to the Basilisk sim.
 *
 * This frame is the sim's clock: its gate holds the next Basilisk step until
 * the ack for the frame it published arrives, so an ack that never leaves
 * cFS does not drop a nicety, it stops the simulation (loudly, on a timeout,
 * with this counter nonzero as the reason).
 */
static void BSK_Link_ForwardStepAck(const CFE_SB_Buffer_t *BufPtr)
{
    const CFE_MSG_Message_t *Msg    = &BufPtr->Msg;
    size_t                   MsgLen = 0;
    BskStepAck_Payload_t     Body;

    BSK_LinkAppData.AckRecvCount++;

    CFE_MSG_GetSize(Msg, &MsgLen);
    if (MsgLen != sizeof(BskStepAckTlm_t))
    {
        BSK_LinkAppData.AckDropCount++;
        return;
    }
    memcpy(&Body, (const uint8 *)Msg + sizeof(CFE_MSG_TelemetryHeader_t), sizeof(Body));

    if (BSK_Link_EgressFrame(&Body, sizeof(Body), BSK_LINK_ACK_TOPIC))
    {
        BSK_LinkAppData.AckFwdCount++;
        if ((BSK_LinkAppData.AckFwdCount == 1) ||
            ((BSK_LinkAppData.AckFwdCount % BSK_LINK_TORQUE_EVENT_EVERY) == 0))
        {
            CFE_EVS_SendEvent(BSK_LINK_TORQUE_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "step ack forwarded t=%.1fs frame=%llu mode=%u total=%lu", Body.SimTimeS,
                              (unsigned long long)Body.Frame, (unsigned int)Body.Mode,
                              (unsigned long)BSK_LinkAppData.AckFwdCount);
        }
    }
    else
    {
        if (BSK_LinkAppData.AckDropCount == 0)
        {
            CFE_EVS_SendEvent(BSK_LINK_ACK_ERR_EID, CFE_EVS_EventType_ERROR,
                              "step ack dropped: no egress capacity (%s); sim will stall at its gate",
                              BSK_LinkCmdEndpoint);
        }
        BSK_LinkAppData.AckDropCount++;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Forward one accepted camera config from the bus out to the Basilisk sim.
 *
 * The sim drains this topic between ticks and applies it at the next
 * frame boundary; nothing here waits for an answer. A failed egress is rare
 * (configs are commanded, not streamed) and loud: the ground will otherwise
 * see CAM_HK.RenderRateCmd frozen against CamState.RenderRateS and chase the
 * wrong wire.
 */
static void BSK_Link_ForwardCamConfig(const CFE_SB_Buffer_t *BufPtr)
{
    const CFE_MSG_Message_t *Msg    = &BufPtr->Msg;
    size_t                   MsgLen = 0;
    BskCamConfig_Payload_t   Body;

    BSK_LinkAppData.CamRecvCount++;

    CFE_MSG_GetSize(Msg, &MsgLen);
    if (MsgLen != sizeof(BskCamConfigTlm_t))
    {
        BSK_LinkAppData.CamDropCount++;
        return;
    }
    memcpy(&Body, (const uint8 *)Msg + sizeof(CFE_MSG_TelemetryHeader_t), sizeof(Body));

    if (BSK_Link_EgressFrame(&Body, sizeof(Body), BSK_LINK_CAM_TOPIC))
    {
        BSK_LinkAppData.CamFwdCount++;
        CFE_EVS_SendEvent(BSK_LINK_CAM_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "cam config forwarded t=%.1fs camera=%llu shutter=%llu rate=%.3fs total=%lu",
                          Body.SimTimeS, (unsigned long long)Body.CameraId, (unsigned long long)Body.Shutter,
                          Body.RenderRateS, (unsigned long)BSK_LinkAppData.CamFwdCount);
    }
    else
    {
        if (BSK_LinkAppData.CamDropCount == 0)
        {
            CFE_EVS_SendEvent(BSK_LINK_EGRESS_ERR_EID, CFE_EVS_EventType_ERROR,
                              "cam config dropped: no egress capacity (%s)", BSK_LinkCmdEndpoint);
        }
        BSK_LinkAppData.CamDropCount++;
    }
}

/* -------------------------------------------------------------------------- */

/**
 * Pull one multipart frame off the feed, if one is fully queued, and account for it.
 *
 * Reading topic and payload together matters: a topic consumed without its payload
 * desynchronises every later frame from its topic, and a payload read without its
 * topic loses the channel identity. A poll first means a half-arrived message is
 * simply left in the queue for the next cycle.
 */
/* -------------------------------------------------------------------------- */

/**
 * Name the rows a mask selects, for an event line a human can act on.
 *
 * Truncation is deliberate: the event text has a limit and a fleet's part list
 * is short; a mask wider than the line still reports its bits.
 */
static void BSK_Link_FitMaskNames(uint32 Mask, char *Out, size_t Size)
{
    size_t Used = 0;

    Out[0] = '\0';
    for (unsigned Index = 0; Index < BSK_LINK_FRAME_ROWS; ++Index)
    {
        if ((Mask & (1u << Index)) == 0)
        {
            continue;
        }
        const int Written = snprintf(Out + Used, Size - Used, "%s%s", (Used ? "," : ""),
                                     BskLink_WorldFrames[Index].Label);
        if (Written < 0 || (size_t)Written >= Size - Used)
        {
            break;
        }
        Used += (size_t)Written;
    }
    if (Used == 0)
    {
        snprintf(Out, Size, "-");
    }
}

/**
 * Compare the three views of this vehicle's fit.
 *
 * The world's manifest CLAIMS a fit; this binary was BUILT from a registry
 * digest; the bus STREAMS whatever actually decoded. The two ways they can
 * disagree are categorically different, and the code says so:
 *
 *   - The DIGESTS differ: this firmware predates (or postdates) the catalog the
 *     world is running. Sizes may match while layouts differ, so a decoded
 *     field could be another field's bytes. A mission must not fly on data it
 *     may be mislabelling: latch the abort, refuse the barrier, and say both
 *     digests out loud. The sim then stalls with a named cause instead of
 *     quietly drifting on invented knowledge.
 *
 *   - The BITS differ with the digests agreed: a claimed part that never
 *     streams (SILENT), or a stream the manifest denies (DENIED). Both are
 *     facts about this vehicle, not lies about the schema, so the mission flies
 *     and the event names the rows - absence stays declared, never zero-filled
 *     and an undeclared stream stays an alarm rather than telemetry.
 *
 * Events fire on TRANSITIONS only: the manifest arrives every tick, and a
 * verdict that repeated itself at 1 Hz would drown the console it reports to.
 */
static const char *BSK_Link_FitSlotLabel(uint8 Slot)
{
    for (unsigned Index = 0; Index < BSK_LINK_FRAME_ROWS; ++Index)
    {
        if (BskLink_WorldFrames[Index].IdSlot == (int8_t)Slot)
        {
            return BskLink_WorldFrames[Index].Label;
        }
    }
    return "instance";
}

static unsigned BSK_Link_FitFirstId(uint32 Mask)
{
    for (unsigned Bit = 0; Bit < 32; ++Bit)
    {
        if ((Mask & (1u << Bit)) != 0)
        {
            return Bit + 1;
        }
    }
    return 0;
}

static void BSK_Link_EvaluateFit(void)
{
    BSK_LinkAppData_t *App = &BSK_LinkAppData;
    char               Claimed[96];
    char               Denied[96];
    uint8              Verdict;

    App->FitStreamingBits = 0;
    for (unsigned Index = 0; Index < BSK_LINK_FRAME_ROWS; ++Index)
    {
        if (App->WorldFrameCounts[Index] > 0)
        {
            App->FitStreamingBits |= (1u << Index);
        }
    }

    if (App->FitDeclaredHash != BSK_LINK_FIT_REGISTRY_HASH)
    {
        App->FitSilentBits = 0;
        App->FitDeniedBits = 0;
        App->FitIdBits     = 0;
        Verdict            = BSK_LINK_FIT_STALE;
    }
    else if (App->FitFrameCount <= BSK_LINK_FIT_GRACE_TICKS)
    {
        /* No verdict yet. The mask of what has streamed is meaningless until
         * every declared part has had a tick or five to arrive, and a
         * MATCHED printed over an empty bus is not agreement - it is the
         * absence of evidence. Silence here is honest: the operator gets one
         * verdict per boot, not a guess and then a correction. */
        App->FitSilentBits = 0;
        App->FitDeniedBits = 0;
        App->FitIdBits     = 0;
        App->FitVerdict    = BSK_LINK_FIT_UNKNOWN;
        return;
    }
    else
    {
        App->FitSilentBits = App->FitDeclaredBits & ~App->FitStreamingBits;
        App->FitDeniedBits = App->FitStreamingBits & ~App->FitDeclaredBits;
        App->FitIdBits     = 0;
        App->FitIdDenied   = false;
        if (App->FitSilentBits != 0)
        {
            Verdict = BSK_LINK_FIT_SILENT;
        }
        else if (App->FitDeniedBits != 0)
        {
            Verdict = BSK_LINK_FIT_DENIED;
        }
        else
        {
            /* Rows agree; are the instances honest? An id claim no frame
             * carried, or an id on the bus the manifest never named, is the
             * same defect one tier down - it names the instance that
             * stalled, not just the part that went quiet. */
            Verdict = BSK_LINK_FIT_MATCHED;
            for (uint8 Slot = 0; Slot < BSK_LINK_FIT_ID_SLOTS; ++Slot)
            {
                uint32 IdSilent = App->FitDeclaredIds[Slot] & ~App->FitStreamedIds[Slot];
                uint32 IdDenied = App->FitStreamedIds[Slot] & ~App->FitDeclaredIds[Slot];
                if (IdSilent != 0)
                {
                    App->FitIdBits     = IdSilent;
                    App->FitIdSlot     = Slot;
                    App->FitIdDenied   = false;
                    Verdict            = BSK_LINK_FIT_SILENT;
                    break;
                }
                if (IdDenied != 0)
                {
                    App->FitIdBits   = IdDenied;
                    App->FitIdSlot   = Slot;
                    App->FitIdDenied = true;
                    Verdict          = BSK_LINK_FIT_DENIED;
                    break;
                }
            }
        }
    }

    if (Verdict == App->FitVerdict)
    {
        return;
    }
    App->FitVerdict = Verdict;

    switch (Verdict)
    {
        case BSK_LINK_FIT_STALE:
            App->FitAbort = true;
            CFE_EVS_SendEvent(BSK_LINK_FIT_ERR_EID, CFE_EVS_EventType_CRITICAL,
                              "FIT ABORT: firmware built from registry %08x, world runs %08x - declared=%08x. "
                              "Rebuild (cfs_mission/build.sh) or match the scenario; refusing the barrier.",
                              (unsigned int)BSK_LINK_FIT_REGISTRY_HASH, (unsigned int)App->FitDeclaredHash,
                              (unsigned int)App->FitDeclaredBits);
            break;
        case BSK_LINK_FIT_SILENT:
            if (App->FitIdBits != 0)
            {
                CFE_EVS_SendEvent(BSK_LINK_FIT_REPORT_EID, CFE_EVS_EventType_ERROR,
                                  "FIT SILENT: manifest claims %s id %u but no frame carried it - "
                                  "ids declared=%08x streamed=%08x",
                                  BSK_Link_FitSlotLabel(App->FitIdSlot), BSK_Link_FitFirstId(App->FitIdBits),
                                  (unsigned int)App->FitDeclaredIds[App->FitIdSlot],
                                  (unsigned int)App->FitStreamedIds[App->FitIdSlot]);
            }
            else
            {
                BSK_Link_FitMaskNames(App->FitSilentBits, Claimed, sizeof(Claimed));
                CFE_EVS_SendEvent(BSK_LINK_FIT_REPORT_EID, CFE_EVS_EventType_ERROR,
                                  "FIT SILENT: manifest claims %s (%08x) but the bus never streamed it - "
                                  "channel stays off, declared=%08x streamed=%08x",
                                  Claimed, (unsigned int)App->FitSilentBits, (unsigned int)App->FitDeclaredBits,
                                  (unsigned int)App->FitStreamingBits);
            }
            break;
        case BSK_LINK_FIT_DENIED:
            if (App->FitIdBits != 0)
            {
                CFE_EVS_SendEvent(BSK_LINK_FIT_REPORT_EID, CFE_EVS_EventType_ERROR,
                                  "FIT DENIED: bus streams %s id %u that the manifest denies - "
                                  "ids declared=%08x streamed=%08x",
                                  BSK_Link_FitSlotLabel(App->FitIdSlot), BSK_Link_FitFirstId(App->FitIdBits),
                                  (unsigned int)App->FitDeclaredIds[App->FitIdSlot],
                                  (unsigned int)App->FitStreamedIds[App->FitIdSlot]);
            }
            else
            {
                BSK_Link_FitMaskNames(App->FitDeniedBits, Denied, sizeof(Denied));
                CFE_EVS_SendEvent(BSK_LINK_FIT_REPORT_EID, CFE_EVS_EventType_ERROR,
                                  "FIT DENIED: bus streams %s (%08x) that the manifest denies - "
                                  "declared=%08x streamed=%08x",
                                  Denied, (unsigned int)App->FitDeniedBits, (unsigned int)App->FitDeclaredBits,
                                  (unsigned int)App->FitStreamingBits);
            }
            break;
        case BSK_LINK_FIT_MATCHED:
            CFE_EVS_SendEvent(BSK_LINK_FIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "FIT MATCHED: declared=%08x streamed=%08x registry=%08x - manifest, build, and bus agree",
                              (unsigned int)App->FitDeclaredBits, (unsigned int)App->FitStreamingBits,
                              (unsigned int)BSK_LINK_FIT_REGISTRY_HASH);
            break;
        default:
            break;
    }
}

/**
 * Decode and judge one BSK_LINK/FitManifest.
 *
 * The handshake fields are the declared fit mask, the registry digest, and
 * the instance-id slots; SimTimeS and Frame exist so the frame reads like
 * every other frame on the wire, and so a stale counter in the operator's
 * log is a stale counter rather than an unreadable packet.
 */
static bool BSK_Link_DecodeFitFrame(const void *Payload, size_t Size)
{
    EdsLib_Binding_DescriptorObject_t Frame;
    double                            Bits = 0.0;
    double                            Hash = 0.0;

    if (!EdsLib_Is_Valid(BSK_LinkFitTypeId))
    {
        return false; /* init already said so on the event bus */
    }

    EdsLib_Binding_InitDescriptor(&Frame, &ORRERY_DATABASE, BSK_LinkFitTypeId);
    EdsLib_Binding_SetDescBuffer(&Frame, BSK_LinkFitBuffer);
    EdsLib_Binding_InitStaticFields(&Frame);

    if (Size != BSK_LinkFitFrameSize)
    {
        BSK_Link_ReportDecodeFailure("fit size", Size, BSK_LinkFitFrameSize);
        return false;
    }
    if (EdsLib_Binding_InitFromPackedBuffer(&Frame, Payload, Size) != EDSLIB_SUCCESS)
    {
        BSK_Link_ReportDecodeFailure("fit unpack", Size, BSK_LinkFitFrameSize);
        return false;
    }
    if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_DECLARED_BITS, &Bits) == EDSLIB_BASICTYPE_NONE)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_DECLARED_BITS, Size, BSK_LinkFitFrameSize);
        return false;
    }
    if (BSK_Link_ReadScalar(&Frame, BSK_LINK_FIELD_REGISTRY_HASH, &Hash) == EDSLIB_BASICTYPE_NONE)
    {
        BSK_Link_ReportDecodeFailure(BSK_LINK_FIELD_REGISTRY_HASH, Size, BSK_LinkFitFrameSize);
        return false;
    }
    for (unsigned Slot = 0; Slot < BSK_LINK_FIT_ID_SLOTS; ++Slot)
    {
        char   Field[16];
        double Mask = 0.0;
        snprintf(Field, sizeof(Field), "DeclaredIds%u", Slot);
        if (BSK_Link_ReadScalar(&Frame, Field, &Mask) == EDSLIB_BASICTYPE_NONE)
        {
            BSK_Link_ReportDecodeFailure(Field, Size, BSK_LinkFitFrameSize);
            return false;
        }
        BSK_LinkAppData.FitDeclaredIds[Slot] = (uint32)Mask;
    }

    BSK_LinkAppData.FitDeclaredBits = (uint32)Bits;
    BSK_LinkAppData.FitDeclaredHash = (uint32)Hash;
    BSK_LinkAppData.FitFrameCount++;
    BSK_Link_EvaluateFit();
    return true;
}

/* Store outlet: the sim's IMG chunks become /cf/img files.
 *
 * The framing is orrery's own and deliberately OUTSIDE the EDS model (the
 * STEP_ACK precedent for non-ground frames): a packed 24 B header - cam id,
 * capture ordinal, part index, part total - then the binary slice. TCP
 * orders, ZMQ frames, and NOTHING here acks: an incomplete capture stays a
 * `.part` file, a visible corpse on the record rather than a retransmit
 * request. Paths ride OSAL so the PSP's /cf mapping keeps its meaning even
 * when the yard moves the binary's cwd. */
static void BSK_Link_EmitCamStorage(void)
{
    BSK_LinkAppData.CamStorageTlm.Payload.SimTimeS    = BSK_LinkAppData.LastSimTimeS;
    BSK_LinkAppData.CamStorageTlm.Payload.Frame       = BSK_LinkAppData.LastFrame;
    BSK_LinkAppData.CamStorageTlm.Payload.FilesStored = BSK_LinkAppData.FilesStored;
    BSK_LinkAppData.CamStorageTlm.Payload.BytesStored = BSK_LinkAppData.BytesStored;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(BSK_LinkAppData.CamStorageTlm.TelemetryHeader));
    if (CFE_SB_TransmitMsg(CFE_MSG_PTR(BSK_LinkAppData.CamStorageTlm.TelemetryHeader), true) != CFE_SUCCESS)
    {
        BSK_LinkAppData.StorePublishFail++;
        if (BSK_LinkAppData.StorePublishFail == 1)
        {
            CFE_EVS_SendEvent(BSK_LINK_PUBLISH_ERR_EID, CFE_EVS_EventType_ERROR,
                              "cam storage ledger publish rejected");
        }
    }
}

static void BSK_Link_IngestImageChunk(const void* Data, size_t Size)
{
    BskImageChunkHeader_t Header;
    const uint8*          Slice = (const uint8*)Data;
    char                  PartPath[64];
    char                  FinalPath[64];
    int32                 OsStatus;
    size_t                SliceLen;

    BSK_LinkAppData.ImgChunkCount++;
    if (Size <= sizeof(Header) || Size - sizeof(Header) > (size_t)BSK_LINK_IMG_SLICE_MAX)
    {
        BSK_LinkAppData.ImgDropCount++;
        return;
    }
    memcpy(&Header, Data, sizeof(Header));
    SliceLen = Size - sizeof(Header);

    if (Header.Part == 0)
    {
        /* a new capture: any still-open stream is a dropped corpse; close
         * the fd but LEAVE the .part file - evidence, not garbage */
        if (OS_ObjectIdDefined(BSK_LinkAppData.StoreFd))
        {
            OS_close(BSK_LinkAppData.StoreFd);
            BSK_LinkAppData.StoreFd = OS_OBJECT_ID_UNDEFINED;
        }
        snprintf(PartPath, sizeof(PartPath), "/cf/img/.%llu_%llu.part",
                 (unsigned long long)Header.CamId, (unsigned long long)Header.Capture);
        OsStatus = OS_OpenCreate(&BSK_LinkAppData.StoreFd, PartPath,
                                 OS_FILE_FLAG_CREATE | OS_FILE_FLAG_TRUNCATE, OS_WRITE_ONLY);
        if (OsStatus != OS_SUCCESS)
        {
            BSK_LinkAppData.StoreFd     = OS_OBJECT_ID_UNDEFINED;
            BSK_LinkAppData.ImgDropCount++;
            if (BSK_LinkAppData.ImgDropCount == 1)
            {
                CFE_EVS_SendEvent(BSK_LINK_STORE_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "store cannot open %s (status %d)", PartPath, (int)OsStatus);
            }
            return;
        }
        BSK_LinkAppData.StoreCam      = Header.CamId;
        BSK_LinkAppData.StoreCapture  = Header.Capture;
        BSK_LinkAppData.StoreTotal    = Header.Total;
        BSK_LinkAppData.StoreNextPart = 0;
        BSK_LinkAppData.StoreBytes    = 0;
    }
    else if (!OS_ObjectIdDefined(BSK_LinkAppData.StoreFd) ||
             Header.CamId != BSK_LinkAppData.StoreCam ||
             Header.Capture != BSK_LinkAppData.StoreCapture ||
             Header.Part != BSK_LinkAppData.StoreNextPart || Header.Total != BSK_LinkAppData.StoreTotal)
    {
        /* out of order, foreign, or mid-stream metadata drift: count it,
         * do not chase it - the sender never hears back, by law */
        BSK_LinkAppData.ImgDropCount++;
        return;
    }

    OsStatus = OS_write(BSK_LinkAppData.StoreFd, Slice + sizeof(Header), SliceLen);
    if (OsStatus != (int32)SliceLen)
    {
        OS_close(BSK_LinkAppData.StoreFd);
        BSK_LinkAppData.StoreFd = OS_OBJECT_ID_UNDEFINED;
        BSK_LinkAppData.ImgDropCount++;
        if (BSK_LinkAppData.ImgDropCount == 1)
        {
            CFE_EVS_SendEvent(BSK_LINK_STORE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "store write refused (status %d, wanted %u)", (int)OsStatus,
                              (unsigned)SliceLen);
        }
        return;
    }
    BSK_LinkAppData.StoreBytes += (uint64)SliceLen;
    BSK_LinkAppData.StoreNextPart++;

    if (Header.Part + 1u < Header.Total)
    {
        return; /* mid-capture: silence is the normal voice here */
    }

    /* the last byte landed: move the file into daylight under its real name */
    snprintf(PartPath, sizeof(PartPath), "/cf/img/.%llu_%llu.part",
             (unsigned long long)Header.CamId, (unsigned long long)Header.Capture);
    snprintf(FinalPath, sizeof(FinalPath), "/cf/img/frame_%llu_%llu.png",
             (unsigned long long)Header.CamId, (unsigned long long)Header.Capture);
    OS_close(BSK_LinkAppData.StoreFd);
    BSK_LinkAppData.StoreFd = OS_OBJECT_ID_UNDEFINED;
    if (OS_mv(PartPath, FinalPath) != OS_SUCCESS)
    {
        BSK_LinkAppData.ImgDropCount++;
        CFE_EVS_SendEvent(BSK_LINK_STORE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "store rename failed into %s", FinalPath);
        return;
    }
    BSK_LinkAppData.FilesStored++;
    BSK_LinkAppData.BytesStored += BSK_LinkAppData.StoreBytes;
    BSK_Link_EmitCamStorage();
    if (BSK_LinkAppData.FilesStored == 1 ||
        BSK_LinkAppData.FilesStored % BSK_LINK_SENSOR_EVENT_EVERY == 0)
    {
        CFE_EVS_SendEvent(BSK_LINK_STORE_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "cam stored %s (%llu B) files=%llu bytes=%llu", FinalPath,
                          (unsigned long long)BSK_LinkAppData.StoreBytes,
                          (unsigned long long)BSK_LinkAppData.FilesStored,
                          (unsigned long long)BSK_LinkAppData.BytesStored);
    }
}

static void BSK_Link_DrainOneFrame(void)
{
    zmq_pollitem_t Item;
    zmq_msg_t      Topic;
    zmq_msg_t      Body;
    size_t         TopicLen;
    int            More;
    size_t         MoreLen;
    /* Roomier than the topic itself: sizing this to strlen("SENSOR") left no NUL
     * slot, so the comparison below could never match and every frame was quietly
     * counted as a foreign topic. */
    char TopicText[32];

    memset(&Item, 0, sizeof(Item));
    Item.socket = BSK_LinkZmqSocket;
    Item.events = ZMQ_POLLIN;
    if (zmq_poll(&Item, 1, 0) <= 0)
    {
        return;
    }

    if (zmq_msg_init(&Topic) != 0)
    {
        return;
    }
    if (zmq_msg_recv(&Topic, BSK_LinkZmqSocket, ZMQ_DONTWAIT) <= 0)
    {
        zmq_msg_close(&Topic);
        return;
    }

    TopicLen = zmq_msg_size(&Topic);
    memset(TopicText, 0, sizeof(TopicText));
    if (TopicLen < sizeof(TopicText))
    {
        memcpy(TopicText, zmq_msg_data(&Topic), TopicLen);
    }
    zmq_msg_close(&Topic);

    /* The payload must belong to the message whose topic we just took: if RCVMORE
     * is not set, a first frame arrived without its pair and reading on would
     * consume the NEXT message's topic as this message's payload, desynchronising
     * every frame after it. Never seen for small loopback frames, but the pairing
     * guarantee is only real if it is enforced here. */
    More    = 0;
    MoreLen = sizeof(More);
    if (zmq_getsockopt(BSK_LinkZmqSocket, ZMQ_RCVMORE, &More, &MoreLen) != 0 || !More)
    {
        BSK_LinkAppData.MalformedMsgCount++;
        return;
    }

    if (zmq_msg_init(&Body) != 0)
    {
        return;
    }
    if (zmq_msg_recv(&Body, BSK_LinkZmqSocket, ZMQ_DONTWAIT) <= 0)
    {
        zmq_msg_close(&Body);
        return;
    }

    /* The contract is exactly [topic, payload]. Frames beyond that belong to this
     * message and must be consumed with it: leaving one behind makes it the next
     * message's topic. */
    for (;;)
    {
        zmq_msg_t Extra;

        More    = 0;
        MoreLen = sizeof(More);
        if (zmq_getsockopt(BSK_LinkZmqSocket, ZMQ_RCVMORE, &More, &MoreLen) != 0 || !More)
        {
            break;
        }
        if (zmq_msg_init(&Extra) != 0)
        {
            break;
        }
        if (zmq_msg_recv(&Extra, BSK_LinkZmqSocket, ZMQ_DONTWAIT) <= 0)
        {
            zmq_msg_close(&Extra);
            break;
        }
        zmq_msg_close(&Extra);
        BSK_LinkAppData.MalformedMsgCount++;
    }

    BSK_LinkAppData.MsgRecvCount++;

    if (strcmp(TopicText, BSK_LINK_FIT_TOPIC) == 0)
    {
        /* Read even while aborting, so the console keeps saying what the world
         * claims now. The ABORT ITSELF is latched until restart, on purpose: a
         * mission that refused its ticks has already broken the lockstep gate's
         * timing, and a link that resumed on its own after a world edit would
         * hide the very mistake this verdict exists to make loud. */
        if (!BSK_Link_DecodeFitFrame(zmq_msg_data(&Body), zmq_msg_size(&Body)))
        {
            BSK_LinkAppData.MalformedMsgCount++;
        }
        zmq_msg_close(&Body);
        return;
    }

    if (strcmp(TopicText, BSK_LINK_SENSOR_TOPIC) == 0)
    {
        if (BSK_LinkAppData.FitAbort)
        {
            /* A mission that cannot say WHICH parts it is flying does not
             * fly. Refusing the tick stalls the lockstep gate, which names the
             * stall in its own log; silently decoding a fit known to be
             * mislabelled would be the worse failure. */
            BSK_LinkAppData.FitRefusedTicks++;
            zmq_msg_close(&Body);
            return;
        }
        if (!BSK_Link_DecodeSensorFrame(zmq_msg_data(&Body), zmq_msg_size(&Body)))
        {
            BSK_LinkAppData.MalformedMsgCount++;
            zmq_msg_close(&Body);
            return;
        }

        BSK_LinkAppData.SensorMsgCount++;

        /* The decoded frame becomes mission telemetry: fcs runs its control law
         * on this MsgId, and the beacon streams it, so COSMOS sees exactly what
         * the satellite senses - nothing the simulator knows can sneak in,
         * because only decoded link bytes are copied into the payload. */
        BSK_Link_PublishSensorFrame();

        /* The timecode this frame carries is now the mission clock. */
        BSK_Link_SyncMissionTime();

        /* EVS on the first frame and then once per N, so a 1 Hz feed does not
         * flood the event bus. Values here came out of the EDS decode. */
        if ((BSK_LinkAppData.SensorMsgCount == 1) ||
            ((BSK_LinkAppData.SensorMsgCount % BSK_LINK_SENSOR_EVENT_EVERY) == 0))
        {
            CFE_EVS_SendEvent(BSK_LINK_SENSOR_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "basilisk sensor frame t=%.1fs frame=%llu wx=%+.5f wy=%+.5f wz=%+.5f total=%lu",
                              BSK_LinkAppData.LastSimTimeS, (unsigned long long)BSK_LinkAppData.LastFrame,
                              BSK_LinkAppData.OmegaBnB[0], BSK_LinkAppData.OmegaBnB[1], BSK_LinkAppData.OmegaBnB[2],
                              (unsigned long)BSK_LinkAppData.SensorMsgCount);
        }
    }
    else if (strcmp(TopicText, BSK_LINK_IMG_TOPIC) == 0)
    {
        /* Store outlet: bytes become files. Fire-and-forget by
         * design - counted and reported, never acked; the record of a
         * damaged transfer is the .part file itself. */
        BSK_Link_IngestImageChunk(zmq_msg_data(&Body), zmq_msg_size(&Body));
    }
    else
    {
        /* Observability frames are registry rows: one generic consumer
         * per topic in orrery_frames.h, each on its own scratch space and its
         * own MsgId, never part of the barrier. Failure is counted, not
         * fatal - knowledge missing changes the screen, not the physics. */
        const int RowIndex = BSK_Link_WorldFrameLookup(TopicText);

        if (RowIndex >= 0)
        {
            if (!BSK_Link_WorldFrame((unsigned)RowIndex, zmq_msg_data(&Body), zmq_msg_size(&Body)))
            {
                BSK_LinkAppData.MalformedMsgCount++;
            }
        }
        else
        {
            BSK_LinkAppData.TopicSkipCount++;
            if (BSK_LinkAppData.TopicSkipCount == 1)
            {
                CFE_EVS_SendEvent(BSK_LINK_TOPIC_ERR_EID, CFE_EVS_EventType_ERROR,
                                  "link frame on unexpected topic '%s'", TopicText);
            }
        }
    }

    zmq_msg_close(&Body);
}

/* -------------------------------------------------------------------------- */

static void BSK_Link_ResolveEndpoint(char *Buffer, const char *EnvName)
{
    const char *Env = getenv(EnvName);

    if (Env != NULL && Env[0] != '\0')
    {
        strncpy(Buffer, Env, BSK_LINK_ENDPOINT_MAX - 1);
        Buffer[BSK_LINK_ENDPOINT_MAX - 1] = '\0';
    }
    /* else: keep the compiled default already in the buffer */
}

static int32 BSK_Link_AppInit(void)
{
    CFE_Status_t Status;

    memset(&BSK_LinkAppData, 0, sizeof(BSK_LinkAppData));

    /* Fleet blocks: before any socket or EVS uses the endpoints. */
    BSK_Link_ResolveEndpoint(BSK_LinkSensorEndpoint, "ORRERY_ZMQ_SENSOR_ENDPOINT");
    BSK_Link_ResolveEndpoint(BSK_LinkCmdEndpoint, "ORRERY_ZMQ_CMD_ENDPOINT");

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS)
    {
        return Status;
    }

    Status = CFE_SB_CreatePipe(&BSK_LinkAppData.CmdPipe, BSK_LINK_PIPE_DEPTH, "BSK_LINK.Pipe");
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR, "pipe create failed, status=%08x",
                          (unsigned int)Status);
        return Status;
    }

    /* The control loop closes through this app: fcs publishes torque on the bus
     * and bsk_link is its only exit to the sim. A failed subscribe would leave
     * the satellite unable to act on its own decisions, so it is an init-time
     * error, loudly, not a silent drop. */
    CFE_MSG_Init(CFE_MSG_PTR(BSK_LinkAppData.SensorFrameTlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(BSK_LINK_SENSOR_FRAME_TLM_MID), sizeof(BSK_LinkAppData.SensorFrameTlm));
    CFE_MSG_Init(CFE_MSG_PTR(BSK_LinkAppData.CamStorageTlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(BSK_LINK_CAM_STORAGE_TLM_MID), sizeof(BSK_LinkAppData.CamStorageTlm));
    /* The store's home, and the idle seat of a fd that is only
     * ever real mid-capture. mkdir is best-effort by law (EEXIST is fine);
     * if the directory genuinely cannot exist, the first chunk's EVS line
     * says so loudly instead of the first frame dying quietly. */
    BSK_LinkAppData.StoreFd = OS_OBJECT_ID_UNDEFINED;
    OS_mkdir("/cf/img", 0);
    for (unsigned RowIndex = 0; RowIndex < BSK_LINK_FRAME_ROWS; ++RowIndex)
    {
        /* One pre-initialized message per registry row: header sized once,
         * like the stock apps, so sequence counts flow. Total length is the
         * 16 B header plus the container bytes the registry declared. */
        CFE_MSG_Init(CFE_MSG_PTR(BSK_LinkAppData.WorldFrameTlm[RowIndex].TelemetryHeader),
                     CFE_SB_ValueToMsgId(BskLink_WorldFrames[RowIndex].Mid),
                     sizeof(CFE_MSG_TelemetryHeader_t) + BskLink_WorldFrames[RowIndex].Size);
    }
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BSK_LINK_TORQUE_COMMAND_TLM_MID), BSK_LinkAppData.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(BSK_LINK_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "torque subscribe failed, status=%08x; control egress DEAD", (unsigned int)Status);
    }
    /* The lockstep gate depends on this subscription exactly as the control
     * loop depends on the one above: an unforwarded ack stalls the sim. */
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BSK_LINK_STEP_ACK_TLM_MID), BSK_LinkAppData.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(BSK_LINK_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "step ack subscribe failed, status=%08x; lockstep gate DEAD", (unsigned int)Status);
    }
    /* Camera config egress. A failed subscribe costs a bird its
     * eye's command surface, never its control loop: loud, not fatal. */
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(BSK_LINK_CAM_CONFIG_TLM_MID), BSK_LinkAppData.CmdPipe);
    if (Status != CFE_SUCCESS)
    {
        CFE_EVS_SendEvent(BSK_LINK_SUBSCRIBE_ERR_EID, CFE_EVS_EventType_ERROR,
                          "cam config subscribe failed, status=%08x; camera command surface DEAD",
                          (unsigned int)Status);
    }

    /* Resolve the linked-in database before accepting any traffic: if the schema is
     * not there, every frame would otherwise count as malformed. Looking the type
     * name up also proves the display half of the database survived the link
     * (whole-archive is what keeps it there at all).
     *
     * The name is the identity we resolve at runtime. The generated constant from
     * bsk_link_eds_defines.h is a data-type-dictionary INDEX, not a packed EdsLib_Id_t
     * (which also encodes genre/app/format), so the two are not comparable - the
     * constant's job is to prove this app was compiled against the same EDS tree. */
    EdsLib_Binding_Initialize();
    BSK_LinkSensorTypeId = EdsLib_DisplayDB_LookupTypeName(&ORRERY_DATABASE, BSK_LINK_SENSOR_TYPE_NAME);

    if (!EdsLib_Is_Valid(BSK_LinkSensorTypeId))
    {
        BSK_LinkSensorTypeId = EDSLIB_ID_INVALID;
        CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "EDS database has no %s; link decode disabled", BSK_LINK_SENSOR_TYPE_NAME);
    }
    else if (!EdsLib_Is_DataType(BSK_LinkSensorTypeId))
    {
        /* Name resolved to something that is not a data type: wrong package, wrong
         * kind of entity. Either way, decoding it would produce nonsense. */
        CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "%s is not an EDS data type (id=%lu); link decode disabled", BSK_LINK_SENSOR_TYPE_NAME,
                          (unsigned long)BSK_LinkSensorTypeId);
        BSK_LinkSensorTypeId = EDSLIB_ID_INVALID;
    }

    if (EdsLib_Is_Valid(BSK_LinkSensorTypeId))
    {
        EdsLib_DataTypeDB_DerivedTypeInfo_t DerivInfo;

        if (EdsLib_DataTypeDB_GetDerivedInfo(&ORRERY_DATABASE, BSK_LinkSensorTypeId, &DerivInfo) != EDSLIB_SUCCESS)
        {
            DerivInfo.MaxSize.Bytes = 0;
        }
        BSK_LinkSensorFrameSize = DerivInfo.MaxSize.Bytes;
        BSK_LinkFrameBuffer = EdsLib_Binding_AllocManagedBuffer(DerivInfo.MaxSize.Bytes);
        if (BSK_LinkFrameBuffer == NULL)
        {
            CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                              "no memory for a %u byte sensor frame; link decode disabled",
                              (unsigned int)DerivInfo.MaxSize.Bytes);
            BSK_LinkSensorTypeId = EDSLIB_ID_INVALID;
        }
    }

    /* Referenced so the generated header stays load-bearing: if the BSK_LINK package
     * ever disappears from the EDS tree, this app stops compiling rather than failing
     * in flight. */
    if (EdsContainer_BSK_LINK_SensorFrame_DATADICTIONARY == EDS_PACKAGE_BSK_LINK_DATATYPEDB_RESERVED)
    {
        CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "compiled EDS schema has no SensorFrame; rebuild edslink/");
        BSK_LinkSensorTypeId = EDSLIB_ID_INVALID;
    }

    /* Boot handshake: resolve the FitManifest schema like every other
     * inbound frame. A defs build without the container is reported once and
     * simply leaves the link without a verdict - which the operator learns
     * from the missing FIT event, not from a silent pass. */
    BSK_LinkFitTypeId = EdsLib_DisplayDB_LookupTypeName(&ORRERY_DATABASE, BSK_LINK_FIT_TYPE_NAME);
    if (EdsLib_Is_DataType(BSK_LinkFitTypeId))
    {
        EdsLib_DataTypeDB_DerivedTypeInfo_t FitInfo;

        if (EdsLib_DataTypeDB_GetDerivedInfo(&ORRERY_DATABASE, BSK_LinkFitTypeId, &FitInfo) == EDSLIB_SUCCESS)
        {
            BSK_LinkFitFrameSize = FitInfo.MaxSize.Bytes;
            BSK_LinkFitBuffer    = EdsLib_Binding_AllocManagedBuffer(FitInfo.MaxSize.Bytes);
        }
        if (BSK_LinkFitBuffer == NULL)
        {
            BSK_LinkFitTypeId = EDSLIB_ID_INVALID;
            CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                              "no memory for a %u byte fit manifest; handshake disabled",
                              (unsigned int)BSK_LinkFitFrameSize);
        }
    }
    else
    {
        BSK_LinkFitTypeId = EDSLIB_ID_INVALID;
        CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                          "EDS database has no %s; fit handshake disabled", BSK_LINK_FIT_TYPE_NAME);
    }

    /* World frames: resolve every generated table row against the
     * linked schema, and disable - by name - any row the schema cannot
     * answer for. This is also the runtime registry check: the REAL size of
     * the container must equal the registry's declared size, or the table
     * is stale (re-run codegen/emit_frames.py) - a row that disagrees never
     * decodes, and the event bus says which and why. Never guess a schema. */
    for (unsigned RowIndex = 0; RowIndex < BSK_LINK_FRAME_ROWS; ++RowIndex)
    {
        const BskLink_WorldFrameDef_t     *Row       = &BskLink_WorldFrames[RowIndex];
        EdsLib_DataTypeDB_DerivedTypeInfo_t DerivInfo;

        BSK_LinkWorldTypeIds[RowIndex] = EdsLib_DisplayDB_LookupTypeName(&ORRERY_DATABASE, Row->Container);
        if (!EdsLib_Is_DataType(BSK_LinkWorldTypeIds[RowIndex]))
        {
            BSK_LinkWorldTypeIds[RowIndex] = EDSLIB_ID_INVALID;
            CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                              "EDS database has no %s; %s decode disabled", Row->Container, Row->Label);
            continue;
        }
        if (EdsLib_DataTypeDB_GetDerivedInfo(&ORRERY_DATABASE, BSK_LinkWorldTypeIds[RowIndex], &DerivInfo) !=
                EDSLIB_SUCCESS ||
            DerivInfo.MaxSize.Bytes != Row->Size)
        {
            BSK_LinkWorldTypeIds[RowIndex] = EDSLIB_ID_INVALID;
            CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                              "%s is %u bytes in schema but %u in the registry table - re-run emit_frames",
                              Row->Container, (unsigned int)DerivInfo.MaxSize.Bytes, (unsigned int)Row->Size);
            continue;
        }
        BSK_LinkWorldBuffers[RowIndex] = EdsLib_Binding_AllocManagedBuffer(DerivInfo.MaxSize.Bytes);
        if (BSK_LinkWorldBuffers[RowIndex] == NULL)
        {
            BSK_LinkWorldTypeIds[RowIndex] = EDSLIB_ID_INVALID;
            CFE_EVS_SendEvent(BSK_LINK_INIT_ERR_EID, CFE_EVS_EventType_ERROR,
                              "no memory for a %u byte %s frame; decode disabled",
                              (unsigned int)DerivInfo.MaxSize.Bytes, Row->Label);
        }
    }

    CFE_ES_WaitForStartupSync(true);

    /* Egress socket, created once: PUSH connects eagerly (ZeroMQ keeps
     * retrying on its own), and the HWM bounds the queue so a sim that never
     * comes up costs a bounded ring of commands, never unbounded memory. */
    if (BSK_LinkZmqContext == NULL)
    {
        BSK_LinkZmqContext = zmq_ctx_new();
    }
    if (BSK_LinkZmqContext == NULL)
    {
        CFE_EVS_SendEvent(BSK_LINK_ZMQ_ERR_EID, CFE_EVS_EventType_ERROR, "zmq_ctx_new failed");
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    BSK_LinkZmqPushSocket = zmq_socket(BSK_LinkZmqContext, ZMQ_PUSH);
    if (BSK_LinkZmqPushSocket == NULL)
    {
        CFE_EVS_SendEvent(BSK_LINK_ZMQ_ERR_EID, CFE_EVS_EventType_ERROR, "zmq PUSH socket failed: %s",
                          zmq_strerror(errno));
        return CFE_STATUS_EXTERNAL_RESOURCE_FAIL;
    }
    {
        int Hwm = BSK_LINK_EGRESS_HWM;
        zmq_setsockopt(BSK_LinkZmqPushSocket, ZMQ_SNDHWM, &Hwm, sizeof(Hwm));
    }
    zmq_connect(BSK_LinkZmqPushSocket, BSK_LinkCmdEndpoint);

    CFE_EVS_SendEvent(BSK_LINK_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "BSK_LINK initialized, basilisk feed %s -> tlm, %s <- commands, EDS decode %s",
                      BSK_LinkSensorEndpoint, BSK_LinkCmdEndpoint,
                      EdsLib_Is_Valid(BSK_LinkSensorTypeId) ? "ready" : "DISABLED");

    return CFE_SUCCESS;
}

/* -------------------------------------------------------------------------- */

static void BSK_Link_ZmqTryConnect(void)
{
    int    ReconnectDelayMs = 250; /* ms; ZMQ_RECONNECT_IVL value, not the option id */
    int    OptionRc;

    if (BSK_LinkZmqConnected)
    {
        return;
    }

    if (BSK_LinkZmqContext == NULL)
    {
        BSK_LinkZmqContext = zmq_ctx_new();
        if (BSK_LinkZmqContext == NULL)
        {
            CFE_EVS_SendEvent(BSK_LINK_ZMQ_ERR_EID, CFE_EVS_EventType_ERROR, "zmq_ctx_new failed");
            return;
        }
    }

    if (BSK_LinkZmqSocket == NULL)
    {
        BSK_LinkZmqSocket = zmq_socket(BSK_LinkZmqContext, ZMQ_SUB);
        if (BSK_LinkZmqSocket == NULL)
        {
            CFE_EVS_SendEvent(BSK_LINK_ZMQ_ERR_EID, CFE_EVS_EventType_ERROR, "zmq_socket failed: %s",
                              zmq_strerror(errno));
            return;
        }

        /* Subscribe to every topic and filter in code: the command echo and truth
         * channels are on other ports, but a future multi-topic sensor feed should
         * not need a config change here. */
        zmq_setsockopt(BSK_LinkZmqSocket, ZMQ_SUBSCRIBE, "", 0);
        /* ZeroMQ retries TCP connect on its own; cap the interval so a sim
         * that starts after cFS still gets picked up quickly. */
        zmq_setsockopt(BSK_LinkZmqSocket, ZMQ_RECONNECT_IVL, &ReconnectDelayMs, sizeof(ReconnectDelayMs));
    }

    OptionRc = zmq_connect(BSK_LinkZmqSocket, BSK_LinkSensorEndpoint);
    BSK_LinkAppData.ZmqConnectAttempts++;
    if (OptionRc == 0)
    {
        BSK_LinkZmqConnected = true;
        CFE_EVS_SendEvent(BSK_LINK_ZMQ_CONNECT_INF_EID, CFE_EVS_EventType_INFORMATION, "connected to basilisk feed %s",
                          BSK_LinkSensorEndpoint);
    }
}

/* -------------------------------------------------------------------------- */

static void BSK_Link_DrainZmq(void)
{
    if (!BSK_LinkZmqConnected)
    {
        BSK_Link_ZmqTryConnect();
        if (!BSK_LinkZmqConnected)
        {
            return;
        }
    }

    if (!EdsLib_Is_Valid(BSK_LinkSensorTypeId))
    {
        return;
    }

    /* Bound the drain: a scenario faster than the app must not starve the pipe. */
    for (uint32 Count = 0; Count < BSK_LINK_DRAIN_BUDGET; ++Count)
    {
        BSK_Link_DrainOneFrame();
    }
}

/* -------------------------------------------------------------------------- */

void BSK_LINK_AppMain(void)
{
    uint32          RunStatus = CFE_ES_RunStatus_APP_RUN;
    CFE_SB_Buffer_t *BufPtr;
    CFE_Status_t    Status;

    Status = BSK_Link_AppInit();
    if (Status == CFE_SUCCESS)
    {
        while (CFE_ES_RunLoop(&RunStatus))
        {
            /* Ground commands and fcs torque, both on one pipe. */
            Status = CFE_SB_ReceiveBuffer(&BufPtr, BSK_LinkAppData.CmdPipe, BSK_LINK_SB_TIMEOUT);
            if (Status == CFE_SUCCESS)
            {
                CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;
                CFE_SB_MsgId_Atom_t MsgIdValue;

                CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
                MsgIdValue = CFE_SB_MsgIdToValue(MsgId);
                if (MsgIdValue == (CFE_SB_MsgId_Atom_t)BSK_LINK_TORQUE_COMMAND_TLM_MID)
                {
                    BSK_Link_ForwardTorque(BufPtr);
                }
                else if (MsgIdValue == (CFE_SB_MsgId_Atom_t)BSK_LINK_STEP_ACK_TLM_MID)
                {
                    BSK_Link_ForwardStepAck(BufPtr);
                }
                else if (MsgIdValue == (CFE_SB_MsgId_Atom_t)BSK_LINK_CAM_CONFIG_TLM_MID)
                {
                    BSK_Link_ForwardCamConfig(BufPtr);
                }
                /* Any other MsgId (e.g. future ground commands) is accounted in
                 * MsgRecvCount only; there is no second consumer to surprise. */
            }

            BSK_Link_DrainZmq();
        }
    }

    CFE_ES_WriteToSysLog("BSK_LINK exit: recv=%lu frames=%lu bad=%lu other_topic=%lu torque=%lu/%lu/%lu acks=%lu/%lu/%lu cam=%lu/%lu/%lu store=%llu files/%llu B drops=%lu\n",
                         (unsigned long)BSK_LinkAppData.MsgRecvCount,
                         (unsigned long)BSK_LinkAppData.SensorMsgCount,
                         (unsigned long)BSK_LinkAppData.MalformedMsgCount,
                         (unsigned long)BSK_LinkAppData.TopicSkipCount,
                         (unsigned long)BSK_LinkAppData.TorqueRecvCount,
                         (unsigned long)BSK_LinkAppData.TorqueFwdCount,
                         (unsigned long)BSK_LinkAppData.TorqueDropCount,
                         (unsigned long)BSK_LinkAppData.AckRecvCount,
                         (unsigned long)BSK_LinkAppData.AckFwdCount,
                         (unsigned long)BSK_LinkAppData.AckDropCount,
                         (unsigned long)BSK_LinkAppData.CamRecvCount,
                         (unsigned long)BSK_LinkAppData.CamFwdCount,
                         (unsigned long)BSK_LinkAppData.CamDropCount,
                         (unsigned long long)BSK_LinkAppData.FilesStored,
                         (unsigned long long)BSK_LinkAppData.BytesStored,
                         (unsigned long)BSK_LinkAppData.ImgDropCount);

    CFE_ES_ExitApp(RunStatus);
}
