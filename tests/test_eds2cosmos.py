"""Golden tests: eds2cosmos against the pinned cFS v7.0.1 checkout."""

from pathlib import Path

import pytest

from codegen.eds2cosmos import (
    build_eds,
    emit_c_cmd_table,
    emit_c_table,
    emit_cosmos,
    mission_packets,
)
from codegen.eds_model import Eds, Leaf, Packet

REPO = Path(__file__).resolve().parents[1]
CFS = REPO / "cfs_mission/cFS"
MISSION_DEFS = REPO / "cfs_mission/mission/orrery_defs/eds"

pytestmark = pytest.mark.skipif(
    not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"
)


@pytest.fixture(scope="module")
def eds() -> Eds:
    """The loaded EDS model of the pinned checkout."""
    return build_eds(CFS, MISSION_DEFS)


@pytest.fixture(scope="module")
def packets(eds: Eds) -> list[Packet]:
    """All generated packet definitions."""
    return eds.packets()


def _pkt(packets: list[Packet], name: str) -> Packet:
    return next(p for p in packets if p.name == name)


def test_packet_coverage_threshold(packets: list[Packet]) -> None:
    """Every app/core package contributes packets (floor guards regressions).

    Floors re-anchored at the 2026-11-27 prebaked purge: thirteen silent
    modules left the model (31 tlm / 32 cmd remain), and a floor is a floor
    for what the mission actually ships - not for the pile it retired.
    """
    tlm = [p for p in packets if p.direction == "tlm"]
    cmd = [p for p in packets if p.direction == "cmd"]
    assert len(tlm) >= 30
    assert len(cmd) >= 30


def test_no_skip_warnings(eds: Eds) -> None:
    """No app package failed to resolve (regression guard)."""
    assert eds.warnings == []


def test_msgid_mapping_matches_mission_defs(packets: list[Packet]) -> None:
    """Topic->msgid mirrors CFE_MissionLib_TryMapping (V1 header packing).

    The 2026-11-27 purge moved these witnesses from HK/TO_LAB to the
    mission's own packets: the derivation law did not change, only whose
    numbers prove it.
    """
    assert _pkt(packets, "OBC_HK_TLM").msgid == 0x08CD  # 0x0800 | 0x0CD (TM topic)
    assert _pkt(packets, "FCS_CMD").msgid == 0x18C8  # 0x1800 | 0x0C8 (TC topic)
    assert _pkt(packets, "FCS_HK_TLM").msgid == 0x08CA  # 0x0800 | 0x0CA (TM topic)


def test_function_code_states(packets: list[Packet]) -> None:
    """CAM_CMD carries the EDS-constrained function-code dispatch table.

    (This law was witnessed on HK_CMD until the purge retired it; FC 1
    stays unclaimed there for its own reason - see the retirement test.)
    """
    cam = _pkt(packets, "CAM_CMD")
    assert cam.function_codes == {"NoopCmd": 0, "SnapCmd": 2}
    cmd_text, _ = emit_cosmos(packets)
    assert 'STATE "SnapCmd" 2' in cmd_text


def test_header_layout_matches_c_structs(packets: list[Packet]) -> None:
    """CCSDS length word + sec header place payloads where the C structs do.

    Regression guard for the flattened model skipping the EDS <LengthEntry>:
    primary(6) + telemetry sec hdr(6) + alignment pad(4) -> payload at 16.
    (Witnessed on HK until the 2026-11-27 purge; the OBC's housekeeping
    carries the same flattened-header shape.)
    """
    assert _pkt(packets, "OBC_HK_TLM").byte_offset(".CommandCounter") == 16
    assert _pkt(packets, "CFE_EVS_LONG_EVENT_MSG").byte_offset(".AppName") == 16
    # command: primary(6) + FunctionCode + Checksum
    fcs_cmd = _pkt(packets, "FCS_CMD")
    assert fcs_cmd.byte_offset(".FunctionCode") == 6
    assert fcs_cmd.byte_offset(".Checksum") == 7
    assert fcs_cmd.total_bytes() == 8  # bare NOOP


def test_obc_hk_tlm_layout(packets: list[Packet]) -> None:
    """OBC_HK_TLM payload items reflect the EDS entry list."""
    names = [i.name.rsplit(".", 1)[-1] for i in _pkt(packets, "OBC_HK_TLM").items]
    assert "CommandCounter" in names
    assert "FcsMode" in names
    tlm_text = emit_cosmos(packets)[1]
    assert 'APPEND_ITEM COMMAND_COUNTER 8 UINT "auto-generated from EDS" LITTLE_ENDIAN' in tlm_text
    assert "TELEMETRY SAT_CHIEF OBC_HK_TLM" in tlm_text
    assert f"APPEND_ID_ITEM PKTID 16 UINT {0x08CD}" in tlm_text  # OBC_HK_TLM id


def test_command_def_syntax_v6(packets: list[Packet]) -> None:
    """Commands use PARAMETER keywords and id-valued PKTID (CE v6 parser)."""
    cmd_text = emit_cosmos(packets)[0]
    assert "APPEND_PARAMETER" in cmd_text
    assert "APPEND_ITEM" not in cmd_text
    assert f"APPEND_ID_PARAMETER PKTID 16 UINT 0 65535 {0x18C8}" in cmd_text  # FCS_CMD
    assert "APPEND_ID_PARAMETER FUNCCODE 8 UINT 0 255 0" in cmd_text


def test_wire_endianness_matches_native_target(packets: list[Packet]) -> None:
    """Headers are network-order; payload is target-native (x86 = little).

    Regression guard for the first wire capture: an HK NOOP event decoded as
    big-endian EVS EventID read 1024 instead of 4. The HK packets retired
    with the 2026-11-27 prebaked purge; OBC_HK_TLM carries the same mixed
    shape (network-order time header, native-order payload).
    """
    obc = _pkt(packets, "OBC_HK_TLM")
    data = bytearray(28)
    data[6:10] = (1001023).to_bytes(4, "big")  # time header: big-endian
    data[16] = 7  # CommandCounter (uint8, LE)
    assert obc.wire_get(bytes(data), ".Seconds") == 1001023
    assert obc.wire_get(bytes(data), ".CommandCounter") == 7
    evs = _pkt(packets, "CFE_EVS_LONG_EVENT_MSG")
    data = bytearray(64)
    data[36:38] = (4).to_bytes(2, "little")  # EventID: native little
    assert evs.wire_get(bytes(data), ".EventID") == 4


def test_model_drift_exclusion(packets: list[Packet]) -> None:
    """CF_HK_TLM is modeled by the EDS but excluded from every artifact.

    The CF EDS XML does not describe the runtime (non-EDS) CF_HkPacket:
    94 B modeled vs 184 B on the wire. Until the XML matches (upstream), the
    packet must appear in NO beacon table and NO ground def - a wrong def
    misidentifies live traffic as CF_HK_TLM and COSMOS logs it as an error.
    """
    from codegen.eds2cosmos import MODEL_DRIFT_EXCLUSIONS, mission_packets

    assert "CF_HK_TLM" in {p.name for p in packets}  # the model still carries it
    kept = mission_packets(packets)
    assert "CF_HK_TLM" not in {p.name for p in kept}
    assert "CF_HK_TLM" not in emit_c_table(kept, "088b2fa")
    cmd_text, tlm_text = emit_cosmos(kept)
    assert "CF_HK_TLM" not in tlm_text and "CF_HK_TLM" not in cmd_text
    assert MODEL_DRIFT_EXCLUSIONS  # list exists; wire_audit keeps it honest


def test_cf_eot_survives_the_vendor_typo(packets: list[Packet]) -> None:
    """CF_EOT_TLM reaches the beacon tables even though NASA's cf.xml is broken.

    Its EotTlmTopicId closes the ${...} one byte early (``...TOPICID)}``),
    which used to drop CF's only per-file telemetry out of the model in
    silence. Eds.VENDOR_REPAIRS patches that in memory - the core file is
    untouched, and the repair reports itself when upstream fixes it. The
    field order and sizes here are the wire law measured off the beacon on
    2026-11-25: 180 B total, sixteen-byte telemetry header, then nine
    target-native uint32s and two 64-byte names.
    """
    eot = _pkt(packets, "CF_EOT_TLM")
    assert eot.msgid == 0x08B3  # 0x0800 | (179-0): TM topic 179, cpu instance 1
    assert eot.total_bytes() == 180
    names = [i.name.rsplit(".", 1)[-1] for i in eot.items]
    for field in ("seq_num", "direction", "txn_stat", "peer_eid", "src_filename", "dst_filename"):
        assert field in names, f"measured EOT field missing from the def: {field}"
    assert names.index("direction") < names.index("txn_stat") < names.index("src_filename")
    from codegen.eds2cosmos import mission_packets

    assert "CF_EOT_TLM" in emit_c_table(mission_packets(packets), "088b2fa")


def test_cfdp_down_payload_is_a_real_packet(packets: list[Packet]) -> None:
    """The file request arrives on the wire exactly as declared.

    144 B: two 64-character path strings and three pad bytes, no more. The
    fields were once declared ``BASE_TYPES/uint8`` with an ``arraySize`` the
    model silently ignored, and both the COSMOS def and the SPS router table
    shrank to the resulting 8-byte lie - ground requests flew only because
    the SPS length check was generous.
    """
    pkt = _pkt(packets, "FCS_CFDP_DOWN")
    assert pkt.total_bytes() == 144
    names = [i.name.rsplit(".", 1)[-1] for i in pkt.items]
    assert all(pad in names for pad in ("Pad_0", "Pad_1", "Pad_2"))
    for field in ("SrcPath", "DstName"):
        leaf = next(i for i in pkt.items if i.name.endswith(f".{field}"))
        assert isinstance(leaf, Leaf), f"{field} vanished behind a Padding"
        assert leaf.bits == 512 and leaf.cbase == "string", f"{field} is not a 64-char string"


def test_strings_reach_the_ground_as_text(packets: list[Packet]) -> None:
    """EDS-declared strings emit as STRING, not BLOCK.

    The station must be able to type a path into a command and read a
    filename out of telemetry.
    """
    cmd_text, tlm_text = emit_cosmos(packets)
    assert 'APPEND_PARAMETER SRC_PATH 512 STRING "auto-generated from EDS"' in cmd_text
    assert 'APPEND_PARAMETER DST_NAME 512 STRING "auto-generated from EDS"' in cmd_text
    assert "APPEND_ITEM SRC_FILENAME 512 STRING" in tlm_text
    assert "SRC_FILENAME 512 BLOCK" not in tlm_text


def test_sps_router_knows_the_request_length(packets: list[Packet]) -> None:
    """The onboard SPS forward table carries the same 136-byte payload law."""
    table = emit_c_cmd_table(packets, "088b2fa")
    assert '{0x18D6, 0, 136, "FCS_CFDP_DOWN"}' in table


def test_the_retired_inventory_command_stays_retired(packets: list[Packet]) -> None:
    """FC 1 is gone from every surface: EDS, ground defs, and the SPS table.

    The store publishes /cf/manifest.txt itself - boot census and every
    mutation - so the command that once opened the ledger must not creep
    back through an edited golden or a revived EDS block: a ground that can
    send what the craft refuses is a trap, and an SPS row scheduling that
    refusal is the same trap with a timer. NOOP and SNAP keep their numbers
    (FC 1 stays unclaimed); SNAP's identity is scripts and muscle memory.
    """
    pkt = _pkt(packets, "CAM_CMD")
    assert pkt.msgid == 0x18D2
    cmd_text, _ = emit_cosmos(packets)
    assert "ManifestCmd" not in cmd_text
    assert 'STATE "NoopCmd" 0' in cmd_text
    assert 'STATE "SnapCmd" 2' in cmd_text
    table = emit_c_cmd_table(packets, "088b2fa")
    assert "ManifestCmd" not in table
    assert "{0x18D2, 1, 0," not in table
    assert "{0x18D2, 2, 0," in table


def test_vendor_repair_reports_a_fixed_upstream(tmp_path: Path) -> None:
    """A repair that finds nothing to repair says so, so it cannot fossilize."""
    xml = tmp_path / "apps/cf/eds/cf.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text(
        '<?xml version="1.0"?><PackageFile xmlns="http://www.ccsds.org/schema/sois/seds"/>'
    )
    eds = Eds()
    eds.load_xml(xml)
    assert any("no longer present" in w for w in eds.warnings)


def test_c_table_telemetry_only(packets: list[Packet]) -> None:
    """gs_tcp header lists every telemetry MsgId, sorted, with the right count.

    Commands (e.g. 0x18C8 FCS_CMD) must NOT be beacon subscriptions: they
    are consumed by their apps, and scheduled command traffic (sch_lab
    wake-ups) would flood the ground with UNKNOWN-packet rejects.
    """
    text = emit_c_table(packets, "088b2fa")
    n_tlm = sum(1 for p in packets if p.direction == "tlm")
    assert 0 < n_tlm < len(packets)  # the model really carries commands too
    assert f"#define ORRERY_MID_COUNT {n_tlm}" in text
    assert '{0x08CA, "FCS_HK_TLM"}' in text
    assert "FCS_CMD" not in text
    assert not any(line.lstrip().startswith("{0x18") for line in text.splitlines())


def test_cmd_table_is_the_sps_dictionary(packets: list[Packet]) -> None:
    """The schedule dictionary: one row per schedulable (MsgId, Fc).

    sps validates schedules against the EDS, so the table covers exactly
    the commands the ground can send, with sizes from the same model that
    emits the COSMOS cmd defs.

    Scheduled execution rests on this table being the truth: an unknown schedule
    is REFUSED at the uplink (EVS + HK counter), and an accepted one must
    fire as a message its target app accepts at dispatch - which is only
    guaranteed if the stored payload length equals the wire size the EDS
    defines for that (MsgId, Fc).
    """
    text = emit_c_cmd_table(packets, "088b2fa")
    # FC-dispatch packets contribute one row per declared subtype...
    assert '{0x18C8, 1, 0, "FCS_CMD.EnableCmd"}' in text
    assert '{0x18C8, 2, 0, "FCS_CMD.DisableCmd"}' in text
    assert '{0x18C8, 3, 0, "FCS_CMD.BurnStopCmd"}' in text
    # ...payload-bearing packets one row at FC 0 with their real payload
    # size (24 B: what fcs's own _Static_assert pins for FcsSetGain).
    assert '{0x18CA, 0, 24, "FCS_SET_GAIN"}' in text
    # the Rung 1 burn ask: 40 B payload (force, duration, 3-axis), pinned the
    # same way - what fcs will check its length against.
    assert '{0x18D7, 0, 40, "FCS_BURN"}' in text
    # the scheduler's own port is in the dictionary (the SCHEDULE row exists -
    # flight refuses SELF-schedules at runtime, the dictionary stays complete)
    assert '{0x18CF, 0, 60, "SPS_SCHEDULE"}' in text
    assert '{0x18D0, 0, 0, "SPS_SEND_HK"}' in text
    n_cmd = sum(1 for p in packets if p.direction == "cmd")
    n_rows = len([line for line in text.splitlines() if line.lstrip().startswith("{0x")])
    assert n_rows >= n_cmd  # every packet contributes at least one row
    # telemetry never enters a command dictionary
    assert "FCS_HK_TLM" not in text
    assert not any(line.lstrip().startswith("{0x08") for line in text.splitlines())


def test_the_prebaked_apps_stay_purged(packets: list[Packet]) -> None:
    """2026-11-27 purge tripwire: thirteen silent modules, absent everywhere.

    hk hs lc mm sc md ds cs fm ci_lab to_lab sample_app sample_lib left
    the registry, the build, and the EDS source set (registry/apps.toml
    tells the audit story). Their packets must not creep back through a
    regenerated golden or a re-added bundle eds/ directory: not in the
    model, not in the defs, not in either C table. MIDs and event IDs
    leave unclaimed - this is what keeps them unclaimed.
    """
    retired = ("HK_", "HS_", "LC_", "MM_", "MD_", "SC_", "DS_", "CS_", "FM_", "CI_LAB_")
    assert not [p.name for p in packets if p.name.startswith(retired)]
    assert not [p.name for p in packets if p.name.startswith(("TO_LAB_", "SAMPLE_"))]
    cmd_text, tlm_text = emit_cosmos(mission_packets(packets))
    table = emit_c_cmd_table(packets, "088b2fa")
    for name in (
        "HK_HK_TLM",
        "HK_CMD",
        "HS_HK_TLM",
        "LC_LC_TLM",
        "MM_MM_TLM",
        "MD_DWELL_TLM",
        "SC_SC_TLM",
        "DS_FILE_DIR_TLM",
        "CS_CS_TLM",
        "FM_DIR_LIST_TLM",
        "CI_LAB_HK_TLM",
        "TO_LAB_HK_TLM",
        "SAMPLE_APP_TLM",
    ):
        assert name not in cmd_text, name
        assert name not in tlm_text, name
        assert name not in table, name


def test_schedule_envelope_wire(packets: list[Packet]) -> None:
    """Wire contract for SPS_SCHEDULE: 68 B total.

    8-byte command header + the 60 B envelope (T, MsgId, Fc, length, 48 B
    verbatim payload), pinned against the C struct's _Static_assert (60 B)
    in orrery_bus_msg.h by arithmetic here.
    """
    sched = _pkt(packets, "SPS_SCHEDULE")
    assert sched.total_bytes() == 68
    assert sched.byte_offset(".ExecTimeS") == 8
    assert sched.byte_offset(".CmdMsgId") == 16
    assert sched.byte_offset(".CmdFc") == 18
    assert sched.byte_offset(".CmdDataLen") == 19
    assert sched.byte_offset(".CmdData_0") == 20


def test_custom_app_packets(packets: list[Packet]) -> None:
    """The custom apps (bsk_link frames, fcs) carry the pinned MsgIds.

    These numbers are simultaneously: EDS XML topics (ground defs here),
    orrery_defs/inc/orrery_topicids.h (flight C, test below), and the
    gs_tcp beacon table. Three consumers, one generation step - if any two
    disagree, this test or the mission build fails.
    """
    assert _pkt(packets, "BSK_LINK_SENSOR_FRAME_TLM").msgid == 0x08C8
    assert _pkt(packets, "BSK_LINK_TORQUE_COMMAND_TLM").msgid == 0x08C9
    assert _pkt(packets, "FCS_HK_TLM").msgid == 0x08CA
    assert _pkt(packets, "BSK_LINK_STEP_ACK_TLM").msgid == 0x08CB
    assert _pkt(packets, "FCS_CMD").msgid == 0x18C8
    assert _pkt(packets, "FCS_SEND_HK").msgid == 0x18C9
    assert _pkt(packets, "FCS_SET_GAIN").msgid == 0x18CA
    assert _pkt(packets, "FCS_CMD").function_codes == {
        "NoopCmd": 0,
        "EnableCmd": 1,
        "DisableCmd": 2,
        "BurnStopCmd": 3,
    }


def test_truth_never_a_bus_packet(packets: list[Packet]) -> None:
    """TruthState has no TopicId ON PURPOSE: truth must not exist inside cFS.

    If someone assigns it one, the satellite would start "knowing" simulator
    truth and the two-target separation dies. This assertion is the tripwire.
    """
    assert not any("TRUTH" in p.name for p in packets)


def test_set_gain_carries_its_payload(packets: list[Packet]) -> None:
    """FCS_SET_GAIN is its own interface because FC-subtype payloads are lost.

    The generator models one layout per interface: argument-bearing commands
    hidden behind function-code subtypes (CF TxFileCmd, the upstream pattern)
    silently drop their fields. fcs.xml therefore gives gain upload a MsgId,
    and this pins the fields actually made it onto the ground definition.
    """
    gain = _pkt(packets, "FCS_SET_GAIN")
    names = [item.name for item in gain.items]
    assert any(n.endswith(".Kp_0") for n in names)
    assert any(n.endswith(".Kp_1") for n in names)
    assert any(n.endswith(".Kp_2") for n in names)
    assert gain.total_bytes() == 32  # 8 B command header + 24 B of doubles
    cmd_text, _ = emit_cosmos(packets)
    assert "APPEND_PARAMETER KP_0 64 FLOAT" in cmd_text


def test_wheel_limit_command_is_a_real_payload_packet(packets: list[Packet]) -> None:
    """Threshold upload: same own-packet pattern as SET_GAIN, one f64."""
    limit = _pkt(packets, "FCS_SET_WHEEL_LIMIT")
    assert any(item.name.endswith(".WheelLimit") for item in limit.items)
    assert limit.total_bytes() == 16  # 8 B command header + 8 B threshold
    cmd_text, _ = emit_cosmos(packets)
    assert "APPEND_PARAMETER WHEEL_LIMIT 64 FLOAT" in cmd_text


def test_bus_struct_header_matches_model(packets: list[Packet]) -> None:
    """orrery_bus_msg.h pins C payload sizes; the model must agree exactly.

    The header's _Static_assert guards the build; this guards the OTHER
    direction - a schema edit that keeps C compiling but changes the wire.
    """
    header = (REPO / "cfs_mission/mission/orrery_defs/inc/orrery_bus_msg.h").read_text()
    pinned = dict(
        (match[1], int(match[0]))
        for match in [
            (s.split("==")[1].split(",")[0].strip(), s.split("sizeof(")[1].split(")")[0])
            for s in header.splitlines()
            if "_Static_assert(sizeof(" in s
        ]
    )
    payload_bytes = {
        "BSK_LINK_SENSOR_FRAME_TLM": pinned["BskSensorFrame_Payload_t"],
        "BSK_LINK_ONBOARD_STATE_TLM": pinned["BskOnboardState_Payload_t"],
        "BSK_LINK_TORQUE_COMMAND_TLM": pinned["BskTorqueCmd_Payload_t"],
        "BSK_LINK_STEP_ACK_TLM": pinned["BskStepAck_Payload_t"],
        "BSK_LINK_BURN_COMMAND_TLM": pinned["BskBurnCmd_Payload_t"],
        "FCS_HK_TLM": pinned["FcsHkTlm_Payload_t"],
        "OBC_HK_TLM": pinned["ObcHk_Payload_t"],
    }
    for name, payload in payload_bytes.items():
        total = _pkt(packets, name).total_bytes()
        # 16 B, not 14: the cFS telemetry header is CCSDS primary (6) + cFS
        # time header with its alignment tail, modelled and on the wire
        # identically (wire audit). CFE_MSG_TelemetryHeader_t agrees, which
        # is exactly why both sides pin these numbers.
        assert total == payload + 16, (
            f"{name}: defs {total} != C header {payload} + 16 B tlm header"
        )


def test_topicids_header_mirrors_xml() -> None:
    """orrery_topicids.h == the topic <Define> values in cfe-topicids.xml."""
    import re

    xml = (MISSION_DEFS / "cfe-topicids.xml").read_text()
    xml_topics = {
        f"{pkg}_{topic}": int(value)
        for pkg, topic, value in re.findall(
            r'Define name="(BSK_LINK|FCS|OBC|SPS|CAM)_(\w+)_TOPICID"'
            r'\s+value="\$\{CFE_MISSION/\w+\} \+ (\d+)"',
            xml,
        )
    }
    header = (REPO / "cfs_mission/mission/orrery_defs/inc/orrery_topicids.h").read_text()
    # registered part topics moved into the generated include: the C
    # preprocessor sees both files as one mirror, so the test does too
    header += (REPO / "cfs_mission/mission/orrery_defs/inc/orrery_part_topicids.h").read_text()
    c_topics = {
        name.removeprefix("CFE_MISSION_").removesuffix("_TOPICID"): int(value)
        for name, value in re.findall(r"#define (CFE_MISSION_\w+_TOPICID) (\d+)", header)
    }
    assert c_topics  # the header really declares the custom topics
    for name, value in xml_topics.items():
        assert c_topics.get(name) == value, (
            f"topic {name}: XML {value} != C header {c_topics.get(name)}"
        )
    assert len(xml_topics) == len(c_topics) == 29  # 20 apps + 5 parts + ledger + CFDP + burn pair


def test_time_cmd_mid_derives_from_topicids() -> None:
    """Time sync rides the TIME service command port: MID == base + topic 5.

    bsk_link slaves mission MET to the link timecode with a SET_MET over
    ORRERY_TIME_CMD_MID; there is no EDS/defs packet for it (a cFE service
    command), so the wire fact to pin is the derivation itself: the header
    composes the MID as the command base plus the TIME_CMD topic, the same
    construction as every custom-app MID.
    """
    import re

    topicids = (REPO / "cfs_mission/mission/orrery_defs/inc/orrery_topicids.h").read_text()
    msgids = (REPO / "cfs_mission/mission/orrery_defs/inc/orrery_msgids.h").read_text()
    assert re.search(r"#define ORRERY_TIME_CMD_TOPICID 5\b", topicids), (
        "TIME_CMD topic must be the cFE service's block-relative id (5)"
    )
    mid_line = "ORRERY_TIME_CMD_MID (SAMPLE_CPU1_CMD_MID_BASE + ORRERY_TIME_CMD_TOPICID)"
    assert mid_line in msgids


def test_beacon_table_streams_the_control_loop(packets: list[Packet]) -> None:
    """The beacon carries exactly what the satellite senses and commands."""
    text = emit_c_table(packets, "088b2fa")
    assert '{0x08C8, "BSK_LINK_SENSOR_FRAME_TLM"}' in text
    assert '{0x08C9, "BSK_LINK_TORQUE_COMMAND_TLM"}' in text
    assert '{0x08CA, "FCS_HK_TLM"}' in text
    assert "FCS_CMD" not in text and "FCS_SET_GAIN" not in text


def test_step_ack_never_reaches_ground(packets: list[Packet]) -> None:
    """The lockstep barrier ack is bus-internal by exclusion, twice over.

    ``BSK_LINK_STEP_ACK_TLM`` (0x08CB) must exist as a real bus packet - fcs
    publishes it every processed frame - and must be absent from BOTH the
    ground defs and the beacon subscription. The absence from the defs is the
    tripwire: a leak onto the beacon then lands as an UNKNOWN packet in the
    COSMOS log instead of decoding quietly. A 1 Hz barrier chatter stream on
    the ground is a bug, not a feature.
    """
    from codegen.eds2cosmos import BEACON_EXCLUSIONS, mission_packets

    ack = _pkt(packets, "BSK_LINK_STEP_ACK_TLM")  # on the bus: real and pinned
    assert ack.msgid == 0x08CB
    assert "BSK_LINK_STEP_ACK_TLM" in BEACON_EXCLUSIONS
    kept = mission_packets(packets)
    assert not any(p.name == "BSK_LINK_STEP_ACK_TLM" for p in kept)
    _, tlm_text = emit_cosmos(kept)
    assert "STEP_ACK" not in tlm_text
    assert "STEP_ACK" not in emit_c_table(kept, "088b2fa")


def test_target_is_an_input_not_a_fork(eds: Eds) -> None:
    """--target renames the COSMOS target column and nothing else.

    A renamed vehicle's defs must be provably just the original with the target
    column swapped: heterogeneity through parameters, not through code paths. The
    comparison is byte-exact, which it could not be while the header carried the
    generation second — two calls straddling a boundary differed in that one line,
    and the test reported a leaked target name that was really a clock.
    """
    from codegen.eds2cosmos import mission_packets

    packets = mission_packets(eds.packets())
    default = emit_cosmos(packets)
    explicit = emit_cosmos(packets, target="SAT_CHIEF")
    renamed = emit_cosmos(packets, target="SAT_CHIEF2")

    for text in (*default, *explicit, *renamed):
        assert text.splitlines()[0].startswith(
            "# AUTO-GENERATED by codegen/eds2cosmos from cFS EDS content "
        ), "a generated def must name the content it was built from"

    assert default == explicit, "the default target must be SAT_CHIEF byte-identical"
    for original, swapped in zip(default, renamed, strict=True):
        expected = [line.replace("SAT_CHIEF ", "SAT_CHIEF2 ") for line in original.splitlines()]
        assert swapped.splitlines() == expected


def test_truth_defs_generated_from_the_bare_container(eds: Eds) -> None:
    """Truth defs come from the bare container, not a cFS packet.

    TRUTH_STATE_TLM is generated from BSK_LINK/TruthState itself - the
    headerless container that (by the tripwire above) can never appear in
    flight defs gets its own ground-framed target, tlm-only.
    """
    from codegen.eds2cosmos import _cosmos_name, emit_cosmos_truth

    items = eds.leaf("BSK_LINK/TruthState", "BSK_LINK")
    assert len(items) == 18, "TruthState is 18 float64s (144 B: 14 orbit + burn ledger)"
    text = emit_cosmos_truth(items, target="TRUTH_CHIEF")
    assert "APPEND_ID_ITEM PACKET_ID 8 UINT 1" in text
    assert "APPEND_ITEM PACKET_LENGTH 16 UINT" in text
    assert text.count("APPEND_ITEM") == 2 + 18  # reserved + length + payload
    for item in items:
        assert f"APPEND_ITEM {_cosmos_name(item.name)}" in text
    assert "COMMAND" not in text  # absence-of-cmd-def is the tripwire
