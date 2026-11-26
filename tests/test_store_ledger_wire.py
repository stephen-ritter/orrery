"""The store ledger is one wire with four owners: pin them to each other.

bsk_link writes ``CamStorage`` from a C struct (``orrery_bus_msg.h``), the
EDS computes the container from ``bsk_link.xml``, ``edslink/verify.py``
asserts ``struct.pack`` describes the same bytes, and COSMOS decodes with
defs generated from the database. The C assert fires at compile time and
the verify case at build time - both only when someone builds. This test is
the check that runs on a bare checkout: it fails when fewer than all four
moved together, which is exactly the drift that ships a telemetry packet
the ground decodes as noise.
"""

from __future__ import annotations

import importlib.util
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
XML = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds" / "bsk_link.xml"
HEADER = REPO / "cfs_mission" / "mission" / "orrery_defs" / "inc" / "orrery_bus_msg.h"
VERIFY = REPO / "edslink" / "verify.py"


def _verify_case() -> tuple[str, list[str]]:
    spec = importlib.util.spec_from_file_location("orrery_eds_verify", VERIFY)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # module level is data; EdsLib loads only in main()
    fmt, values = module.CASES["CamStorage"]
    return fmt, list(values)


def _xml_entries() -> list[str]:
    ns = "{http://www.ccsds.org/schema/sois/seds}"
    root = ET.parse(XML).getroot()
    for container in root.iter(f"{ns}ContainerDataType"):
        if container.get("name") == "CamStorage":
            entries = container.find(f"{ns}EntryList")
            assert entries is not None, "CamStorage lost its EntryList"
            return [e.get("name") or "" for e in entries.findall(f"{ns}Entry")]
    raise AssertionError("CamStorage container vanished from bsk_link.xml")


def test_the_c_twin_is_sized_like_the_packed_wire() -> None:
    """The compile-time assert and struct.pack agree without compiling."""
    fmt, _ = _verify_case()
    pinned = re.search(r"sizeof\(BskCamStorage_Payload_t\) == (\d+)", HEADER.read_text())
    assert pinned is not None, "the CamStorage size pin vanished from the mirror header"
    assert int(pinned.group(1)) == struct.calcsize(fmt), (
        "the C mirror and verify.py's struct format disagree: one of the four "
        "owners of this wire moved alone"
    )


def test_the_eds_lists_what_the_ground_packs() -> None:
    """Same fields, same order, in the XML and in the byte-level case."""
    _, fields = _verify_case()
    assert _xml_entries() == fields, (
        "bsk_link.xml and verify.py describe different field lists; the defs "
        "are generated from the XML and decoded with the bytes - they must be "
        "the same list"
    )
