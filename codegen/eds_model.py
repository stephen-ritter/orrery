"""Parse cFS EDS (CCSDS SOIS SEDS) XML into a flat model of packets.

The mission's EDS XML trees (cFE modules, apps, mission defines) are the
single source of truth for message layouts (D3); this module resolves the
``${NS/NAME}`` constants, follows ``baseType`` chains, and flattens
container types into bit-level fields ready for code emission.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from codegen.eds_sources import core_xml_files

SEDS_NS = "{http://www.ccsds.org/schema/sois/seds}"

_REF_RE = re.compile(r"\$\{([^}]+)\}")
_SAFE_EXPR = re.compile(r"^[\d\s+\-*/().xA-Fa-f]+$")
_PADDED_DECIMAL = re.compile(r"\b0+(\d)")


def _eval_int(expr: str) -> int:
    """Evaluate a restricted integer expression (hex + padded decimals)."""
    normalized = _PADDED_DECIMAL.sub(r"\1", expr)
    if not _SAFE_EXPR.match(normalized):
        raise ValueError(f"non-numeric EDS expression: {expr!r}")
    return int(eval(normalized, {"__builtins__": {}}, {}))


def _local(tag: str) -> str:
    """Strip the SEDS namespace from an element tag."""
    return tag.rsplit("}", 1)[-1]


def _findall_local(elem: ET.Element, path: str) -> list[ET.Element]:
    """ElementTree findall that ignores the SEDS namespace on every step."""
    parts = [f"{SEDS_NS}{step}" for step in path.split("/")]
    return elem.findall("/".join(parts), {})


def _find_local(elem: ET.Element, path: str) -> ET.Element | None:
    """find() with namespace-agnostic steps."""
    found = _findall_local(elem, path)
    return found[0] if found else None


def _find_any(elem: ET.Element, *paths: str) -> ET.Element | None:
    """First element matching any of the given paths."""
    for path in paths:
        hit = _find_local(elem, path)
        if hit is not None:
            return hit
    return None


class Constants:
    """Mission EDS constants (``<Define>``) with ``${NS/NAME}`` resolution."""

    def __init__(self) -> None:
        """Start with an empty constant set."""
        self._expr: dict[str, str] = {}
        self._text: dict[str, str] = {}
        self._cache: dict[str, int] = {}

    def load_mission_file(self, path: Path) -> None:
        """All ``<Define>``s in a file; package-scoped where enclosed."""
        self._scan_defines(ET.parse(path).getroot())

    def _scan_defines(self, root: ET.Element) -> None:
        """Scan every ``<Define>`` in a file tree.

        Mission files root at ``DesignParameters`` (CFE_MISSION namespace);
        package files may carry their own ``<DefineSet>`` scopes.
        """
        parent = {child: elem for elem in root.iter() for child in elem}
        for elem in root.iter():
            if _local(elem.tag) != "Define":
                continue
            name, value = elem.get("name"), elem.get("value")
            if not (name and value is not None):
                continue
            ns = "CFE_MISSION"
            node: ET.Element | None = elem
            while node is not None:
                if _local(node.tag) == "Package":
                    ns = node.get("name", "CFE_MISSION")
                    break
                node = parent.get(node)
            self._expr[f"{ns}/{name}"] = value

    def value_text(self, name: str, _seen: frozenset[str] = frozenset()) -> str:
        """Fully ``${...}``-expanded text of a constant (may be non-numeric)."""
        if name in _seen:
            raise ValueError(f"cyclic EDS constant: {name}")
        try:
            expr = self._expr[name]
        except KeyError as exc:
            raise KeyError(f"undefined EDS constant {name}") from exc
        if name in self._text:
            return self._text[name]
        expanded = _REF_RE.sub(lambda m: self.value_text(m.group(1), _seen | {name}), expr)
        self._text[name] = expanded
        return expanded

    def resolve(self, name: str) -> int:
        """Numeric value of a constant."""
        if name not in self._cache:
            self._cache[name] = _eval_int(self.value_text(name))
        return self._cache[name]

    def expand(self, text: str) -> str:
        """Substitute ``${...}`` references with their expanded text."""
        return _REF_RE.sub(lambda m: self.value_text(m.group(1)), text)

    def expand_to_int(self, text: str) -> int:
        """Evaluate a possibly-``${...}`` expression string to int."""
        return _eval_int(self.expand(text.strip()))


@dataclass
class Leaf:
    """A resolved scalar field: integer/float/bool/binary with enum values."""

    name: str
    bits: int
    cbase: str  # uint | int | float | string | binary
    states: dict[int, str] = field(default_factory=dict)
    array_count: int = 1
    big_endian: bool = False  # wire order; False = target-native (x86 = LE)


@dataclass
class Padding:
    """Alignment padding between fields."""

    name: str
    bits: int
    array_count: int = 1


Item = Leaf | Padding


@dataclass
class Packet:
    """One SB packet (interface) with resolved layout and numeric MsgId."""

    target_app: str
    name: str
    msgid: int
    direction: str  # tlm | cmd
    description: str
    items: list[Item]
    function_codes: dict[str, int]  # command name -> FC, for FC dispatch

    def byte_offset(self, leaf_suffix: str) -> int:
        """Byte offset of the first item whose name ends with ``leaf_suffix``."""
        bits = 0
        for item in self.items:
            if item.name.endswith(leaf_suffix):
                if bits % 8 != 0:
                    raise ValueError(f"{self.name}: {leaf_suffix} not byte aligned")
                return bits // 8
            bits += item.bits
        raise KeyError(f"{self.name}: no item ending with {leaf_suffix}")

    def total_bytes(self) -> int:
        """Size of the packet per the EDS layout."""
        bits = sum(i.bits for i in self.items)
        if bits % 8 != 0:
            raise ValueError(f"{self.name}: size not byte aligned")
        return bits // 8

    def wire_get(self, data: bytes, leaf_suffix: str) -> int:
        """Decode an integer item from a captured packet, per model layout.

        Width and wire order come from the EDS (headers big-endian, payload
        target-native), so callers never hardcode offsets or byte order.
        """
        off = self.byte_offset(leaf_suffix)
        leaf = next(i for i in self.items if i.name.endswith(leaf_suffix))
        if not isinstance(leaf, Leaf):
            raise ValueError(f"{self.name}: {leaf_suffix} is not a scalar leaf")
        width = leaf.bits // 8
        if width == 0 or off + width > len(data):
            raise ValueError(f"{self.name}: {leaf_suffix} outside packet")
        return int.from_bytes(data[off : off + width], "big" if leaf.big_endian else "little")


class Eds:
    """All EDS packages loaded from a cFS checkout + mission defines."""

    # cFE MsgId bit layout, V1 (SpacePacketBasic) headers; mirrors
    # CFE_MissionLib_TryMapping in cfe_missionlib_runtime_default.c:
    #   MsgId = {type @11-12 | (instance-1) @8-9 | (topic - region base) @0-7}
    # with global topics carrying the 0x0060 signature.
    CMD_TYPE_BITS = 0x1800
    TLM_TYPE_BITS = 0x0800
    GLOBAL_BITS = 0x0060
    INSTANCE_SHIFT = 8
    INSTANCE_MASK = 0x03

    def __init__(self) -> None:
        """Initialize empty registries."""
        self.constants = Constants()
        self.types: dict[str, tuple[str, ET.Element, str]] = {}  # ref -> (kind, elem, pkg)
        self.packages: list[tuple[str, ET.Element]] = []
        self.warnings: list[str] = []

    # ---- loading -------------------------------------------------------- #

    # One typo in NASA's CF EDS, repaired at load time instead of in the
    # core. Its source is pristine by law (cfs_mission/cfs.lock); this dict
    # is the mission-side record of the drift, like MODEL_DRIFT_EXCLUSIONS -
    # but here the measured symptom was silence, not mismatch: the value of
    # ``EotTlmTopicId`` closes its ${...} one byte late, so the constant
    # never resolves and CF's whole EOT packet - the only per-file testimony
    # CF gives - drops out of the defs and the beacon subscription table.
    # The wire carries 180-byte EOTs either way; the ground just cannot read
    # them. Drop an entry when the pin moves past the upstream fix: a repair
    # that finds nothing to repair is reported as a warning below, so this
    # table cannot quietly fossilize.
    VENDOR_REPAIRS: ClassVar[dict[str, tuple[str, str]]] = {
        "apps/cf/eds/cf.xml": (
            "${CFE_MISSION/CF_EOT_TLM_TOPICID)}",
            "${CFE_MISSION/CF_EOT_TLM_TOPICID}",
        ),
    }

    def _vendor_repaired(self, path: Path) -> str:
        """File text with this mission's known upstream typos corrected.

        The core file on disk is never touched: the repair lives in memory,
        between the pristine checkout and the model, where it can be named,
        counted and deleted.
        """
        text = path.read_text()
        repair = next(
            (rule for name, rule in self.VENDOR_REPAIRS.items() if path.as_posix().endswith(name)),
            None,
        )
        if repair is None:
            return text
        broken, fixed = repair
        if broken not in text:
            self.warnings.append(
                f"{path.name}: repair target {broken} no longer present - upstream "
                "fixed it, delete the VENDOR_REPAIRS entry"
            )
            return text
        return text.replace(broken, fixed)

    def load_xml(self, path: Path, mission: bool = False) -> None:
        """Load one XML file: mission defines and/or data-type packages.

        A ``DesignParameters`` root is defines-only. Since P3c the mission
        directory also holds ``PackageFile`` roots (the custom app packages),
        so being in the mission list does not by itself mean "defines only":
        those parse as packages like any module EDS.
        """
        root = ET.fromstring(self._vendor_repaired(path))
        self.constants._scan_defines(root)
        if _local(root.tag) == "DesignParameters":
            return
        for pkg in root.iter(f"{SEDS_NS}Package"):
            pkg_name = pkg.get("name", "")
            if _find_local(pkg, "DataTypeSet") is not None:
                self.packages.append((pkg_name, pkg))
            data_types = _find_local(pkg, "DataTypeSet")
            if data_types is None:
                continue
            for elem in data_types:
                kind = _local(elem.tag)
                name = elem.get("name")
                if name:
                    self.types[f"{pkg_name}/{name}"] = (kind, elem, pkg_name)

    # ---- type resolution -------------------------------------------------- #

    def _lookup(self, ref: str, ctx: str = "") -> tuple[str, ET.Element, str]:
        ref = self.constants.expand(ref)
        if "/" not in ref:
            if not ctx:
                raise UnknownType(f"{ref} (bare type ref outside a package)")
            ref = f"{ctx}/{ref}"
        try:
            return self.types[ref]
        except KeyError as exc:
            raise UnknownType(ref) from exc

    def _bits_attr(self, enc: ET.Element | None, default: str) -> int:
        """SizeInBits that may itself be a ``${...}`` reference."""
        if enc is None:
            return int(default)
        return self.constants.expand_to_int(enc.get("sizeInBits", default))

    def leaf(self, ref: str, name: str, depth: int = 0, ctx: str = "") -> list[Item]:
        """Resolve a type reference to flat leaf items (may be several)."""
        kind, elem, owner = self._lookup(ref, ctx)
        # CCSDS/CFE_HDR go on the wire network-order (the message module
        # encodes them), and explicit byteOrder="bigEndian" says the same;
        # everything else is a C struct field: target-native order (x86 LE).
        #
        # This deliberately overrides CFE_MISSION/DATA_BYTE_ORDER (which cFS sets to
        # bigEndian, and which cFE ignores for payloads in a non-EDS build). Proven
        # item-by-item against live packets: the runtime is big-endian headers +
        # little-endian payload, and flipping that define to littleEndian instead
        # would byte-swap the time secondary header, because the header types inherit
        # it from vendored cFS XML we keep pristine. So the override is load-bearing.
        enc = _find_any(elem, "IntegerDataEncoding", "RealDataEncoding", "FloatDataEncoding")
        be = owner in ("CCSDS", "CFE_HDR") or (
            enc is not None and enc.get("byteOrder") == "bigEndian"
        )
        items = self._leaf_impl(kind, elem, owner, name, depth)
        if be:
            for item in items:
                if isinstance(item, Leaf):
                    item.big_endian = True
        return items

    def _leaf_impl(
        self, kind: str, elem: ET.Element, owner: str, name: str, depth: int
    ) -> list[Item]:
        if depth > 24:
            raise UnknownType(f"{owner} (type reference nests too deeply)")
        if kind == "AliasDataType":
            return self.leaf(elem.get("type", ""), name, depth + 1, owner)
        if kind == "IntegerDataType":
            enc = _find_local(elem, "IntegerDataEncoding")
            bits = self._bits_attr(enc, "8")
            base = "int" if enc is not None and enc.get("encoding") == "signed" else "uint"
            return [Leaf(name, bits, base)]
        if kind in ("RealDataType", "FloatDataType"):
            enc = _find_any(elem, "RealDataEncoding", "FloatDataEncoding")
            bits = self._bits_attr(enc, "32")
            return [Leaf(name, bits, "float")]
        if kind == "BooleanDataType":
            return [Leaf(name, 1, "uint")]
        if kind == "StringDataType":
            # strings carry an explicit `length` in characters, no encoding
            # child; "string" (not "binary") so the ground defs stay text
            length = elem.get("length", "0")
            return [Leaf(name, 8 * self.constants.expand_to_int(length), "string")]
        if kind == "BinaryDataType":
            enc = _find_local(elem, "BinaryDataEncoding")
            return [Leaf(name, self._bits_attr(enc, "0"), "binary")]
        if kind in ("EnumerationDataType", "EnumeratedDataType"):
            enc = _find_any(elem, "IntegerDataEncoding", "EnumeratorList/IntegerDataEncoding")
            bits = self._bits_attr(enc, "16")
            states: dict[int, str] = {}
            for e in _findall_local(elem, "EnumeratorList/Enumerator"):
                try:
                    code = self.constants.expand_to_int(e.get("value", "0"))
                except (KeyError, ValueError):
                    continue
                states[code] = e.get("name", "?")
            return [Leaf(name, bits, "uint", states)]
        if kind == "ArrayDataType":
            ref_elem = elem.get("dataTypeRef", "")
            dims = _findall_local(elem, "DimensionList/Dimension")
            count = 1
            if dims:
                count = self.constants.expand_to_int(dims[0].get("size", "1"))
            inner = self.leaf(ref_elem, name, depth + 1, owner)
            if len(inner) == 1 and count <= 64:
                return [_index(inner[0], i) for i in range(count)]
            # multi-field arrays or huge arrays: emit as opaque binary blob
            return [Leaf(name, _bits(inner) * count, "binary")]
        if kind == "ContainerDataType":
            return self.container_items(elem, name, depth, owner)
        if kind == "BitFieldDataType":
            size = elem.get("sizeInBits", "0")
            return [Leaf(name, self.constants.expand_to_int(size), "binary")]
        raise UnknownType(f"{name} (kind {kind})")

    def container_items(self, elem: ET.Element, prefix: str, depth: int, owner: str) -> list[Item]:
        """Flatten a container (baseType chain + its own EntryList)."""
        items: list[Item] = []
        base_ref = elem.get("baseType")
        if base_ref:
            items.extend(self.leaf(base_ref, prefix, depth + 1, owner))
        entries = _find_local(elem, "EntryList")
        if entries is None:
            return items
        for entry in entries:
            tag = _local(entry.tag)
            if tag in ("Entry", "LengthEntry"):
                ref = entry.get("type", "")
                sub = entry.get("name", "") or ref.rsplit("/", 1)[-1]
                try:
                    leaves = self.leaf(ref, f"{prefix}.{sub}", depth + 1, owner)
                except UnknownType:
                    # An entry we cannot size would misalign every later
                    # field: keep the failure loud, the caller skips packet.
                    raise
                # arraySize repeats the entry's type: ignoring it would
                # silently shrink a packet, so the repeat happens here.
                count = self.constants.expand_to_int(entry.get("arraySize", "1"))
                if count > 1:
                    if len(leaves) == 1 and count <= 64:
                        leaves = [_index(leaves[0], i) for i in range(count)]
                    else:
                        leaves = [Leaf(f"{prefix}.{sub}", _bits(leaves) * count, "binary")]
                items.extend(leaves)
            elif tag == "PaddingEntry":
                items.append(
                    Padding("padding", self.constants.expand_to_int(entry.get("sizeInBits", "0")))
                )
            elif tag == "ErrorControlEntry":
                # e.g. the command checksum byte: model as an opaque uint8
                items.append(Leaf(f"{prefix}.{entry.get('name', 'ErrorControl')}", 8, "uint"))
        return items

    def bits_of_ref(self, ref: str) -> int:
        """Total size of a type in bits (for PKT length sanity)."""
        return _bits(self.leaf(ref, "x"))

    # ---- packets ---------------------------------------------------------- #

    def packets(self) -> list[Packet]:
        """Build packet definitions for every CFE_SB interface found."""
        packets: list[Packet] = []
        for pkg_name, pkg in self.packages:
            if pkg_name in ("CFE_MISSION", "CFE_HDR", "CCSDS", "BASE_TYPES"):
                continue
            try:
                packets.extend(self._component_packets(pkg_name, pkg))
            except (UnknownType, ValueError, KeyError) as exc:
                self.warnings.append(f"{pkg_name}: skipped packets ({exc})")
        return packets

    def topic_to_msgid(self, direction: str, topic: int, instance: int = 1) -> int:
        """Map an EDS topic ID to a runtime MsgId (cpu ``instance``, default 1).

        Exact replica of ``CFE_MissionLib_MapListener/PublisherComponent``
        for V1 headers: region decides the type/global signature bits, the
        topic contributes its offset within the region (8 bits), the CPU
        instance goes into bits 8-9.
        """
        c = self.constants
        if direction == "cmd":
            g_base, g_max = (
                c.resolve("CFE_MISSION/GLOBAL_CMD_BASE_TOPICID"),
                c.resolve("CFE_MISSION/GLOBAL_CMD_MAX_TOPICID"),
            )
            base, check = c.resolve("CFE_MISSION/TELECOMMAND_BASE_TOPICID"), self.CMD_TYPE_BITS
        else:
            g_base, g_max = (
                c.resolve("CFE_MISSION/GLOBAL_TLM_BASE_TOPICID"),
                c.resolve("CFE_MISSION/GLOBAL_TLM_MAX_TOPICID"),
            )
            base, check = c.resolve("CFE_MISSION/TELEMETRY_BASE_TOPICID"), self.TLM_TYPE_BITS
        if g_base <= topic < g_max:  # global topics bypass the instance bits
            return check | self.GLOBAL_BITS | (topic - g_base)
        idx = topic - base
        if not 0 <= idx < 1 << self.INSTANCE_SHIFT:  # 8-bit topic field (V1 packing)
            raise ValueError(f"topic {topic} outside {direction} region (base {base})")
        return check | ((instance - 1) & self.INSTANCE_MASK) << self.INSTANCE_SHIFT | idx

    def _component_packets(self, pkg_name: str, pkg: ET.Element) -> list[Packet]:
        out: list[Packet] = []
        for component in _findall_local(pkg, "ComponentSet/Component"):
            topic_ids = self._topic_ids(component)
            interfaces = _findall_local(component, "RequiredInterfaceSet/Interface")
            for iface in interfaces:
                iface_type = iface.get("type", "")
                if "CFE_SB" not in iface_type:
                    continue
                if_name = iface.get("name", "")
                msgid = topic_ids.get(if_name)
                if msgid is None:
                    continue
                data_type = _interface_data_type(iface, iface_type)
                if data_type is None:
                    continue
                direction = "cmd" if "Telecommand" in iface_type else "tlm"
                msgid = self.topic_to_msgid(direction, msgid)
                ref = f"{pkg_name}/{data_type}"
                items = self.leaf(ref, pkg_name)
                fcs = {} if direction == "tlm" else self._function_codes(pkg_name, data_type)
                out.append(
                    Packet(
                        target_app=pkg_name,
                        name=f"{pkg_name}_{if_name}",
                        msgid=msgid,
                        direction=direction,
                        description=iface.get("shortDescription", ""),
                        items=items,
                        function_codes=fcs,
                    )
                )
        return out

    def _topic_ids(self, component: ET.Element) -> dict[str, int]:
        vars_: dict[str, int] = {}
        variables = _find_local(component, "Implementation/VariableSet")
        if variables is not None:
            for var in _findall_local(variables, "Variable"):
                initial = var.get("initialValue", "")
                if _REF_RE.search(initial) or initial.isdigit():
                    try:
                        vars_[var.get("name", "")] = self.constants.expand_to_int(initial)
                    except (KeyError, ValueError):
                        continue
        out: dict[str, int] = {}
        for pmap in _findall_local(component, "Implementation/ParameterMapSet/ParameterMap"):
            if pmap.get("parameter") != "TopicId":
                continue
            msgid = vars_.get(pmap.get("variableRef", ""))
            if msgid is not None:
                out[pmap.get("interface", "")] = msgid
        return out

    def _full_ref(self, pkg: str, ref: str) -> str:
        expanded = self.constants.expand(ref)
        return expanded if "/" in expanded else f"{pkg}/{expanded}"

    def _function_codes(self, pkg_name: str, root_type: str) -> dict[str, int]:
        """Containers constrained on Sec.FunctionCode under this command type."""
        chain: set[str] = set()
        frontier = [(pkg_name, root_type)]
        while frontier:
            pkg, name = frontier.pop()
            full = self._full_ref(pkg, name)
            if full in chain:
                continue
            chain.add(full)
            entry = self.types.get(full)
            if entry is not None and entry[1].get("baseType"):
                base = entry[1].get("baseType", "")
                frontier.append((entry[2], base))
        fcs: dict[str, int] = {}
        for ref, (kind, elem, pkg) in self.types.items():
            if kind != "ContainerDataType" or pkg != pkg_name or ref in chain:
                continue
            base_ref = elem.get("baseType")
            if not base_ref or self._full_ref(pkg, base_ref) not in chain:
                continue
            constraint = _find_local(elem, "ConstraintSet/ValueConstraint")
            if constraint is not None and constraint.get("entry", "").endswith("FunctionCode"):
                fcs[elem.get("name", ref)] = self.constants.expand_to_int(
                    constraint.get("value", "0")
                )
        return fcs


class UnknownType(Exception):
    """A type reference could not be resolved (caller may skip packet)."""


def _index(item: Item, i: int) -> Item:
    if isinstance(item, Leaf):
        return Leaf(f"{item.name}_{i}", item.bits, item.cbase, item.states)
    return Padding(f"{item.name}_{i}", item.bits)


def _bits(items: list[Item]) -> int:
    return sum(i.bits * getattr(i, "array_count", 1) for i in items)


def _interface_data_type(iface: ET.Element, iface_type: str) -> str | None:
    want = "TelecommandDataType" if "Telecommand" in iface_type else "TelemetryDataType"
    for gmap in _findall_local(iface, "GenericTypeMapSet/GenericTypeMap"):
        if gmap.get("name") == want:
            return gmap.get("type")
    return None


def load_eds(cfs_root: Path, mission_def_files: list[Path], extra_files: list[Path]) -> Eds:
    """Load all module/app EDS XML plus the mission define files.

    The checkout file list comes from :func:`codegen.eds_sources.core_xml_files`,
    the same enumeration the EdsLib runtime-database build in ``edslink/`` uses.
    """
    eds = Eds()
    for path in core_xml_files(cfs_root):
        eds.load_xml(path)
    for path in extra_files:
        eds.load_xml(path)
    for path in mission_def_files:
        eds.load_xml(path, mission=True)
    return eds


__all__ = ["Constants", "Eds", "Item", "Leaf", "Packet", "Padding", "load_eds"]
