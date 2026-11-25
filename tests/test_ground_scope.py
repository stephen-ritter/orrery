"""The ground dictionary is scoped by the same wake list that scopes the boot.

Rung three of the source-of-truth ladder (docs/what_the_scenario_rules.md):
`eds2cosmos --scope-scenario` cuts a vehicle's COSMOS dictionary to the apps
its scenario wakes. The tests here pin the two halves of the promise: the
flight that already happens changes not one byte (the declared-eye scope IS
the shipped dictionary), and the vehicle that declared no eye is offered no
command for an organ it never grew.
"""

from __future__ import annotations

import functools

import pytest

from codegen.eds2cosmos import (
    Packet,
    build_eds,
    emit_cosmos,
    mission_packets,
    scope_packets,
    woken_app_names,
)
from codegen.emit_startup import StartupDoorError
from sim.parts import REPO

CFS = REPO / "cfs_mission" / "cFS"
MISSION_DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds"
SCENARIOS = REPO / "sim" / "scenarios"
SHIPPED = REPO / "cosmos_plugin" / "targets" / "SAT_CHIEF" / "cmd_tlm"


@functools.lru_cache(maxsize=1)
def _packets() -> tuple[Packet, ...]:
    return tuple(mission_packets(build_eds(CFS, MISSION_DEFS).packets()))


def _scope(scenario: str, vehicle: str) -> tuple[str, str]:
    woken = woken_app_names(SCENARIOS / scenario, vehicle)
    return emit_cosmos(scope_packets(list(_packets()), woken), target=f"SAT_{vehicle.upper()}")


# --- the flight that already happens does not move ---------------------------


def test_a_declared_eye_scopes_to_exactly_the_shipped_dictionary() -> None:
    """Every check that commands the eye commands THIS byte sequence, still.

    The scoped emission for a vehicle that mounted the camera must equal the
    committed definitions: scoping is a filter, not a re-render - the
    fingerprint, the ordering, the exclusions, all untouched.
    """
    cmd_text, tlm_text = _scope("camera_ops.toml", "chief")
    assert cmd_text == (SHIPPED / "sat_chief_cmd_def.txt").read_text(encoding="utf-8")
    assert tlm_text == (SHIPPED / "sat_chief_tlm_def.txt").read_text(encoding="utf-8")


# --- the vehicle that declared no eye is offered nothing it cannot honor -----

CAM_COMMANDS = ("CAM_CMD", "CAM_SET_CONFIG", "CAM_SEND_HK")


def test_the_undeclared_vehicle_loses_exactly_the_camera_authority() -> None:
    """Three commands and one housekeeping packet: CAM's whole dictionary.

    The delta must be exactly that - no core command may scope away, and the
    frames that ride the generic consumer (CamState, OpNav) stay: they are
    shared vocabulary owned by bsk_link, scoped by row-landing on the wire,
    not by the boot record - the same law that keeps NavState defined on a
    vehicle whose GNSS row never streams.
    """
    catalog_cmd, catalog_tlm = emit_cosmos(list(_packets()))
    bare_cmd, bare_tlm = _scope("chief.toml", "chief")
    for name in CAM_COMMANDS:
        assert f"COMMAND SAT_CHIEF {name} " in catalog_cmd
        assert f"COMMAND SAT_CHIEF {name} " not in bare_cmd
    assert "TELEMETRY SAT_CHIEF CAM_HK_TLM " in catalog_tlm
    assert "TELEMETRY SAT_CHIEF CAM_HK_TLM " not in bare_tlm
    assert catalog_cmd.count("\nCOMMAND ") - bare_cmd.count("\nCOMMAND ") == 3
    assert catalog_tlm.count("\nTELEMETRY ") - bare_tlm.count("\nTELEMETRY ") == 1
    assert "TELEMETRY SAT_CHIEF BSK_LINK_CAM_STATE_TLM " in bare_tlm  # rides the generic carrier
    assert "TELEMETRY SAT_CHIEF CAM_HK_TLM " in _scope("camera_record.toml", "chief")[1]


def test_both_twin_vehicles_scope_alike_and_core_survives() -> None:
    """Heterogeneity is an input, and the core's commands are nobody's to scope.

    The same scenario cuts both vehicles the same honest way; the core's own
    commands survive in every dictionary because the core always boots.
    """
    for vehicle in ("chief", "deputy"):
        cmd_text, tlm_text = _scope("twin_ops.toml", vehicle)
        target = f"SAT_{vehicle.upper()}"
        assert f"COMMAND {target} CAM_CMD " not in cmd_text
        assert f"COMMAND {target} CFE_ES" in cmd_text  # the core always boots
        assert "APPEND_ID_ITEM PKTID" in tlm_text


# --- the ownership registry refuses lies --------------------------------------


def test_every_packet_app_is_owned_by_someone() -> None:
    """The scope door's own inventory check: no packet roams unclaimed.

    A new app whose EDS package nobody declared would scope itself out of
    every dictionary in silence; the door refuses that shape on sight.
    """
    from codegen.emit_startup import load_apps

    owned = {pkg for row in load_apps() for pkg in row.eds_packages}
    orphans = (
        {p.target_app for p in _packets()}
        - owned
        - {p.target_app for p in _packets() if p.target_app.startswith("CFE_")}
    )
    assert not orphans, f"EDS packages no app claims (add eds_packages): {sorted(orphans)}"


def test_scope_refuses_a_package_no_app_claims() -> None:
    """A packet from an unclaimed package has no honest scope: the door says so."""
    ghost = Packet(
        target_app="GHOST_PKG",
        name="GHOST_TLM",
        msgid=0x09FF,
        direction="tlm",
        description="a packet from nowhere",
        items=[],
        function_codes={},
    )
    with pytest.raises(StartupDoorError, match="no owning app"):
        scope_packets([ghost], {"cam"})


def test_woken_names_come_from_the_door_itself() -> None:
    """One wake list, two consumers: the startup generator IS the authority."""
    assert "cam" not in woken_app_names(SCENARIOS / "chief.toml", "chief")
    assert "cam" in woken_app_names(SCENARIOS / "camera_downlink.toml", "chief")
    assert "bsk_link" in woken_app_names(SCENARIOS / "chief.toml", "chief")
