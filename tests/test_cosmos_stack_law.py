"""The station script's own laws, pinned offline.

A green word from a wrapper is a claim; the registry is the proof. The yard
learned that twice on the load path - 2026-11-24 (a crashed load still
printed "loaded") and 2026-11-27 (a version bump unloaded the NEW version's
name, a silent no-op, and the half-installed entry listed a plugin that
served no targets) - and neither lesson survived in code until it was a
test. These read the script's text, the way the firmware guard's tests do:
cheap, offline, and refusing the exact failure class that already happened.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
STACK = REPO / "scripts" / "cosmos_stack.sh"


def test_the_load_never_unloads_the_name_of_what_is_not_loaded() -> None:
    """Unload by what `cli list` carries, never by the version being installed.

    The entry is versioned (openc3-cosmos-orrery-<ver>.gem__<n>): unloading
    the NEW version's name before a bump is the no-op that let the old entry
    survive, the load half-install beside it, and the replace sweep kill the
    fresh microservices.
    """
    text = STACK.read_text(encoding="utf-8")
    assert re.search(r"run_cli list DEFAULT.*openc3-cosmos-orrery-", text, re.S), (
        "cosmos_stack.sh no longer asks the registry which entry is loaded"
    )
    assert 'unload "openc3-cosmos-orrery-$GEM_VERSION.gem__0"' not in text, (
        "unloading the version-about-to-be-installed is the race, back verbatim"
    )


def test_a_loaded_plugin_must_be_SERVING_before_the_script_says_loaded() -> None:
    """The post-load tripwire: get_target_names answers SAT_CHIEF, or exit 1.

    A listed-but-not-serving entry is the half-installed registry; the gates
    would meet it three minutes later as a missing-packet timeout, which is
    how a station bug becomes folklore.
    """
    text = STACK.read_text(encoding="utf-8")
    assert "get_target_names" in text, "the registry tripwire left with the last fix"
    assert "SAT_CHIEF" in text
    tail = text[text.rindex("COSMOS stack up") :]
    assert "SERVING" in tail, "the closing sentence says loaded without saying served"


def test_the_crash_and_the_lie_still_refuse() -> None:
    """The 2026-11-24 guards survive the rewrite: backtrace and lying cache."""
    text = STACK.read_text(encoding="utf-8")
    assert "in '<main>'" in text, "the crash signature check is gone"
    assert "No version change detected" in text, "the lying-registry check is gone"
