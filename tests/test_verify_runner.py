"""The check runner: properties the runner itself must keep.

`scripts/verify.sh` has now failed in production twice, both times for the same
reason: a check is executed in a CHILD shell (`timeout … bash -c "$cmd"`), and the
build check was first a shell function the child could not see (rc=127), then a
function whose `$REPO` reached the child as a name rather than a value
(`docker: invalid spec: :/work`, rc=125). Both were invisible to every offline
mirror, because nothing imported the runner - so the runner gets its own tests.

These are cheap on purpose (`--list` is side-effect-free): a runner whose guards
cost minutes is a runner people bypass.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RUNNER = REPO / "scripts" / "verify.sh"
TEXT = RUNNER.read_text(encoding="utf-8")

# Commands must be resolvable without this file's environment; a token list is
# also the honest description of what a sweep needs installed.
ALLOWED_ENTRY = re.compile(r'^\s*"(?P<entry>(?:offline|build|live)\|[^"]*)"\s*$')


def gate_entries() -> list[dict[str, str]]:
    """The GATES table, parsed rather than trusted.

    Parsed from the shell source because that table IS the acceptance ladder: if
    a test kept its own copy of the list, the two could disagree and the runner
    would quietly sweep fewer checks than the suite claims exist.
    """
    rows: list[dict[str, str]] = []
    for line in TEXT.splitlines():
        match = ALLOWED_ENTRY.match(line)
        if not match:
            continue
        fields = match.group("entry").split("|")
        assert len(fields) == 5, f"a check row must be tier|name|proves|cmd|tag: {line}"
        tier, name, proves, cmd, tag = fields
        rows.append({"tier": tier, "name": name, "proves": proves, "cmd": cmd, "tag": tag})
    return rows


def test_the_table_is_not_empty_or_hollow() -> None:
    """A runner that silently selected nothing would report a green sweep."""
    checks = gate_entries()
    assert len(checks) >= 13, f"expected the full ladder, found {len(checks)} checks"
    for check in checks:
        assert check["proves"], f"{check['name']}: a check that proves nothing is a sleep"
        assert check["cmd"], f"{check['name']}: no command"
    assert {g["tier"] for g in checks} == {"offline", "build", "live"}


def test_no_gate_depends_on_the_runners_own_environment() -> None:
    """The rc=125 invariant, stated once: check commands are argv, not prose.

    A `$` in a check command means it needs a variable this shell happens to have,
    and the child shell does not happen to have it. That is how the mission build
    asked docker to mount `:/work`.
    """
    offenders = [g["name"] for g in gate_entries() if "$" in g["cmd"] or "`" in g["cmd"]]
    assert not offenders, (
        f"check commands must be literal argv or script paths (the child shell has "
        f"none of this file's variables): {offenders}"
    )


def test_no_gate_is_a_shell_function() -> None:
    """`export -f` is the shape that failed twice; forbid the shape, not the case."""
    assert "export -f" not in TEXT, (
        "a check that is a shell function is invisible to `bash -c` (rc=127) - "
        "give it a script, which is also what a human would type"
    )


def test_every_gate_target_exists() -> None:
    """Renaming a check driver must fail here, not three minutes into a sweep."""
    for check in gate_entries():
        for target in re.findall(r"(?:scripts|edslink|cfs_mission)/[\w./-]+", check["cmd"]):
            path = REPO / target
            assert path.exists(), f"{check['name']}: {target} does not exist"
        first = check["cmd"].split()[0]
        assert subprocess.run(
            ["bash", "-c", 'command -v "$1"', "bash", first],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip(), f"{check['name']}: child shell cannot resolve {first!r}"


def test_the_blocked_rule_runs_before_the_yard_is_touched() -> None:
    """A broken build must not even take the operator's fleet down.

    Ordering is the whole rule: report BLOCKED before the sweep, or the runner
    destroys the standing yard on its way to refusing to use it.
    """
    blocked = TEXT.index('"BLOCKED"')
    sweep = TEXT.index("sweeping the yard")
    assert blocked < sweep, "the live-tier BLOCKED check must precede the fleet sweep"
    assert "BUILD_BROKEN" in TEXT


def test_tiers_are_cumulative_and_cheap_to_inspect() -> None:
    """`--tier live` also runs offline and build, and `--list` proves it free.

    This is the design that keeps live checks from flying a stale build directory:
    `fleet.sh` boots the ALREADY-BUILT container-start, so a live-only request is
    a request to fly firmware nobody compiled.
    """

    def tiers(*args: str) -> set[str]:
        result = subprocess.run(
            ["bash", str(RUNNER), *args],
            cwd=REPO,
            env={**os.environ, "GIT_DIR": ""},
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        return set(re.findall(r"\[(offline|build|live)\s*\]", result))

    assert tiers("--tier", "offline", "--list") == {"offline"}
    assert tiers("--tier", "build", "--list") == {"offline", "build"}
    assert tiers("--list") == {"offline", "build", "live"}, "live must imply build"


def test_an_unknown_tier_is_refused_rather_than_flying_everything() -> None:
    """A typo'd tier must fail loudly, never silently mean "no checks"."""
    result = subprocess.run(
        ["bash", str(RUNNER), "--tier", "liv"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "no such tier" in result.stderr


def test_every_live_gate_names_the_driver_it_runs() -> None:
    """Live checks are the fleet ladder: each must be a driver in scripts/.

    A live check that is an inline blob of docker and python is a check whose
    verdict cannot be reproduced by hand at 2 a.m.
    """
    for check in gate_entries():
        if check["tier"] == "live":
            assert check["cmd"].startswith("bash scripts/"), (
                f"{check['name']}: live checks run a driver script, not an inline blob"
            )
