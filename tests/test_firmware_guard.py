"""The firmware provenance guard: no gate flies flight software it did not build.

`scripts/fleet.sh` and the gate drivers start a binary that sits on disk whether
or not it matches the sources that produced it. `scripts/firmware.sh` gives the
missing sentence - a content hash of the mission's inputs, stamped by the build
and demanded by every launcher - and this file is what stops the guard from
becoming folklore again:

* **every** launcher of `container-start` must call it, including the scripts
  someone adds next month, because a guard on one path and not the others is a
  guard that gets bypassed by accident;
* the refusal, the override, and the happy path each say a different true thing,
  so all four are pinned by running the script rather than by reading it;
* the hash's INPUT SET is pinned, because a guard silently narrowed to fewer
  files is the most dangerous kind - it keeps passing. The one deliberate
  narrowing is in there too, and says so: prose is not a build input, because a
  guard that grounds the fleet over a README typo gets switched off.
* the cFS core is fetched from `cfs_mission/cfs.lock` rather than committed, and
  the guard compares what is on disk with that pin: the core drifting off its
  pin is invisible in every other signal this yard produces.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tomllib
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
GUARD = REPO / "scripts" / "firmware.sh"
SCRIPTS = REPO / "scripts"
LOCK = REPO / "cfs_mission" / "cfs.lock"
CORE = REPO / "cfs_mission" / "cFS"

# these guard flights read the real pinned core; a fresh clone has none
needs_core = pytest.mark.skipif(
    not CORE.is_dir(), reason="pinned cFS core not fetched here: scripts/fetch_cfs.sh"
)


def _pin_of_the_lock() -> str:
    """The pin, read by a real TOML parser: a third opinion on the file format.

    The fetcher greps it (it runs before Python is installed) and the guard
    greps it (it runs inside a container that must not depend on the fetcher).
    Both are wrong in the same way if the format moves, so the test reads it
    properly and forces the two shell readers to agree with that.
    """
    with LOCK.open("rb") as handle:
        return str(tomllib.load(handle)["commit"])


# Files whose bytes change the mission binary. If one of these ever stops being
# tracked, the stamp stops describing what the compiler saw, and the guard
# starts passing for the wrong reason.
PROVENANCE_INPUTS = (
    "cfs_mission/mission/orrery_defs/inc/orrery_frames.h",  # the baked registry digest
    "cfs_mission/mission/orrery_defs/eds/bsk_link.xml",  # the link schema
    "cfs_mission/mission/orrery_defs/inc/orrery_part_msgids.h",  # generated identity
    "cfs_mission/mission/orrery_defs/inc/orrery_part_topicids.h",
    "cfs_mission/mission/orrery_defs/targets.cmake",  # which apps are compiled in
    "cfs_mission/mission/apps/bsk_link/fsw/src/bsk_link.c",
)


def _guard(verb: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    # fixed argv, this repo's own script - no shell, no user input
    return subprocess.run(
        ["bash", str(GUARD), verb],
        cwd=REPO,
        env={**os.environ, **(env or {})},
        capture_output=True,
        text=True,
        check=False,
    )


def _fake_yard(tmp_path: Path, *, with_binary: bool = True) -> dict[str, str]:
    """A launch path pointed at a scratch build tree, so nothing real is touched."""
    exe = tmp_path / "exe"
    exe.mkdir(parents=True, exist_ok=True)
    if with_binary:
        (exe / "core-cpu1").write_text("#!/bin/sh\n", encoding="utf-8")
        (exe / "core-cpu1").chmod(0o755)
    return {"EXE_DIR": str(exe), "STAMP_FILE": str(tmp_path / "orrery_firmware.sha")}


def _run(verb: str, cwd: Path, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(
        ["bash", str(GUARD), verb],
        cwd=cwd,
        env={**os.environ, **(env or {})},
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout if result.returncode == 0 else f"ERROR: {result.stdout}{result.stderr}"


def test_the_hash_means_the_same_thing_from_any_directory(tmp_path: Path) -> None:
    """Pinned because the mission build broke exactly here.

    `git ls-files` answers in paths relative to the REPOSITORY ROOT while callers
    stand where they stand - cfs_mission/build.sh stamps from cfs_mission/, and
    the guard refused to hash a tree it could not resolve. A stamp that differs
    per caller is a guard that fails on Tuesdays for reasons nobody can name.
    """
    from_root = _run("hash", REPO)
    from_mission = _run("hash", REPO / "cfs_mission")
    from_elsewhere = _run("hash", tmp_path)
    assert from_root == from_mission == from_elsewhere, (
        "provenance must not depend on the caller's directory"
    )


@needs_core
def test_a_stamp_taken_in_the_container_checks_out_on_the_host(tmp_path: Path) -> None:
    """The two halves of the guard run in different worlds.

    cfs_mission/build.sh stamps INSIDE orrery-cfs:dev (cwd /work/cfs_mission);
    the launchers check OUTSIDE it. That seam is where the bug lived, so the seam
    is what gets tested - not each half alone.
    """
    env = _fake_yard(tmp_path)
    assert _run("stamp", REPO / "cfs_mission", env).startswith("firmware: stamped")
    checked = _run("check", REPO, env)
    assert checked.startswith("firmware: provenance OK"), checked


def test_the_guard_hash_is_cheap_and_stable() -> None:
    """Two runs agree, and it costs milliseconds enough to run on every launch.

    A pre-flight check that takes seconds gets dropped from the launch path;
    this one hashes the build inputs under `cfs_mission/` (about eighty) and the
    cFS pin in `cfs_mission/cfs.lock`.
    """
    first, second = _guard("hash"), _guard("hash")
    assert first.returncode == 0, first.stderr
    assert first.stdout.strip() == second.stdout.strip()
    assert len(first.stdout.strip()) == 64


def test_a_missing_input_fails_instead_of_hashing_what_remains(tmp_path: Path) -> None:
    """A guard that prints a hash while complaining is a guard that lies.

    Pinned because the first version of this script did exactly that: the file
    list was piped into `sha256sum | sha256sum | awk`, so a missing input made
    the inner command fail - and the pipeline's exit status belonged to `awk`,
    so a truncated digest came out the other end saying OK. Reproduced here on a
    scratch repo rather than by trusting the fix: the bug needs a tracked file
    that has gone missing, which is one `rm` in a fixture and nothing in a
    developer's tree.
    """
    repo = tmp_path / "repo"
    (repo / "cfs_mission" / "mission").mkdir(parents=True)
    (repo / "scripts").mkdir()
    shutil.copy(GUARD, repo / "scripts" / "firmware.sh")
    shutil.copy(REPO / "cfs_mission" / "cfs.lock", repo / "cfs_mission" / "cfs.lock")
    source = repo / "cfs_mission" / "mission" / "app.c"
    source.write_text("void app(void) {}\n", encoding="utf-8")
    for args in (
        ["init", "-q"],
        ["-c", "user.email=t@.invalid", "-c", "user.name=test", "add", "-A"],
        ["-c", "user.email=t@invalid", "-c", "user.name=test", "commit", "-qm", "fixture"],
    ):
        subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)

    assert _run("hash", repo).strip(), "the fixture cannot hash its own tracked file"

    source.unlink()  # tracked, and gone: the state a half-checked-out tree produces
    result = subprocess.run(
        ["bash", str(repo / "scripts" / "firmware.sh"), "hash"],
        cwd=repo,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0, result.stdout
    assert not result.stdout.strip(), f"a truncated digest escaped as a stamp: {result.stdout!r}"
    assert "missing from the tree" in result.stderr + result.stdout


def test_the_core_is_fetched_and_never_committed() -> None:
    """The core is NASA's, unmodified, and ~500 MB: it may not become our content.

    A submodule keeps it out of the tree too, but at the cost of making the
    upstream repository a participant in `git status`: every accidental edit to
    the core arrives as `modified: cfs_mission/cFS` and is read as noise about
    git instead of as the rule violation it is. So the pin is committed and the
    checkout is ignored - which is only safe while that stays true, hence a test
    that looks rather than trusts.
    """
    tracked = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "--", "cfs_mission/cFS"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()
    assert tracked == [], f"the core is in the index again: {tracked}"

    assert not (REPO / ".gitmodules").exists(), ".gitmodules is back; the pin is the mechanism now"

    ignored = subprocess.run(
        ["git", "-C", str(REPO), "check-ignore", "-q", "cfs_mission/cFS/Makefile"],
        check=False,
    )
    assert ignored.returncode == 0, "the core is not ignored, so a build could dirty the tree"

    with LOCK.open("rb") as handle:
        pin = tomllib.load(handle)
    assert pin["checkout"] == "cfs_mission/cFS"
    assert len(pin["commit"]) == 40 and pin["tag"], pin
    assert pin["verify"]["required_paths"], "a pin nobody can check is a rumour"


@pytest.mark.skipif(not CORE.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh")
def test_the_pin_describes_the_core_that_is_actually_here() -> None:
    """The pin is a claim about a directory, and claims get checked.

    This is the one test that runs `--verify` for real: 25 nested repositories
    and all, about half a second. It fails on the day someone checks out a
    different cFS and forgets that the mission's identity includes the core.
    """
    result = subprocess.run(
        ["bash", str(SCRIPTS / "fetch_cfs.sh"), "--verify"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "core OK" in result.stdout, result.stdout


def test_prose_is_not_a_build_input() -> None:
    """A README typo must not ground the fleet - and the exclusion is audited.

    Both halves matter. Including documentation means every sentence that
    improves invalidates a yard of running containers, which trains people to set
    ORRERY_ALLOW_STALE and the guard becomes decoration. Excluding it silently is
    the other failure: a narrowed input set keeps passing. So the exclusion is
    asserted through the script's own `inputs` verb, next to an assertion that
    every file that changes the binary is still in there.
    """
    listed = _guard("inputs").stdout.split()
    assert listed, "the guard reports no inputs at all"
    assert not [path for path in listed if path.endswith(".md")], "prose re-entered the stamp"
    assert "cfs_mission/cfs.lock" not in listed, "the lock is an input twice over"
    for path in PROVENANCE_INPUTS:
        assert path in listed, f"{path} left the stamp: the guard now sees less"


def test_the_pin_moves_the_stamp_without_being_hashed_as_a_file(tmp_path: Path) -> None:
    """A lock comment is prose; the commit inside it is the binary's ancestry.

    The pin therefore enters as a named fact (`cfs-pin <sha>`) rather than as
    bytes, which is what makes rewording the lock harmless and moving it loud.
    """
    baseline = _guard("hash").stdout.strip()
    moved = tmp_path / "cfs.lock"
    moved.write_text(
        LOCK.read_text(encoding="utf-8").replace(_pin_of_the_lock(), "f" * 40),
        encoding="utf-8",
    )

    result = subprocess.run(
        ["bash", str(GUARD), "hash"],
        cwd=REPO,
        env={**os.environ, "CFS_LOCK": str(moved)},
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() != baseline, "a different core would wear the same stamp"


@needs_core
def test_the_guard_and_the_fetcher_read_the_same_pin(tmp_path: Path) -> None:
    """Two readers of `cfs_mission/cfs.lock`, deliberately not one calling the other.

    The guard stamps inside the build container and checks outside it, so it
    cannot depend on the fetcher's CLI; the cost of two parsers is a format that
    could drift silently, and that is what this test is for.
    """
    printed = subprocess.run(
        ["bash", str(SCRIPTS / "fetch_cfs.sh"), "--print"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    pin = dict(line.split("=", 1) for line in printed.splitlines())
    assert len(pin["commit"]) == 40, pin

    env = _fake_yard(tmp_path)
    stamped = _run("stamp", REPO / "cfs_mission", env)
    assert pin["commit"][:7] in stamped, f"the guard names a different cFS: {stamped}"
    assert pin["commit"] == _pin_of_the_lock(), "the two readers disagree about the pin"


def test_the_hash_covers_the_inputs_that_change_the_binary() -> None:
    """The guard's input set, pinned by name.

    Narrowing this list silently would turn the guard back into the superstition
    it replaced: it would keep saying OK while the thing it stopped checking was
    exactly what changed.
    """
    tracked = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "--", "cfs_mission"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.splitlines()

    for path in PROVENANCE_INPUTS:
        assert path in tracked, f"{path} is no longer tracked - the stamp cannot see it"


def test_an_unstamped_tree_is_refused(tmp_path: Path) -> None:
    """A binary with no stamp is not trusted, just because it exists.

    This is the state every checkout lands in when the guard is introduced, and
    the message has to say the one useful thing: rebuild once.
    """
    result = _guard("check", _fake_yard(tmp_path))
    assert result.returncode == 1
    assert "no provenance stamp" in result.stdout


def test_a_missing_binary_is_refused_with_the_build_command(tmp_path: Path) -> None:
    """No firmware is not a mystery to be deduced; it is a command to run."""
    result = _guard("check", _fake_yard(tmp_path, with_binary=False))
    assert result.returncode == 1
    assert "nothing built" in result.stdout
    assert "build.sh all" in result.stdout, "the refusal should hand over the fix"


def test_a_mismatched_stamp_is_refused_loudly(tmp_path: Path) -> None:
    """The headline of the refusal is the sentence, not an exit code."""
    env = _fake_yard(tmp_path)
    Path(env["STAMP_FILE"]).write_text("0" * 64 + "\n", encoding="utf-8")

    result = _guard("check", env)
    assert result.returncode == 1
    assert "NOT BUILT FROM THIS TREE" in result.stdout
    assert "stamped 000000000000" in result.stdout


@needs_core
def test_a_matching_stamp_launches_and_records_which_firmware(tmp_path: Path) -> None:
    """Success is not silent: the short hash lands in every gate log."""
    env = _fake_yard(tmp_path)
    want = _guard("hash").stdout.strip()

    stamp = _guard("stamp", env)
    assert stamp.returncode == 0, stamp.stderr
    assert Path(env["STAMP_FILE"]).read_text(encoding="utf-8").strip() == want

    result = _guard("check", env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "provenance OK" in result.stdout
    assert want[:12] in result.stdout


def test_the_override_flies_with_a_banner(tmp_path: Path) -> None:
    """ORRERY_ALLOW_STALE=1 is allowed and cannot be mistaken for a clean run."""
    env = _fake_yard(tmp_path)
    Path(env["STAMP_FILE"]).write_text("0" * 64 + "\n", encoding="utf-8")

    result = _guard("check", {**env, "ORRERY_ALLOW_STALE": "1"})
    assert result.returncode == 0
    assert "STALE FLYING" in result.stdout


def test_every_container_launcher_demands_provenance() -> None:
    """Every script that launches `container-start` must ask what built it.

    The guard's value is that it covers the paths nobody remembers - including
    `scripts/fly_fleet.sh` run straight from a shell, bypassing the runner.
    A new launcher that forgets the check fails here, naming itself. The count
    floor only proves the glob found the yard; the invariant is the forgotten
    list, so deleting a launcher never weakens this test.
    """
    # "./container-start" is the LAUNCH form; gates.sh names the path in prose
    # only, and prose does not need a guard bolted to it.
    launchers = sorted(p for p in SCRIPTS.glob("*.sh") if "./container-start" in p.read_text())
    assert len(launchers) >= 3, f"the glob found no yard: {launchers}"

    forgotten = [
        p.name
        for p in launchers
        if "firmware.sh" not in p.read_text() or 'firmware.sh" check' not in p.read_text()
    ]
    assert not forgotten, f"launchers that would fly unverified firmware: {forgotten}"


def test_fleet_sh_guards_launch_not_shutdown() -> None:
    """Bringing a stale yard DOWN must always be possible.

    A guard on the down path would be a guard that traps an operator unable
    to stop a fleet - which is how people end up disabling guards. The
    provenance check lives in the launch path (scenario_up, after compose's
    mission init has built what it found); the down branch stays clean.
    """
    text = (SCRIPTS / "fleet.sh").read_text()
    down = text.index("    down)")
    down_branch = text[down : text.index("    status)")]
    assert "firmware.sh" not in down_branch, "the down path grew a trap"
    launch = text[: text.index("legacy_pids_down()")]
    assert 'firmware.sh" check' in launch, "the launch path lost its provenance check"


def test_the_mission_build_stamps_itself() -> None:
    """A stamp nobody remembers to take is a stamp that quietly goes stale."""
    text = (REPO / "cfs_mission" / "build.sh").read_text()
    assert "firmware.sh" in text, "the build no longer stamps its own provenance"
    stamped = {
        "build": "build)   build && stamp",
        "install": "install) install && stamp",
        "all": "all)     prep && build && install && stamp",
    }
    for verb, expected in stamped.items():
        assert expected in text, f"the {verb} verb no longer stamps its provenance"
