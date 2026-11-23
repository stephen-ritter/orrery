"""The ground station's pin: cosmos.lock is a fact, and this module is the enforcement.

Why a pin for COSMOS when cFS has one: the CE stack deploys from a CHECKOUT -
`openc3.sh` sources that checkout's `.env` and starts the images it names - so
two version facts must agree. `OPENC3_TAG=latest`, the upstream default, lets
them drift apart silently; the deployment this pin was written for ran a
v6.4-era checkout against 7.5-era images because the checkout was cloned in May
and the images were pulled in October. A gate that "verifies COSMOS answers"
cannot see that: v6.4 compose against 7.5 images answers just fine, until the
day it answers differently.

So the rules tested here, through `scripts/get_cosmos.sh` itself rather than
through a reimplementation:

* the lock parses, names a peeled commit, and `--print` agrees with it;
* a wrong checkout reports EVERY disagreement at once (commit AND image tag) -
  the exact hybrid state, reproduced on a scratch repo;
* `--force` refuses anything that is not this pin's checkout, and refuses local
  edits beyond `.env` - the one tracked file the pin is allowed to write
  (upstream's deployment knob);
* the full round trip (clone from a file:// remote, tag `.env`, verify, drift,
  repair) works and is idempotent, including the move-back path where HEAD
  itself is off the pin.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
GET = REPO / "scripts" / "get_cosmos.sh"
LOCK = REPO / "cosmos.lock"

FIELDS = ("repository", "tag", "commit", "image_tag", "checkout")


def _lock_field(field: str) -> str:
    text = LOCK.read_text(encoding="utf-8")
    match = re.search(rf'^{field}\s*=\s*"([^"]*)"', text, re.MULTILINE)
    assert match, f"{field} is not a quoted key in cosmos.lock"
    return match.group(1)


def _get_cosmos(
    *args: str, home: Path | None = None, lock: Path | None = None
) -> subprocess.CompletedProcess[str]:
    env = dict(os.environ)
    env.pop("COSMOS_HOME", None)
    env.pop("COSMOS_LOCK", None)
    if home is not None:
        env["COSMOS_HOME"] = str(home)
    if lock is not None:
        env["COSMOS_LOCK"] = str(lock)
    return subprocess.run(
        ["bash", str(GET), *args],
        cwd=REPO,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-c", "safe.directory=*", *args],
        cwd=repo if repo.is_dir() else repo.parent,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def _make_release(dir_: Path, message: str) -> str:
    """A tiny COSMOS-shaped tree (the layout v7.5.1 verified against) as a commit."""
    (dir_ / "openc3").mkdir(parents=True, exist_ok=True)
    (dir_ / "openc3.sh").write_text("#!/bin/sh\n", encoding="utf-8")
    (dir_ / "compose.yaml").write_text("services: {}\n", encoding="utf-8")
    (dir_ / "openc3" / "Gemfile").write_text("gem 'openc3'\n", encoding="utf-8")
    (dir_ / ".env").write_text("OPENC3_TAG=latest\n", encoding="utf-8")
    _git(dir_, "add", "-A")
    _git(dir_, "commit", "-qm", message)
    return _git(dir_, "rev-parse", "HEAD")


@pytest.fixture()
def scratch_cosmos(tmp_path: Path) -> tuple[Path, Path]:
    """A fabricated (remote, lock) pair: the pin machinery, no GitHub involved."""
    remote = tmp_path / "remote"
    remote.mkdir()
    _git(remote, "init", "-q", "-b", "main")
    _git(remote, "config", "user.email", "t@t")
    _git(remote, "config", "user.name", "t")
    sha = _make_release(remote, "release")
    _git(remote, "tag", "v9.9.9")
    lock = tmp_path / "cosmos.lock"
    lock.write_text(
        "[cosmos]\n"
        f'repository = "file://{remote}"\n'
        'tag = "v9.9.9"\n'
        f'commit = "{sha}"\n'
        'image_tag = "9.9.9"\n'
        "renames_from = [\n"
        '    "file:///old-name-of-this-project",\n'
        "]\n\n"
        "[verify]\n"
        "required_paths = [\n"
        '    "openc3.sh",\n'
        '    ".env",\n'
        '    "compose.yaml",\n'
        '    "openc3/Gemfile",\n'
        "]\n",
        encoding="utf-8",
    )
    return remote, lock


# --- the lock file itself -----------------------------------------------------


def test_the_lock_names_a_peeled_commit_and_agrees_with_its_tag() -> None:
    """An annotated tag pins a tag *object*; a checkout lands on the commit."""
    values = {field: _lock_field(field) for field in FIELDS}
    assert re.fullmatch(r"[0-9a-f]{40}", values["commit"]), "a short pin is a moving pin"
    assert values["tag"].startswith("v") and values["image_tag"] == values["tag"].lstrip("v")
    assert values["repository"].startswith("https://github.com/"), (
        "the repository moved once already (COSMOS/OpenC3 -> openc3/openc3); pin the current home"
    )


def test_print_agrees_with_the_lock() -> None:
    """Two readers of the lock (tests, the script) must not drift apart."""
    result = _get_cosmos("--print")
    assert result.returncode == 0, result.stderr
    for field in FIELDS:
        value = _lock_field(field)
        if field == "checkout":
            # the lock stores a portable "~" default; --print shows where the
            # script will ACTUALLY look, which is the same path resolved
            value = value.replace("~", str(Path.home()), 1)
        assert f"{field:<10} = {value}" in result.stdout, f"--print contradicts the lock on {field}"


# --- the failures the pin exists to catch ------------------------------------


def test_an_absent_checkout_names_the_command_that_fixes_it(tmp_path: Path) -> None:
    """Absence is a common case, not an error: a fresh machine has no COSMOS."""
    result = _get_cosmos("--verify", home=tmp_path / "nowhere")
    assert result.returncode != 0
    assert "scripts/get_cosmos.sh" in result.stderr


def test_the_hybrid_stack_is_reported_as_one_full_diagnosis(tmp_path: Path) -> None:
    """Wrong commit AND 'latest' images, listed together, because they co-occur.

    This is the real deployment bug: a checkout old enough to describe different
    containers than the ones .env launches. A verifier that reports the commit
    and stops sends the operator back twice for one incident.
    """
    home = tmp_path / "hybrid"
    home.mkdir()
    _git(home, "init", "-q", "-b", "main")
    _git(home, "config", "user.email", "t@t")
    _git(home, "config", "user.name", "t")
    _make_release(home, "v6.4-era checkout")  # .env still says OPENC3_TAG=latest

    result = _get_cosmos("--verify", home=home)
    assert result.returncode != 0
    assert "checked out" in result.stderr and "the pin is" in result.stderr
    assert "OPENC3_TAG=latest" in result.stderr


def test_force_refuses_a_foreign_checkout_even_when_it_is_clean(tmp_path: Path) -> None:
    """Right filenames, wrong repository: a deployment is more than its layout."""
    home = tmp_path / "someone-elses"
    home.mkdir()
    _git(home, "init", "-q", "-b", "main")
    _git(home, "config", "user.email", "t@t")
    _git(home, "config", "user.name", "t")
    _make_release(home, "different repository, same filenames")

    result = _get_cosmos("--force", home=home)
    assert result.returncode != 0
    assert "not this pin's checkout" in result.stderr


# --- the paths that must work --------------------------------------------------


def test_fetch_clones_tags_the_env_and_verifies_idempotently(
    scratch_cosmos: tuple[Path, Path], tmp_path: Path
) -> None:
    """The whole newcomer path: clone, own the image tag, verify twice the same."""
    _remote, lock = scratch_cosmos
    home = tmp_path / "clone"

    first = _get_cosmos("fetch", home=home, lock=lock)
    assert first.returncode == 0, first.stderr + first.stdout
    assert _git(home, "rev-parse", "HEAD") == _lock_field_lock(lock, "commit")
    assert "OPENC3_TAG=9.9.9" in (home / ".env").read_text(encoding="utf-8"), (
        "the pin must own the image tag"
    )

    again = _get_cosmos("--verify", home=home, lock=lock)
    assert again.returncode == 0, again.stderr


def test_drift_of_the_image_tag_is_refused_and_force_repairs_only_that(
    scratch_cosmos: tuple[Path, Path], tmp_path: Path
) -> None:
    """A drift back to OPENC3_TAG=latest is the regression this pin exists for."""
    _remote, lock = scratch_cosmos
    home = tmp_path / "clone"
    assert _get_cosmos("fetch", home=home, lock=lock).returncode == 0

    (home / ".env").write_text(
        "OPENC3_TAG=latest\n", encoding="utf-8"
    )  # operator "upgrades" to latest
    assert _get_cosmos("--verify", home=home, lock=lock).returncode != 0

    repaired = _get_cosmos("--force", home=home, lock=lock)
    assert repaired.returncode == 0, repaired.stderr


def test_force_moves_a_right_origin_with_a_wrong_head(
    scratch_cosmos: tuple[Path, Path], tmp_path: Path
) -> None:
    """The move-back path: our checkout, drifted off the pin by someone's `git pull`."""
    remote, lock = scratch_cosmos
    home = tmp_path / "clone"
    assert _get_cosmos("fetch", home=home, lock=lock).returncode == 0

    # the pinned commit stays behind; the checkout runs ahead
    _git(home, "config", "user.email", "t@t")
    _git(home, "config", "user.name", "t")
    (home / "openc3.sh").write_text("#!/bin/sh\necho newer\n", encoding="utf-8")
    _git(home, "add", "-A")
    _git(home, "commit", "-qm", "ran ahead of the pin")
    assert _get_cosmos("--verify", home=home, lock=lock).returncode != 0

    moved = _get_cosmos("--force", home=home, lock=lock)
    assert moved.returncode == 0, moved.stderr
    assert _git(home, "rev-parse", "HEAD") == _lock_field_lock(lock, "commit")
    # and the remote itself was never the place to fix: its tag still names the pin
    assert _git(remote, "rev-parse", "v9.9.9") == _lock_field_lock(lock, "commit")


def test_force_accepts_a_recorded_rename_of_the_repository(
    scratch_cosmos: tuple[Path, Path], tmp_path: Path
) -> None:
    """An old clone of the RIGHT project predates its newest name: lineage, not foreign."""
    remote, lock = scratch_cosmos
    home = tmp_path / "old-name-clone"
    assert _get_cosmos("fetch", home=home, lock=lock).returncode == 0

    # cloned when the project answered to the old name, and drifted since
    _git(home, "remote", "set-url", "origin", "file:///old-name-of-this-project")
    _git(home, "config", "user.email", "t@t")
    _git(home, "config", "user.name", "t")
    (home / "openc3.sh").write_text("#!/bin/sh\necho newer\n", encoding="utf-8")
    _git(home, "add", "-A")
    _git(home, "commit", "-qm", "ran ahead under the old name")

    moved = _get_cosmos("--force", home=home, lock=lock)
    assert moved.returncode == 0, moved.stderr
    assert _git(home, "rev-parse", "HEAD") == _lock_field_lock(lock, "commit")
    assert _git(remote, "rev-parse", "v9.9.9") == _lock_field_lock(lock, "commit")


def test_force_still_refuses_an_origin_the_lock_has_never_seen(
    scratch_cosmos: tuple[Path, Path], tmp_path: Path
) -> None:
    """Accepting a lineage is not accepting anything: an unknown name stays a refusal."""
    _remote, lock = scratch_cosmos
    home = tmp_path / "who-are-you"
    assert _get_cosmos("fetch", home=home, lock=lock).returncode == 0

    _git(home, "remote", "set-url", "origin", "file:///never-recorded-anywhere")
    refused = _get_cosmos("--force", home=home, lock=lock)
    assert refused.returncode != 0
    assert "not this pin's checkout" in refused.stderr


def _lock_field_lock(lock: Path, field: str) -> str:
    match = re.search(rf'^{field}\s*=\s*"([^"]*)"', lock.read_text(encoding="utf-8"), re.MULTILINE)
    assert match
    return match.group(1)
