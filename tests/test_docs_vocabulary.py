"""The handbook's vocabulary: pages are written for their reader, not for the plan.

Phase numbers, decision ids and round names are how this repository keeps its own
schedule. In a reader's hands they are noise — and a second place to keep honest,
which is how a document starts lying about the code. These rules are cheap and
mechanical so the pages stay readable as they are edited by people who were not
here for the decisions.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
DOCS = REPO / "docs"
HANDBOOK = sorted(DOCS.glob("*.md"))
PAGES = [*HANDBOOK, REPO / "README.md"]

# Private vocabulary: phase numbers, decision ids, and the short names of working
# rounds. Uppercase on purpose: `check_beacon_tables.sh` is a script a reader runs,
# `Pn` is a name from the project's own plan.
PRIVATE = re.compile(r"\b(?:D\d+|P\d[a-c]?|KN2?|REG|OB(?:\([ab]\))?|CAM|ASM|R\d)\b")
LINK = re.compile(r"\]\((\w+\.md)\)")


def _offenders(text: str, pattern: re.Pattern[str]) -> list[str]:
    return [
        f"  {number}: {line.strip()[:96]}"
        for number, line in enumerate(text.splitlines(), start=1)
        if pattern.search(line)
    ]


@pytest.mark.parametrize("page", PAGES, ids=lambda page: page.name)
def test_no_page_writes_the_projects_own_vocabulary(page: Path) -> None:
    """A page should read as though the design were always this way.

    To its reader, it was.

    The project's private plan is deliberately absent from this list: it is not
    a page a reader gets, and the codes belong nowhere else.
    """
    offenders = _offenders(page.read_text(encoding="utf-8"), PRIVATE)
    assert not offenders, f"{page.relative_to(REPO)} still speaks the working plan:\n" + "\n".join(
        offenders
    )


@pytest.mark.parametrize("page", HANDBOOK, ids=lambda page: page.name)
def test_no_handbook_page_sends_the_reader_to_the_private_plan(page: Path) -> None:
    """A procedure that needs the project's plan to be understood is unfinished.

    `README.md` may name the file, because it lists the repository's files and the
    file exists. A page that tells someone how to do something may not lean on it:
    two places to keep honest is how one of them starts lying.
    """
    text = page.read_text(encoding="utf-8")
    assert "ROADMAP" not in text, f"{page.relative_to(REPO)} points its reader at the private plan"


def test_links_between_pages_resolve() -> None:
    """A dead link is the same defect, smaller: a page that does not lead where it says."""
    for page in PAGES:
        for target in LINK.findall(page.read_text(encoding="utf-8")):
            assert (page.parent / target).exists(), f"{page.name} links to missing {target}"


def test_every_ledger_owner_has_a_label_a_reader_can_act_on() -> None:
    """An owner is a promise about the future, so it has to be in words.

    The ledger stores the working plan's own code for each gap (that is what the
    plan indexes by) and the renderer translates it. An untranslated owner fails
    here rather than printing a code a reader cannot look up.
    """
    from sim.coverage import OWNER_LABELS, load_coverage

    owners = {entry.owner for entry in load_coverage().types} - {""}
    untranslated = sorted(owners - set(OWNER_LABELS))
    assert not untranslated, f"owners with no reader-facing label: {untranslated}"


def test_the_generated_page_is_current_and_its_refresh_works() -> None:
    """`docs/coverage.md` must match `registry/coverage.toml`, right now.

    Run through the module's own command, because that command is the documented
    way to refresh the page: a stale page and a broken refresh are the same bug,
    and this catches either half.
    """
    result = subprocess.run(  # fixed argv: this repo's own module
        [sys.executable, "-m", "sim.coverage", "--check"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_no_page_promises_a_ci_that_does_not_exist() -> None:
    """There is no hosted CI in this repository, so no page may lean on one.

    `scripts/verify.sh` is the substrate that actually runs these checks. Naming a
    service nobody configured is how a guarantee quietly becomes folklore — the
    same mistake as a private-plan reference, in the other direction.
    """
    offenders = [
        f"{page.name}:{number}"
        for page in PAGES
        for number, line in enumerate(page.read_text(encoding="utf-8").splitlines(), start=1)
        if re.search(r"\bCI\b", line)
    ]
    assert not offenders, f"pages that cite CI instead of scripts/verify.sh: {offenders}"


# Working-plan shorthand, uppercase on purpose: a reader who meets `D5` in a
# comment meets a term with no definition, and every code in the tree is one
# more ledger entry to keep true. The plan (and `logs/`) is where they belong.
CODES = re.compile(
    r"\b(?:D\d+|P\d[a-c]?(-\d)?|KN2?|REG|OB|R\d)\b|Phase-\d|\bv2[abc]\b|findings ledger"
)
# The guard names its own quarry, so it must not quote it either.
SELF = "tests/test_docs_vocabulary.py"


def test_the_source_tree_speaks_mission_language() -> None:
    """Comments, strings and data speak for the mission, not for the schedule.

    The whole tree is a reader's documentation: a phase code in a comment is a
    sentence that only makes sense to someone who was here.
    """
    listing = subprocess.run(
        ["git", "ls-files"], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout.splitlines()
    offenders = []
    for name in listing:
        path = REPO / name
        if (
            name == SELF
            or name.startswith(("cfs_mission/cFS/", "tools/", ".gitignore"))
            or not path.is_file()
        ):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for number, line in enumerate(text.splitlines(), start=1):
            if CODES.search(line) or "ROAD" + "MAP" in line:
                offenders.append(f"  {name}:{number}: {line.strip()[:96]}")
    assert not offenders, "the tree still quotes the working plan:\n" + "\n".join(offenders[:20])
