"""The store's index is invisible, conservative, and spent at delivery.

The catalog (/cf/img/.catalog) is the recorder's memory across restarts:
path, size, creation, and whether a receipt proved the ground holds that
path. Three laws hold it honest, pinned here without a toolchain because
they are laws of text and order that a compiler would never catch:

* the catalog is a DOT NAME under the store directory: the boot walk and
  the ground's inventory both skip dot entries, so the index can never
  list itself;
* boot treats every catalog line as a claim to verify, never a fact -
  the walk confirms the file stands before a receipt carries over;
* the queue law's order of operations: a receipt is persisted BEFORE the
  recycle runs, so a restart between the two moments cannot un-know the
  receipt and strand a proven frame.

And the ground never sees a vehicle-side claim: the inventory CSV keeps
three columns; the receipt column lives only in the internal catalog.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
APP_H = REPO / "cfs_mission" / "mission" / "apps" / "bsk_link" / "fsw" / "src" / "bsk_link_app.h"
LINK_C = REPO / "cfs_mission" / "mission" / "apps" / "bsk_link" / "fsw" / "src" / "bsk_link.c"
CAM_C = REPO / "cfs_mission" / "mission" / "apps" / "cam" / "fsw" / "src" / "cam_app.c"


def test_the_catalog_hides_behind_a_dot_name() -> None:
    """An index the walks can see is a file that inventories itself."""
    header = APP_H.read_text()
    match = re.search(r'#define\s+BSK_LINK_STORE_CATALOG\s+BSK_LINK_STORE_DIR\s+"([^"]+)"', header)
    assert match, "the catalog path no longer derives from the store directory"
    name = match.group(1)
    assert name.startswith("/."), f"catalog name {name} is visible to the walks"
    part = re.search(
        r'#define\s+BSK_LINK_STORE_CATALOG_PART\s+BSK_LINK_STORE_DIR\s+"([^"]+)"', header
    )
    assert part and part.group(1).startswith("/."), "the staging file must hide too"


def test_both_walks_skip_store_internals() -> None:
    """Boot seed and the ground's inventory share one blindness: dot names."""
    guard = "Name[0] == '\\0' || Name[0] == '.'"
    assert LINK_C.read_text().count(guard) >= 1, "the boot walk would index the catalog"
    assert CAM_C.read_text().count(guard) >= 1, "the inventory would list the catalog"


def test_the_receipt_is_written_before_the_delete() -> None:
    """Queue law ordering: persist the receipt, then spend it."""
    text = LINK_C.read_text()
    start = text.index("static void BSK_Link_WitnessEot")
    body = text[start : text.index("\nstatic ", start + 10)]
    wrote = body.find("BSK_Link_CatalogWrite()")
    evicted = body.find("BSK_Link_StoreEvictOne()")
    assert wrote != -1 and evicted != -1, "the EOT witness no longer persists or spends receipts"
    assert wrote < evicted, "a receipt deleted before it was written can survive nothing"


def test_the_inventory_speaks_no_receipts() -> None:
    """The ground sees files; the proven column is a vehicle-side claim."""
    assert '"path,size_bytes,created_utc\\n"' in CAM_C.read_text()
    assert '"path,size_bytes,created_utc,proven\\n"' in LINK_C.read_text()
