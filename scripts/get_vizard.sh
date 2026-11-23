#!/usr/bin/env bash
# Put the pinned renderer in the tree. The sibling of scripts/fetch_cfs.sh:
# the pin is committed, the source is not, and this script is the ONLY thing
# that fills in .vizard/ (which .gitignore keeps out of the tree).
#
#   scripts/get_vizard.sh              fetch the locked URL
#   scripts/get_vizard.sh <zip>        verify THIS zip instead (the canonical
#                                      server updates in place; a hand-fetched
#                                      copy with the same bytes is a supported
#                                      path through the same gate)
#
# Either way the sha256 must equal vizard.lock's, or nothing unpacks: the
# renderer that draws the yard's frames is the one the yard measured, not one
# that "probably matches". A moved zip is a renderer decision - re-measure
# (the camera exhibit is the measuring) and edit vizard.lock, deliberately.
#
# Idempotent: a verified unpack is a no-op. To re-verify, delete
# .vizard/.verified-* and re-run.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

url="$(sed -n 's/^url = "\([^"]*\)".*/\1/p' vizard.lock)"
want="$(sed -n 's/^sha256_zip = "\([^"]*\)".*/\1/p' vizard.lock)"
unpack="$(sed -n 's/^unpack = "\([^"]*\)".*/\1/p' vizard.lock)"

if [ -f "$unpack/.verified-$want" ]; then
    echo "vizard: $unpack already verified against vizard.lock ($want)"
    exit 0
fi

zip="${1:-}"
if [ -z "$zip" ]; then
    zip="$(mktemp -d)/Vizard_Linux.zip"
    echo "vizard: fetching $url"
    curl -fL --retry 3 -o "$zip" "$url"
fi

got="$(sha256sum "$zip" | cut -d' ' -f1)"
if [ "$got" != "$want" ]; then
    echo "vizard: sha256 MISMATCH - refusing to unpack" >&2
    echo "  vizard.lock wants $want" >&2
    echo  "           the zip says $got" >&2
    echo "  a moved zip is a renderer decision: re-measure, then edit vizard.lock." >&2
    exit 1
fi

mkdir -p "$unpack"
unzip -q -o "$zip" -d "$unpack"
chmod +x "$unpack/Vizard_Linux/Vizard.x86_64"
touch "$unpack/.verified-$want"
echo "vizard: $unpack/Vizard_Linux is the $want renderer"
