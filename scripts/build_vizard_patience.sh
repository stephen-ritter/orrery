#!/usr/bin/env bash
# The patience build, from source, in one command.
#
#   scripts/build_vizard_patience.sh
#
# vizard.lock pins the official Vizard zip; orrery additionally runs the
# STARTUP-PATIENCE build: the same v2.4.0 sources with tools/vizard-patience/
# patience.patch (the upstream MR pending). Vizard ships as a Mono player, so
# its whole C# surface is one assembly: rebuilding Assembly-CSharp.dll with
# plain dotnet - against the unpack's own Managed/ DLLs, which are the runtime
# the result executes on - and installing that one file IS the patched player.
# No Unity, no license.
#
# The lock is the law: the built assembly's sha256 must equal the lock's, or
# nothing installs. Reproduce that sha by landing on the exact tag: the script
# clones AVSLab/vizard at the tag below and applies the patch to a clean tree.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="v2.4.0"
SRC_URL="https://github.com/AVSLab/vizard.git"
BUILD_DIR="$REPO/.vizard-build"
SRC="$BUILD_DIR/vizard"
UNPACK="$REPO/.vizard/Vizard_Linux"
DLL="$UNPACK/Vizard_Data/Managed/Assembly-CSharp.dll"

command -v dotnet >/dev/null || { echo "needs the .NET SDK (dotnet)" >&2; exit 2; }
[ -d "$UNPACK" ] || { echo "no vizard.lock unpack here - run scripts/get_vizard.sh first" >&2; exit 2; }
want="$(sed -n 's/^patched_assembly_sha256 = "\([^"]*\)".*/\1/p' "$REPO/vizard.lock")"
[ -n "$want" ] || { echo "vizard.lock declares no patched_assembly_sha256" >&2; exit 2; }

mkdir -p "$BUILD_DIR"
if [ -d "$SRC" ]; then
    git -C "$SRC" fetch --depth 1 origin "refs/tags/$TAG" >/dev/null
    git -C "$SRC" checkout -q FETCH_HEAD
    git -C "$SRC" clean -qfd
else
    git clone --depth 1 --branch "$TAG" "$SRC_URL" "$SRC" >/dev/null
fi
git -C "$SRC" apply --check "$REPO/tools/vizard-patience/patience.patch" || {
    echo "patience.patch does not apply to $TAG - the pin and the patch disagree" >&2; exit 1;
}
git -C "$SRC" apply "$REPO/tools/vizard-patience/patience.patch"

echo "--- building Assembly-CSharp ($TAG + patience) ---"
dotnet build -v q --nologo "$REPO/tools/vizard-patience/Assembly-CSharp.csproj" -c Release \
    -p:VizardAssets="$SRC/VizardUnityProject/Assets" \
    -p:ManagedDir="$UNPACK/Vizard_Data/Managed" \
    -o "$BUILD_DIR/bin"

got="$(sha256sum "$BUILD_DIR/bin/Assembly-CSharp.dll" | cut -d' ' -f1)"
if [ "$got" != "$want" ]; then
    echo "built $got, vizard.lock wants $want - refusing to install" >&2
    exit 1
fi
cp "$BUILD_DIR/bin/Assembly-CSharp.dll" "$DLL"
echo "installed $got -> Managed/Assembly-CSharp.dll ($(grep -o '2\.4\.0+[a-z0-9]*' "$REPO/vizard.lock" | head -1))"
