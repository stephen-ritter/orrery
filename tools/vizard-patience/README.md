# Rebuilding Vizard without Unity (the dotnet path)

This directory rebuilds the Vizard player's managed assembly from the upstream
`v2.4.0` sources **without a Unity install** and swaps in a startup-patience
patch. It is how orrery ships a renderer that survives a world whose first
message arrives late, without waiting on an upstream release.

## Why this is possible

Vizard ships as a **Mono** player: the engine and native libraries are fixed,
and the entire C# surface — every product script — compiles into a single
assembly, `Vizard_Data/Managed/Assembly-CSharp.dll`. Rebuilding that one file
and dropping it into the unpack *is* a full rebuild for anything under
`Assets/Scripts`. No Unity, no license. This does not touch scenes, assets, or
packages — those still need a real Unity build.

## One command

```bash
scripts/get_vizard.sh              # fetch + verify the vizard.lock unpack first
scripts/build_vizard_patience.sh   # clone v2.4.0, apply patch, rebuild, install
```

The script clones `AVSLab/vizard` at the tag `v2.4.0` into `.vizard-build/`,
applies `patience.patch` to a clean tree, builds `Assembly-CSharp.dll` against
the unpack's own `Managed/` assemblies, and installs it into the unpack **only
if the built SHA-256 equals `vizard.lock`'s** `patched_assembly_sha256`. If the
hash disagrees, nothing installs — the pin and the patch are then provably at
odds. To go back to stock, re-run `scripts/get_vizard.sh` after deleting the
unpack.

To build against a local Vizard worktree instead of a fresh clone (e.g. while
drafting the patch), invoke the project directly:

```bash
dotnet build tools/vizard-patience/Assembly-CSharp.csproj -c Release \
  -p:VizardAssets=/path/to/vizard/VizardUnityProject/Assets \
  -p:ManagedDir=.../Vizard_Linux/Vizard_Data/Managed \
  -o out/
```

## How the project file works (`Assembly-CSharp.csproj`)

An ordinary `Microsoft.NET.Sdk` library pointed at Unity's world:

- **`net48`, reference assemblies only.** Targets the .NET Framework profile,
  but never links an SDK runtime pack — Unity's Mono supplies it at runtime.
- **`DisableImplicitFrameworkReferences`.** Keeps the SDK from pulling its own
  BCL. The `UsePlayerBcl` target then references the player's own
  `Managed/*.dll` as the assembly set, so the compile sees the exact BCL and
  engine DLLs the app ships with. That is what makes the output *droppable*.
  `Assembly-CSharp*.dll` is excluded — we are the one producing it.
- **Source glob** is `VizardAssets/**/*.cs`, minus `Editor/` and `Tests/`
  (those never ship in the player) and minus the XR `StarterAssets` sample.
- **No `DefineConstants`.** The shipped build defines `UNITY_EDITOR` /
  `VIZARD_OPENXR` / `VIZARD_MRTK_VR` / `UNITY_STANDALONE_OSX` off, so we
  compile them out and they drop from the output (verified by decompile, below).

### The one define that differs from shipping: `USE_NATIVE_FILE_BROWSER`

The **shipped** DLL is compiled with `USE_NATIVE_FILE_BROWSER` **on** — the
`Crosstales.FB` native file-dialog package linked at Unity build time from a
UPM package that is *not in the git repo* (only its About-panel logo is). We
compile it **off**: the package is not in the source tree, so turning it on
would not compile.

This is harmless for orrery's use, for two independent reasons:

- The patch is mounted only in **headless socket mode** (`-noDisplay` /
  `-opNavMode`), where a file dialog is never opened and the `Crosstales`
  types are never JIT-loaded (confirmed: the headless `Player.log` never
  touches `FileBrowser`).
- Even interactively on **Linux** the shipping code routes the dialog to the
  native `VizardFileBrowser`; the `Crosstales` branch is Windows/macOS-only.
  So on Linux a `USE_NATIVE_FILE_BROWSER=off` build behaves the same as
  shipping for file dialogs.

The only residual difference is a cosmetic Awake-instantiation of a
`Crosstales` `FileBrowser` prefab that is inert on Linux. **Do not ship this
assembly as a replacement for interactive Windows/macOS file-playback** — it is
an orrery Linux-headless substitute, not a full rebrand of the player.

## Proving what the patch changed (fidelity gate)

Because a rebuild uses a different compiler than Unity did, "it compiles" is not
enough. `ilspycmd` decompiles the three assemblies — the **shipped** DLL, the
**baseline** (unpatched v2.4.0 through this project), and the **patched** build
— and the diffs answer two separate questions:

```bash
dotnet tool install -g ilspycmd --version 9.0.0.7889
for pair in "official …/Managed/Assembly-CSharp.dll" \
            "baseline out-baseline/Assembly-CSharp.dll" \
            "patched   out-patched/Assembly-CSharp.dll"; do
  set -- $pair; ilspycmd -p -o "decomp/$1" "$2"
done
```

- **baseline vs patched** — must differ in exactly the three patched files
  (`StartUpScreenManager`, `DirectCommunicationController`, `ResSocket`) and no
  render/scene/message type. Measured: exactly those three.
- **patched vs official** — the render-path files differ from the shipped DLL
  only as much as the *baseline* does (pure Roslyn-version artifacts: inlined
  string concatenation, hoisted locals — identical semantics). Measured: the
  only file-diffs that are new versus baseline-vs-official are the two
  connection files we touched. This is the image-parity proof: the patch
  introduces zero render-path change against the shipping product.

## Smoke test before it goes near the yard

The patience behaviour is exercised headless against a stand-in Basilisk world
whose first `SIM_UPDATE` is deliberately late (`logs/vizard-smoke/`):

- patient default (`-noDisplay`): waits past the old 0.5 s window, greets,
  loads the scene.
- `-messageTimeout 3`: reopens the socket on a rhythm, greets on the next cycle
  after arrival, loads the scene.
- stock control: prints `Timed out`, stays alive but deaf, world never greeted.

## What ships where

| file | role |
| --- | --- |
| `patience.patch` | the Vizard branch diff, kept reviewable; `git apply --check` gates it against the tag |
| `Assembly-CSharp.csproj` | the license-free build harness |
| `scripts/build_vizard_patience.sh` | one-command rebuild + install, gated on `vizard.lock` |
| `vizard.lock` | `version = "2.4.0+orrery-patience1"` + `patched_assembly_sha256` |
