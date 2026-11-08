#!/usr/bin/env bash
# Proves the Unreal integration recipe actually compiles.
#
# FLS.hpp and FLS.cpp are engine-free, and the CI gate keeps them that way.
# But the recipe the header carries in its comments is Unreal code, and prose
# that claims to compile is worth nothing. This script extracts that recipe
# and builds it as a module in a THROWAWAY Unreal project that has nothing to
# do with this repository or the one it came from.
#
#   UE_ROOT=/path/to/UE_5.3 bash tools/unreal_build_check.sh [work_dir]
#
# Not part of the GitHub Actions matrix: the runners have no Unreal Engine.
# Run it on a machine that does, whenever the recipe changes.
set -euo pipefail
KOK="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
UE="${UE_ROOT:-$HOME/Desktop/UE_5.3}"
B="${UE}/Engine/Build/BatchFiles/Linux/Build.sh"
[ -x "$B" ] || { echo "UE_ROOT is wrong: $B does not exist" >&2; exit 2; }

T="${1:-${TMPDIR:-/tmp}/FlsUnrealCheck}"
rm -rf "$T"; mkdir -p "$T/Source/FlsUnrealCheck"
M="$T/Source/FlsUnrealCheck"

cp "$KOK/FLS.hpp" "$KOK/FLS.cpp" "$M/"
bash "$KOK/tools/extract_unreal_files.sh" "$M"

cat > "$T/FlsUnrealCheck.uproject" <<'J'
{ "FileVersion": 3, "EngineAssociation": "", "Category": "", "Description": "",
  "Modules": [ { "Name": "FlsUnrealCheck", "Type": "Runtime", "LoadingPhase": "Default" } ] }
J
cat > "$T/Source/FlsUnrealCheck.Target.cs" <<'C'
using UnrealBuildTool;
public class FlsUnrealCheckTarget : TargetRules {
    public FlsUnrealCheckTarget(TargetInfo Target) : base(Target) {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V4;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_3;
        ExtraModuleNames.Add("FlsUnrealCheck");
    }
}
C
sed 's/TargetType.Game/TargetType.Editor/; s/FlsUnrealCheckTarget/FlsUnrealCheckEditorTarget/g' \
    "$T/Source/FlsUnrealCheck.Target.cs" > "$T/Source/FlsUnrealCheckEditor.Target.cs"

# STEP 1 of the recipe says no build file change is needed. This Build.cs
# therefore lists NOTHING beyond what a default empty Unreal module has: if the
# recipe secretly needed another module, this build would fail.
cat > "$M/FlsUnrealCheck.Build.cs" <<'C'
using UnrealBuildTool;
public class FlsUnrealCheck : ModuleRules {
    public FlsUnrealCheck(ReadOnlyTargetRules Target) : base(Target) {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
    }
}
C
printf '#include "Modules/ModuleManager.h"\nIMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, FlsUnrealCheck, "FlsUnrealCheck");\n' \
    > "$M/FlsUnrealCheck.cpp"

bash "$B" FlsUnrealCheckEditor Linux Development "$T/FlsUnrealCheck.uproject" \
     -WaitMutex -NoHotReloadFromIDE
echo
echo "UNREAL BUILD CHECK PASSED"
echo "  the recipe in FLS.hpp compiled as an Unreal module in $T"
echo "  Build.cs added nothing beyond Core/CoreUObject/Engine, as STEP 1 claims"
