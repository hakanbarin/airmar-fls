#!/usr/bin/env bash
# Extracts the Unreal adapter files that FLS.hpp carries inside its comments.
#
# The integration recipe lives in the header because the deliverable is two
# files. That makes it comment text, and comment text rots. This script turns
# it back into real files so it can be COMPILED, which is what
# tools/unreal_build_check.sh then does.
#
#   bash tools/extract_unreal_files.sh <output_dir>
set -euo pipefail
KOK="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:?usage: extract_unreal_files.sh <output_dir>}"
mkdir -p "$OUT"
COUNT=0
while read -r NAME; do
    # Strip the leading "// " (and handle blank comment lines written as "//").
    awk -v f="$NAME" '
        $0 == "// === BEGIN FILE " f " ===" { on=1; next }
        $0 == "// === END FILE " f " ==="   { on=0 }
        on { sub(/^\/\/ ?/, "", $0); print }
    ' "$KOK/FLS.hpp" > "$OUT/$NAME"
    [ -s "$OUT/$NAME" ] || { echo "extraction produced an empty $NAME" >&2; exit 1; }
    echo "extracted $NAME ($(wc -l < "$OUT/$NAME") lines)"
    COUNT=$((COUNT + 1))
done < <(grep -oP '^// === BEGIN FILE \K[^ ]+' "$KOK/FLS.hpp")
[ "$COUNT" -gt 0 ] || { echo "no FILE blocks found in FLS.hpp" >&2; exit 1; }
