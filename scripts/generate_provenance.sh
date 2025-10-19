#!/usr/bin/env bash
set -euo pipefail
OUT=docs/PROVENANCE.md
echo "" >> "$OUT"
echo "### SHA256 Checksums" >> "$OUT"
echo "" >> "$OUT"
echo '```' >> "$OUT"
# Hash text-like artifacts; skip binaries & the git dir
find . -type f \
  -not -path "./.git/*" \
  -not -name "*.zip" \
  -not -name "*.png" -not -name "*.jpg" -not -name "*.jpeg" -not -name "*.gif" \
  -not -name "*.pdf" \
  -print0 | sort -z | xargs -0 sha256sum >> "$OUT"
echo '```' >> "$OUT"
echo "Updated $OUT"
