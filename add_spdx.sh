#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# Add SPDX headers to source files

find . -maxdepth 1 -type f \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.S' \) \
  -exec grep -L "SPDX-License-Identifier" {} \; | while read -r f; do
  echo "Adding SPDX to $f"
  printf '%s\n%s\n\n%s' "// SPDX-License-Identifier: AGPL-3.0-or-later" "// Copyright (c) 2023-2025 Modular Battery Technologies, Inc." "$(cat "$f")" > "$f.tmp" && mv "$f.tmp" "$f"
done

echo "SPDX headers added to CellCPU source files"
