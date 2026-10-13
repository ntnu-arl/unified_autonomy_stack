#!/usr/bin/env bash
# Build the pinned decoder locally; dataset files are never opened for writing.
set -euo pipefail
stack_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tools_root="$stack_root/generated/hm3d_tools"
source_dir="$tools_root/basis_universal"
expected_commit=323239a6a5ffa57d6570cfc403be99156e33a8b0
mkdir -p "$tools_root"
if [[ ! -d "$source_dir" ]]; then
    git clone --depth 1 --branch v1_60 https://github.com/BinomialLLC/basis_universal.git "$source_dir"
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$expected_commit" ]]; then
    echo "Existing decoder checkout does not match pinned v1_60; preserving it." >&2
    exit 1
fi
cmake -S "$source_dir" -B "$source_dir/build" -DCMAKE_BUILD_TYPE=Release -DBASISU_SSE=OFF -DBASISU_ZSTD=OFF
cmake --build "$source_dir/build" --target basisu --parallel 4
printf '\nDecoder: %s\n' "$source_dir/bin/basisu"
