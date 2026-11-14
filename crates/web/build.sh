#!/usr/bin/env bash
# Build the wasm package with wasm-pack.
#
# `--mode no-install` uses the wasm-bindgen CLI already on PATH instead of
# downloading a matching release from GitHub. It is the wasm-pack 0.15 spelling
# of the older `--no-install` flag. `--no-opt` does the same for wasm-opt
# (remove it if you have binaryen installed and want a smaller file).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"

wasm-pack build crates/web \
  --target web \
  --mode no-install \
  --no-opt \
  --out-name fem_web \
  --release

echo
echo "已生成 crates/web/pkg/fem_web.js 与 .wasm"
echo "启动静态服务器："
echo "  python3 -m http.server 8000 --directory crates/web"
echo "然后打开 http://localhost:8000/"
