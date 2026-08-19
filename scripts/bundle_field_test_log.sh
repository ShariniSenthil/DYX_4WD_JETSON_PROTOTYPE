#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-}"
ULG_FILE="${2:-}"

if [ -z "$RUN_DIR" ] || [ ! -d "$RUN_DIR" ]; then
  echo "Usage: $0 <field_run_directory> [PX4_LOG.ulg]"
  exit 2
fi

RUN_DIR="$(cd "$RUN_DIR" && pwd)"
if [ -n "$ULG_FILE" ]; then
  if [ ! -f "$ULG_FILE" ]; then
    echo "PX4 ULog not found: $ULG_FILE"
    exit 3
  fi
  cp -f "$ULG_FILE" "$RUN_DIR/"
fi

PARENT="$(dirname "$RUN_DIR")"
NAME="$(basename "$RUN_DIR")"
OUT="$PARENT/${NAME}_BUNDLE.tar.gz"

tar -C "$PARENT" -czf "$OUT" "$NAME"
sha256sum "$OUT" > "$OUT.sha256"

echo "Created: $OUT"
echo "Checksum: $OUT.sha256"
echo "Upload the *_BUNDLE.tar.gz file to ChatGPT for analysis."