#!/bin/bash
set -e

BIN="low_pass_mov_avg"   # must match add_executable target

if [ ! -f "build/$BIN" ]; then
  echo "❌ Executable not found. Run ./build.sh first."
  exit 1
fi

exec "build/$BIN"
