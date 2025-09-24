#!/bin/bash
set -e

# Resolve the directory of this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Temporary copy of shared folder for Docker build
cp -r "$SCRIPT_DIR/../shared" "$SCRIPT_DIR/src/shared_real"

# Build the cross-compile Docker image
docker buildx build -f "$SCRIPT_DIR/Dockerfile.cross" --tag cross-pi .

# Build the project using the compile Dockerfile and output binaries to ./bin
docker buildx build -f "$SCRIPT_DIR/Dockerfile.compile" -o type=local,dest="$SCRIPT_DIR/build" .

# Clean up the temporary shared folder
rm -rf "$SCRIPT_DIR/src/shared_real"

jq --arg dir "$SCRIPT_DIR" \
  '
  map(
    .directory |= gsub("^/code"; $dir) |
    .file |= gsub("^/code"; $dir) |
    .command |= gsub("/code"; $dir)
  )
  ' \
   "$SCRIPT_DIR"/build/compile_commands.json > "$SCRIPT_DIR"/build/compile_commands.json.tmp \
   && mv "$SCRIPT_DIR"/build/compile_commands.json.tmp "$SCRIPT_DIR"/build/compile_commands.json

jq --arg sysroot "$SCRIPT_DIR/build/aarch64-linux-gnu/include" '
  map(
    .command |= gsub("/usr/aarch64-linux-gnu/include/"; ($sysroot + "/"))
  )
' "$SCRIPT_DIR/build/compile_commands.json" \
  > "$SCRIPT_DIR/build/compile_commands.json.tmp" \
  && mv "$SCRIPT_DIR/build/compile_commands.json.tmp" "$SCRIPT_DIR/build/compile_commands.json"

# Generate .clangd with full absolute paths to cross headers
SYSROOT="$SCRIPT_DIR/build/aarch64-linux-gnu/include"

cat > "$SCRIPT_DIR/.clangd" <<EOF
CompileFlags:
  Add:
    [
      -isystem,
      $SYSROOT/c++/12,
      -isystem,
      $SYSROOT/c++/12/aarch64-linux-gnu,
      -isystem,
      $SYSROOT
    ]
EOF
