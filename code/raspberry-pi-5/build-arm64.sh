#!/bin/bash
# set -e  <-- Removed to prevent script from exiting on error

# Resolve the directory of this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Temporary copy of shared folder for Docker build
cp -r "$SCRIPT_DIR/../shared" "$SCRIPT_DIR/src/shared_real"

# Build the cross-compile Docker image
docker buildx build -f "$SCRIPT_DIR/Dockerfile.cross" --tag cross-pi . || echo "WARNING: Docker build for cross-pi failed. Continuing..."

if [ -d "$SCRIPT_DIR/build" ]; then
    rm -r "$SCRIPT_DIR/build"
fi

# Build the project using the compile Dockerfile and output binaries to ./bin
docker buildx build -f "$SCRIPT_DIR/Dockerfile.compile" -o type=local,dest="$SCRIPT_DIR/build" . || echo "WARNING: Docker build for compile failed. Continuing..."

# Clean up the temporary shared folder
rm -rf "$SCRIPT_DIR/src/shared_real"

# --- Post-build steps ---
# We run these commands in a subshell with 'set -e'
# This way, if the first 'jq' command fails (e.g., file not found),
# it won't try to run the next ones.
# The '||' at the end catches any failure from this whole block.
(
    set -e # Exit on error *inside* this block only
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
    > "$SCRIPT_DIR"/build/compile_commands.json.tmp \
    && mv "$SCRIPT_DIR"/build/compile_commands.json.tmp "$SCRIPT_DIR/build/compile_commands.json"

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
) || echo "WARNING: Post-build steps failed (likely because build output was missing)."

echo "Script finished."
