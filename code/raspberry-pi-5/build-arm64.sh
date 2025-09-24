#!/bin/bash
set -e

# Resolve the directory of this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Temporary copy of shared folder for Docker build
cp -r "$SCRIPT_DIR/../shared" "$SCRIPT_DIR/src/shared_real"

# Build the cross-compile Docker image
docker buildx build -f "$SCRIPT_DIR/Dockerfile.cross" --tag cross-pi .

# Build the project using the compile Dockerfile and output binaries to ./bin
docker buildx build -f "$SCRIPT_DIR/Dockerfile.compile" -o type=local,dest="$SCRIPT_DIR/build-arm64" .

# Clean up the temporary shared folder
rm -rf "$SCRIPT_DIR/src/shared_real"

echo "Build complete! Binaries are in $SCRIPT_DIR/bin"
