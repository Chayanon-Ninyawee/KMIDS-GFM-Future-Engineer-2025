#!/bin/bash
set -e  # Exit on error

# If argument is "clean", delete the build directory
if [ "$1" == "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf build
    exit 0
fi

TARGET="$1"

# Generate build system if not already present
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF

# Build all if no target is provided
if [ -z "$TARGET" ]; then
    echo "Building all targets..."
    cmake --build build -j$(nproc)
else
    echo "Building target: $TARGET"
    cmake --build build --target "$TARGET" -j$(nproc)
fi
