#!/bin/bash
set -e  # Exit on error

# If argument is "clean", delete the build directory
if [ "$1" == "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf build
    exit 0
fi

# Generate build system if not already present
cmake -B build_native -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF

# Build all if no target is provided
cmake --build build_native --target log_viewer -j$(nproc)
