#!/bin/bash

# Exit on error
set -e

# Clean and configure
rm -rf build/
mkdir build
cd build

# Configure and build
cmake ..
make -j$(nproc)

echo "✅ Build complete. Run with ./run.sh"