#!/bin/bash

# Save the current directory
current_dir=$(pwd)

# Navigate to the project directory
# cd /home/kurosh/repos/mavlink_example || { echo "Failed to change directory"; exit 1; }

# Remove the build directory if it exists
rm -rf build

# Run cmake to configure the project
cmake -Bbuild -H. || { echo "CMake configuration failed"; exit 1; }

# Build the project using the specified number of processors
nprocs=$(nproc)
cmake --build build -j"$nprocs" || { echo "Build failed"; exit 1; }

echo "Build completed successfully"

# Return to the previous directory
cd "$current_dir" || { echo "Failed to return to the previous directory"; exit 1; }

