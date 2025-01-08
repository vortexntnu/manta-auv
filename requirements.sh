#!/bin/bash
# requirements.sh
# This script installs additional dependencies that are not managed by rosdep.
# It can install both Python libraries (using pip) and C++ libraries (using apt-get).

# Exit immediately if any command fails to prevent the script from continuing after an error.
set -e

echo "Starting manual installation of extra dependencies..."

# ---- PYTHON DEPENDENCIES ----
# Upgrade pip to ensure compatibility with the latest packages
pip3 install --upgrade pip

# Install Python packages with specified versions that are not handled by rosdep.
# Specify versions using `==<version-number>` for consistent builds.
# Example: `pip3 install <package-name>==<version>`
pip3 install control==0.10.1
pip3 install "numpy<1.25.0"

echo "Finished installing extra dependencies."
