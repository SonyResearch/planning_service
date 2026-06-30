#!/bin/bash

# Entrypoint for a build container which will create the necessary artifacts
# for a production deployment.
set -euo pipefail

# If dist or tmp exist, remove them
if [ -d dist ]; then
  echo "Removing existing dist directory..."
  rm -rf dist
fi
if [ -d tmp ]; then
  echo "Removing existing tmp directory..."
  rm -rf tmp
fi
# Build package target
PROJECT_ROOT="$(dirname $(dirname $(realpath $0)))"
pushd ${PROJECT_ROOT}
bazel build //:planning_service-debian
DEB_PATH=$(bazel cquery //:planning_service-debian --output=files)
mkdir dist
cp ${DEB_PATH} dist

# OMPL & Boost dependency
bazel build //:ompl_tar
OMPL_TAR_PATH=$(bazel cquery //:ompl_tar --output=files)
mkdir -p dist/ompl/include
mkdir -p dist/ompl/lib
# tmp directory for extraction
mkdir -p tmp
# Extract OMPL tarball
tar -xzf ${OMPL_TAR_PATH} -C tmp
# Copy includes
cp -r tmp/ompl/include/* dist/ompl/include
# Copy libraries
find tmp/ompl -type f -name "*.so*" -exec cp {} dist/ompl/lib \;
# Remove the tarball and tmp directory
rm -rf ${OMPL_TAR_PATH} tmp
