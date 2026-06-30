#!/bin/bash

# Build an image locally
set -eufo pipefail

PROJECT_ROOT="$(dirname $(dirname $(realpath $0)))"

CACHE_ARG=""
OPT_DEV=false
IMAGE_NAME="planning_service-production"
IMAGE_TAG="local-latest"
# When true, export the image to a .tar.gz into the working directory
OPT_EXPORT_IMAGE=false
# When true, do not build the production image, and only create the packaged assets
OPT_PKG_ONLY=false

SEMVER_TAG=""
usage="$(basename "$0") [--help] [OPTIONS] -- Convenience script to build the image
used by the planning service..

The production build workflow is a two-step process. First,
\`planning_service_build\` builds the Bazel executables and bundles them with their
required dependencies into a Debian package. Then, the production image is built by
creating a lightweight base image and installing the package into it.

The development image process is more straightforward, and just invokes
\`docker compose build\` directly.

where:
    --help            Show this help text.
    --dev             Build a dev image. Naming and export options are ignored.
                      [default: $OPT_DEV]
    --no-cache        Build the image from scratch (i.e., without using
                      previously cached layers).
    -n, --name NAME   Assigned image name. Prod only.  [default: $IMAGE_NAME]
    -t, --tag TAG     Assigned image tag. Prod only.   [default: $IMAGE_TAG]
    --pkg-only        Only create the distributable Debian package. Prod only.
    --export          Export the image to a .tar.gz. Prod only."

while (($#)); do
  case "$1" in
  --help)
    echo "$usage"
    exit
    ;;
  --dev)
    OPT_DEV=true
    shift 1
    ;;
  -n | --name)
    shift 1
    IMAGE_NAME="$1"
    shift 1
    ;;
  -t | --tag)
    shift 1
    IMAGE_TAG="$1"
    shift 1
    ;;
  --no-cache)
    CACHE_ARG="--no-cache"
    shift 1
    ;;
  --pkg-only)
    OPT_PKG_ONLY=true
    shift 1
    ;;
  --export)
    OPT_EXPORT_IMAGE=true
    shift 1
    ;;
  -* | --*=) # unsupported options
    echo "Error: Unsupported option $1" >&2
    exit 1
    ;;
  esac
done
USER_NAME="$(whoami)"
export USER_UID="$(id $USER_NAME -u)"
# dev build
if [[ $OPT_DEV == true ]]; then
  echo "***** BUILDING DEVELOPMENT IMAGE **************"
  docker compose -f docker-compose.dev.yml build ${CACHE_ARG}
# prod build is a multi-step procedure
else
  if [[ ! -f ${PROJECT_ROOT}/version.txt ]]; then
    echo "0.0.0" >${PROJECT_ROOT}/version.txt
  fi
  SEMVER_TAG=$(cat ${PROJECT_ROOT}/version.txt)
  echo "***** CONFIGURING BUILDER CONTAINER ***********"
  docker compose -f ${PROJECT_ROOT}/docker-compose.build.yml build ${CACHE_ARG} planning_service_build
  if [[ -f ${PROJECT_ROOT}/planning_service.deb ]]; then
    echo "    Removing stale package"
    rm -f ${PROJECT_ROOT}/planning_service.deb
  fi
  echo "***** BUILDING PRODUCTION ARTIFACTS ***********"
  docker compose -f ${PROJECT_ROOT}/docker-compose.build.yml run --rm planning_service_build
  if [[ $OPT_PKG_ONLY == false ]]; then
    IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"
    echo "***** BUILDING PRODUCTION IMAGE ***************"
    # always build the prod image without the cache
    docker build --build-arg SEMVER_TAG=${SEMVER_TAG} --no-cache --target ps_prod -t ${IMAGE} ${PROJECT_ROOT}
    echo "    Built image and tagged as ${IMAGE}"

    if [[ $OPT_EXPORT_IMAGE == true ]]; then
      echo "***** EXPORTING PRODUCTION IMAGE *************"
      EXPORT_NAME="${IMAGE_NAME}-${IMAGE_TAG}.tar.gz"
      docker save ${IMAGE} | gzip >${PROJECT_ROOT}/${EXPORT_NAME}
      echo "    Image saved to ${EXPORT_NAME}"
    fi
    # clean up any artifacts
    rm -rf ${PROJECT_ROOT}/dist
  fi
fi
echo "***** ALL BUILD TASKS COMPLETE! ****************"
