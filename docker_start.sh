#!/bin/bash

# Common start executed in docker for the three supported services that comprise `planning_service`.
set -eufo pipefail

SERVICE=""
CMD_PREFIX=""
BAZEL_OUTPUT_BASE=""
OPT_DEV=false
while (($#)); do
  case "$1" in
  --iris)
    SERVICE=iris_builder
    shift 1
    ;;
  --planner)
    SERVICE=motion_planner
    shift 1
    ;;
  --viz)
    SERVICE=visualizer
    shift 1
    ;;
  --dev)
    OPT_DEV=true
    shift 1
    ;;
  -* | --*=) # unsupported options
    echo "Error: Unsupported option $1" >&2
    exit 1
    ;;
  esac
done

if [[ -z ${SERVICE-} ]]; then
  echo "Please specify a service to initiate"
  exit 1
fi

# Argument specifying all resources must be loaded correctly
REQUIRED_ARG=""
# Set the command depending on whether dev or production mode is specified
if [[ $OPT_DEV == true ]]; then
  BAZEL_OUTPUT_BASE="/root/.cache/bazel/output_base_${SERVICE}"
  CMD="bazel --output_base=${BAZEL_OUTPUT_BASE} run //planning_service/app:${SERVICE} --"
else
  CMD="/usr/bin/${SERVICE}"
  REQUIRED_ARG="-r"
fi
CMD_ARGS=" --system_name ${SYSTEM_NAME} -v ${VERBOSITY:-3}"
# Artifact Builder
if [[ "${SERVICE}" = "iris_builder" ]]; then
  CMD_ARGS+=" -i ${DMD_INCLUDE_NAMES:-"urdf"}"
  CMD_ARGS+=" ${REQUIRED_ARG} --max_jobs ${IRIS_MAX_JOBS:-20} --queue_capacity ${IRIS_QUEUE_CAPACITY:-20}"
# Motion Planner
elif [[ "${SERVICE}" = "motion_planner" ]]; then
  CMD_ARGS+=" ${REQUIRED_ARG}"
  CMD_ARGS+=" -i ${DMD_INCLUDE_NAMES:-"urdf"}"
  # Check if LOAD_ON_INIT is set to true, if so, add the --load_on_init argument
  if [[ "${LOAD_ON_INIT:-false}" == "true" ]]; then
    CMD_ARGS+=" --load_on_init"
  fi
# Visualizer
elif [[ "${SERVICE}" = "visualizer" ]]; then
  if [[ "${SET_CONFIGURATION:-}" == "true" ]]; then
    CMD_ARGS+=" --set-configuration"
  elif [[ "${SET_CONFIGURATION:-}" == "false" ]]; then
    CMD_ARGS+=" --no-set-configuration"
  fi
  if [[ "${SLIDERS:-}" == "true" ]]; then
    CMD_ARGS+=" --sliders"
  elif [[ "${SLIDERS:-}" == "false" ]]; then
    CMD_ARGS+=" --no-sliders"
  fi
  if [[ "${DEFAULT_INIT:-}" == "true" ]]; then
    CMD_ARGS+=" --default-init"
  elif [[ "${DEFAULT_INIT:-}" == "false" ]]; then
    CMD_ARGS+=" --no-default-init"
  fi
fi
CMD+=${CMD_ARGS}

exec $CMD
