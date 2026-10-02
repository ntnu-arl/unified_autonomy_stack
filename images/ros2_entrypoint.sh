#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

# Independent workspaces can be mounted as read-only underlays. Source them in
# the declared order before the executable workspace so /workspace remains the
# final overlay. A colon separates setup files, matching PATH-style variables.
if [[ "${SKIP_WORKSPACE_SETUP:-0}" != "1" && -n "${UNDERLAY_WORKSPACE_SETUPS:-}" ]]; then
  IFS=':' read -r -a underlay_workspace_setups <<< "${UNDERLAY_WORKSPACE_SETUPS}"
  for workspace_setup in "${underlay_workspace_setups[@]}"; do
    if [[ ! -r "${workspace_setup}" ]]; then
      echo "ROS 2 underlay setup is not readable: ${workspace_setup}" >&2
      exit 1
    fi
    source "${workspace_setup}"
  done
fi

# Build containers deliberately skip any existing overlay. Runtime containers
# automatically discover packages built in their bind-mounted workspace.
if [[ "${SKIP_WORKSPACE_SETUP:-0}" != "1" && -r "${WORKSPACE}/install/setup.bash" ]]; then
  source "${WORKSPACE}/install/setup.bash"
fi

# Declarative overlays such as ws_agentic_bringup can be mounted read-only at a
# different path and layered after the executable workspace.
if [[ "${SKIP_WORKSPACE_SETUP:-0}" != "1" && -n "${EXTRA_WORKSPACE_SETUP:-}" ]]; then
  if [[ ! -r "${EXTRA_WORKSPACE_SETUP}" ]]; then
    echo "Additional ROS 2 setup is not readable: ${EXTRA_WORKSPACE_SETUP}" >&2
    exit 1
  fi
  source "${EXTRA_WORKSPACE_SETUP}"
fi

mkdir -p "${HOME}/.ros" "${HOME}/.gz"

exec "$@"
