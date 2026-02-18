#!/usr/bin/bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This script must be sourced."
  echo "Run: source setup_env.sh"
  exit 1
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

echo "RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID      = $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY"