#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "yaml file://$DIR/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/80-vive-tracking.list