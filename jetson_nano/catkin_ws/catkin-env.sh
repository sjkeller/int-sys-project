#!/usr/bin/env bash
. /opt/ros/noetic/setup.bash
[ -d devel ] || catkin_make || exit 1
. devel/setup.bash
