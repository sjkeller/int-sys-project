#!/usr/bin/env sh
# generated from catkin.builder Python module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi
# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink:$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib:/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib:/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib/aarch64-linux-gnu:$DYLD_LIBRARY_PATH"
fi
export PATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/bin:$PATH"
export PKG_CONFIG_PATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib/pkgconfig:/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/devel_isolated/mavlink/lib/python3/dist-packages:$PYTHONPATH"
