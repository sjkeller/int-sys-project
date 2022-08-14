#!/usr/bin/env bash
if [ ! -f /usr/local/cuda-10.2/include/curand.h ]
then
	sudo apt install -y \
		cuda-command-line-tools-10-2 \
		cuda-compiler-10-2 \
		cuda-toolkit-10-2 \
		libcurand-dev-10-2 \
		libcublas-dev \
		libcudnn8-dev \
		gcc-7 \
		g++-7
fi
. /opt/ros/noetic/setup.bash
[ -d devel ] || ./release-build.sh || exit 1
. devel/setup.bash
