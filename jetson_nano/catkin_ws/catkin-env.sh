#!/usr/bin/env bash
if [ ! -f /usr/local/cuda-10.0/include/curand.h ]
then
	sudo apt install -y \
		cuda-command-line-tools-10-0 \
		cuda-compiler-10-0 \
		cuda-curand-dev-10-0 \
		cuda-cublas-dev-10-0 \
		libcudnn7-dev \
		gcc-7 \
		g++-7
fi
. /opt/ros/noetic/setup.bash
[ -d devel ] || ./release-build.sh || exit 1
. devel/setup.bash
