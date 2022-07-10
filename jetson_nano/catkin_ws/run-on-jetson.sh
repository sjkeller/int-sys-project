#!/bin/bash
DIR=`dirname "${BASH_SOURCE[0]}"`
JETSON_SSH="jetson"
SYNC_DIR="~/.catkin_ws_rsync/"
rsync -a --delete \
	--exclude="build" \
	--exclude="devel" \
	--exclude=".catkin_workspace" \
	"$DIR"/ $JETSON_SSH:$SYNC_DIR

ssh -t -X $JETSON_SSH "cd ${SYNC_DIR} && . catkin-env.sh && ${@}"
