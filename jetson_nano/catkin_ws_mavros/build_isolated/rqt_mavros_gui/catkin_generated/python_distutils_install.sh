#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/src/rqt_mavros_gui"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/install_isolated/lib/python3/dist-packages:/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/build_isolated/rqt_mavros_gui/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/build_isolated/rqt_mavros_gui" \
    "/usr/bin/python3" \
    "/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/src/rqt_mavros_gui/setup.py" \
     \
    build --build-base "/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/build_isolated/rqt_mavros_gui" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/install_isolated" --install-scripts="/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/install_isolated/bin"
