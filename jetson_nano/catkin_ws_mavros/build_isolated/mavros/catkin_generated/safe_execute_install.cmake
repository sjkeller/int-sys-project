execute_process(COMMAND "/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/build_isolated/mavros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nano/gitlab_repo/jetson_nano/catkin_ws_mavros/build_isolated/mavros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
