#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/nishome/eysalee/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel', type 'exit' to leave"
  . "/nishome/eysalee/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/nishome/eysalee/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel'"
else
  . "/nishome/eysalee/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel/setup.sh"
  exec "$@"
fi
