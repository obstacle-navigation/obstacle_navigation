#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/fri/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel', type 'exit' to leave"
  . "/home/fri/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/fri/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel'"
else
  . "/home/fri/ros/rosbuild_ws/class-code/obstacle-navigation/build/devel/setup.sh"
  exec "$@"
fi
