export DISPLAY=:0  # something in moveit2 needs this
export MAKEFLAGS="-j1"
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclone-config.xml
#source /opt/ros/rolling/setup.bash
source $HOME/bender/ros2/install/setup.bash
