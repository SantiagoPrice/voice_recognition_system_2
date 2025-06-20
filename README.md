Installing dependencies:
cd ros2_ws/src
rosdep init
rosdep update
rosdep install --from-path vint_ros --ignore-src -r -y 

Building package:
cd ..
colcon build --packages-select vint_ros
