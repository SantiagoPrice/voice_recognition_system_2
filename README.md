Installing dependencies:
cd ros2_ws/src && /
rosdep init && /
rosdep update && /
rosdep install --from-path vint_ros --ignore-src -r -y 

Building package:
cd ..
colcon build --packages-select vint_ros


Model sources: https://alphacephei.com/vosk/models

Voice feedback:
Google Gemini is one option to create audio: https://aistudio.google.com/generate-speech

Compilation:

colcon build --symlink-install --packages-select vint_ros

Adding new nodes:

Create node in ${PROJECT_PATH}/vint_ros/vint_ros
Make it executable chmod +x node/path
Add node path in CMakeList.txt