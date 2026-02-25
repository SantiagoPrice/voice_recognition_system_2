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

Activate python environment

colcon build --symlink-install --packages-select vint_ros --cmake-args -DPYTHON_EXECUTABLE=$(which python) -DVOSK_DIR=../../src/VOSK_PATH
