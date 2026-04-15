cmake_minimum_required(VERSION 3.8)
project(vint_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)



# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install console scripts
install(PROGRAMS
  vint_ros/dummy.py
  vint_ros/spin_node.py
  vint_ros/vosk_node.py
  vint_ros/voice_assist_ROS_woGUI.py
  vint_ros/Assesment_node.py
  vint_ros/Assesment_text_node.py
  vint_ros/Perception_node.py
  vint_ros/Edge_Labeling_node.py
  vint_ros/Task_Manager_node.py
  vint_ros/Goal_to_Nav2_node.py
  vint_ros/phase1_trig_vint.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install configuration files
install(
  DIRECTORY
    conf
    others
  DESTINATION share/${PROJECT_NAME}
)




# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)





ament_package()
