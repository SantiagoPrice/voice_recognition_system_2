cmake_minimum_required(VERSION 3.8)
project(vint_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


set(VOSK_DIR "" CACHE PATH "Path to model")


# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install console scripts
install(PROGRAMS
  vint_ros/dummy.py
  vint_ros/spin_node.py
  vint_ros/vosk_node.py
  vint_ros/voice_assist_ROS_woGUI.py
  vint_ros/Assesment_node.py
  vint_ros/Perception_node.py
  vint_ros/Edge_Labeling_node.py
  vint_ros/Task_Manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install configuration files
install(DIRECTORY conf
  DESTINATION share/${PROJECT_NAME}
)


if(VOSK_DIR)
  # Install launch files
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Install Vosk model directory
  install(DIRECTORY ${VOSK_DIR}
    DESTINATION share/${PROJECT_NAME}
  )
endif()



ament_package()
