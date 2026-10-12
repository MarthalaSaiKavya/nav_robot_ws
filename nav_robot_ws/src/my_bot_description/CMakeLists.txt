cmake_minimum_required(VERSION 3.8)
project(my_bot_description)

find_package(ament_cmake REQUIRED)

# This package only contains data files (xacro, yaml, launch, rviz).
# "Installing" them copies them into install/share/my_bot_description so
# that $(find my_bot_description) / get_package_share_directory() can find them.
install(
  DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
