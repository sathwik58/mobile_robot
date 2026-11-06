cmake_minimum_required(VERSION 3.8)
project(amr_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/scan_merger.py scripts/map_outline.py scripts/lane_coverage.py scripts/amr_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
