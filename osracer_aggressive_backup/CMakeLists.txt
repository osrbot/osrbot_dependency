cmake_minimum_required(VERSION 3.8)
project(osracer_aggressive_backup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/aggressive_back_up.cpp
)

ament_target_dependencies(${library_name}
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2_geometry_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(nav2_core osracer_aggressive_backup.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES osracer_aggressive_backup.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  CHANGELOG.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  tf2_geometry_msgs
  sensor_msgs
)

ament_package()
