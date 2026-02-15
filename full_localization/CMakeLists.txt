cmake_minimum_required(VERSION 3.8)
project(full_localization)

# 1. Compiler Settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# 3. Libraries
# SLAM Library
add_library(slam_lib src/ICP.cpp)
target_include_directories(slam_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slam_lib PUBLIC Eigen3::Eigen)
ament_target_dependencies(slam_lib PUBLIC rclcpp sensor_msgs geometry_msgs)

# State Estimator Library
add_library(state_estimator_lib src/EKF.cpp)
target_include_directories(state_estimator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(state_estimator_lib PUBLIC Eigen3::Eigen)
ament_target_dependencies(state_estimator_lib PUBLIC rclcpp nav_msgs geometry_msgs)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. Executable
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node PUBLIC
  slam_lib
  state_estimator_lib
)
ament_target_dependencies(localization_node PUBLIC
  rclcpp std_msgs sensor_msgs nav_msgs tf2_ros geometry_msgs
)

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node PUBLIC slam_lib state_estimator_lib)
ament_target_dependencies(slam_node PUBLIC 
  rclcpp std_msgs sensor_msgs nav_msgs tf2_ros geometry_msgs
)

# 5. Installation
install(TARGETS 
  localization_node 
  slam_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS slam_lib state_estimator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# 6. Ament Package (CRITICAL: These lines generate the bash files you were missing)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This tells ROS 2 how to let other packages link to your libs
ament_export_include_directories(include)
ament_export_libraries(slam_lib state_estimator_lib)
ament_export_dependencies(rclcpp Eigen3 sensor_msgs nav_msgs geometry_msgs)

ament_package()