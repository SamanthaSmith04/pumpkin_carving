cmake_minimum_required(VERSION 3.8)
project(aims_pumpkin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pumpkin_msgs REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(Eigen3 REQUIRED)

# Add library for plugin files
add_library(${PROJECT_NAME}_plugins
  src/plugins/kinematic_limits_check_profile.cpp
)

target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  include
  src/plugins
)

ament_target_dependencies(${PROJECT_NAME}_plugins
  tesseract_common
  tesseract_environment
  tesseract_command_language
  tesseract_task_composer
  tesseract_motion_planners
)

# Add executable
add_executable(motion_planning_server src/motion_planning_server.cpp)

add_executable(motion_planning_client src/test_client.cpp)

target_include_directories(motion_planning_server PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  include
)

target_link_libraries(motion_planning_server
  ${PROJECT_NAME}_plugins
)

ament_target_dependencies(motion_planning_server
  rclcpp
  pumpkin_msgs
  tesseract_common
  tesseract_environment
  tesseract_scene_graph
  tesseract_state_solver
  tesseract_kinematics
  tesseract_motion_planners
  tesseract_command_language
  tesseract_task_composer
  tesseract_rosutils
  tesseract_monitoring
  tesseract_collision
)

ament_target_dependencies(motion_planning_client
  rclcpp
  pumpkin_msgs
  geometry_msgs
)

# Install executable and library
install(TARGETS
  motion_planning_server
  motion_planning_client
  ${PROJECT_NAME}_plugins
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
