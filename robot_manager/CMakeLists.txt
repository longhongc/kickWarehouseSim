cmake_minimum_required(VERSION 3.5)
project(robot_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs)
find_package(geometry_msgs)
find_package(visualization_msgs)
find_package(robot_manager_msgs)

include_directories(
  include)

add_library(nav_action_client_lib
  src/nav_action_client.cpp)

ament_target_dependencies(nav_action_client_lib
  nav2_msgs
  geometry_msgs
  visualization_msgs
  robot_manager_msgs
  rclcpp
  rclcpp_action)

add_library(robot_manager_lib
  src/robot_manager.cpp
  src/nav_action_client.cpp)

ament_target_dependencies(robot_manager_lib
  nav2_msgs
  geometry_msgs
  visualization_msgs
  robot_manager_msgs
  rclcpp
  rclcpp_action)

add_executable(robot_manager
  src/main.cpp)

target_link_libraries(robot_manager
  robot_manager_lib)

target_include_directories(robot_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



install(TARGETS
  # nav_action_client
  nav_action_client_lib
  robot_manager_lib
  robot_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/test_robot_manager.cpp
    test/test_nav_action_client.cpp)

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})

  target_link_libraries(${PROJECT_NAME}_test
    robot_manager_lib)

  ament_target_dependencies(${PROJECT_NAME}_test
    nav2_msgs
    geometry_msgs
    visualization_msgs
    robot_manager_msgs
    rclcpp
    rclcpp_action)

  install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME})

endif()

ament_package()
