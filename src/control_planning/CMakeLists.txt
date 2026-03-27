cmake_minimum_required(VERSION 3.8)
project(control_planning)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -- Find ROS 2 dependencies -------------------------------------------------
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs       REQUIRED)

# -- Build the kart_controller executable ------------------------------------
add_executable(kart_controller src/kart_controller.cpp)

target_include_directories(kart_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(kart_controller PUBLIC cxx_std_17)

ament_target_dependencies(kart_controller
  rclcpp
  geometry_msgs
  ackermann_msgs
  std_msgs
)

# -- Install ------------------------------------------------------------------
install(TARGETS kart_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# -- Tests --------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
