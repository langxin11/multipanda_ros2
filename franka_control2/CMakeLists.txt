cmake_minimum_required(VERSION 3.8)
project(franka_control2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(cartesian_impedance_demo src/cartesian_impedance_demo.cpp)
target_compile_features(cartesian_impedance_demo PRIVATE cxx_std_17)
target_link_libraries(cartesian_impedance_demo Franka::Franka Eigen3::Eigen Threads::Threads)

add_executable(franka_read_only_node src/franka_read_only_node.cpp)
target_compile_features(franka_read_only_node PRIVATE cxx_std_17)
ament_target_dependencies(franka_read_only_node rclcpp sensor_msgs)
target_link_libraries(franka_read_only_node Franka::Franka)

add_executable(franka_control2_node src/franka_control2_node.cpp)
ament_target_dependencies(franka_control2_node ament_index_cpp)
  
target_include_directories(franka_control2_node
  PRIVATE
  include
)

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

install(
  TARGETS franka_control2_node franka_read_only_node cartesian_impedance_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
