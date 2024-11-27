cmake_minimum_required(VERSION 3.8)
project(explore_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)  # Added
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for Eigen and OpenCV
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable for the exploration node
add_executable(explore_node src/explore_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(explore_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  visualization_msgs 
  tf2_ros 
  tf2_geometry_msgs
  nav2_msgs
  rclcpp_action  # Added
  Eigen3
  OpenCV
)

# Link OpenCV libraries
target_link_libraries(explore_node ${OpenCV_LIBS})

# Install the executable
install(TARGETS explore_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
