cmake_minimum_required(VERSION 3.8)
project(reserve_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_executable(reserve_odometry_node src/reserve_odometry_node.cpp)
ament_target_dependencies(reserve_odometry_node
  rclcpp sensor_msgs nav_msgs message_filters geometry_msgs tram_vehicle_msgs 
)
target_link_libraries(reserve_odometry_node Eigen3::Eigen ${Boost_LIBRARIES})

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

install(TARGETS
  reserve_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

ament_package()
