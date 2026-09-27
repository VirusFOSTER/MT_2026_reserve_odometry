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
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  "/usr/share/cmake/geographiclib"
)

find_package(GeographicLib REQUIRED)

add_executable(reserve_odometry_node src/reserve_odometry_node.cpp src/tram_speed_ekf.cpp)
ament_target_dependencies(reserve_odometry_node
  rclcpp sensor_msgs nav_msgs message_filters geometry_msgs tram_vehicle_msgs nlohmann_json GeographicLib)

target_link_libraries(reserve_odometry_node
  nlohmann_json::nlohmann_json
  ${GeographicLib_LIBRARIES}
  Eigen3::Eigen
)

add_executable(metrics_evaluator_node
  src/metrics_evaluator_node.cpp
)

ament_target_dependencies(metrics_evaluator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tram_vehicle_msgs
  message_filters
)

target_include_directories(metrics_evaluator_node PRIVATE
  ${GeographicLib_INCLUDE_DIRS}
)

target_link_libraries(metrics_evaluator_node
  ${GeographicLib_LIBRARIES}
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

install(TARGETS
  reserve_odometry_node
  metrics_evaluator_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

ament_package()
