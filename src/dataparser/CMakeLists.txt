cmake_minimum_required(VERSION 3.14)

project(data_parser LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

find_package(Boost COMPONENTS filesystem system program_options REQUIRED)
include_directories(${Boost_DIRECTORIES})
link_libraries(${Boost_LIBRARIES})

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)

set(DETECTOR_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rclcpp_components
  Eigen3
  eigen3_cmake_module
  tram_vehicle_msgs
)

file(GLOB_RECURSE HEADERS "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")

file(GLOB_RECURSE TEST_HEADERS "tests/*.cpp")
file(GLOB_RECURSE TEST_SOURCES "tests/*.hpp")

set(PROJECT_SOURCES
    ${HEADERS}
    ${SOURCES}
)

include_directories(include)
include_directories(tests)

add_library(data_parser SHARED ${PROJECT_SOURCES})
ament_target_dependencies(data_parser ${DETECTOR_DEPS})
target_link_libraries(data_parser Eigen3::Eigen ${Boost_LIBRARIES} ${catkin_LIBRARIES})
rclcpp_components_register_node(data_parser PLUGIN "CDataParser" EXECUTABLE data_parser_node)

install(TARGETS data_parser
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
