cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

set(library_name safety_monitor)
set(lib_srcs src/monitor.cpp src/timeFreq.cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

include_directories(include)

add_library(${library_name} ${lib_srcs})

ament_target_dependencies(${library_name} rclcpp std_msgs sensor_msgs lart_msgs nav_msgs ackermann_msgs)


add_executable(monitor_node src/monitor.cpp src/timeFreq.cpp)

ament_target_dependencies(
  monitor_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "lart_msgs"
  "nav_msgs"
  "ackermann_msgs"
)

target_link_libraries(monitor_node ${library_name})

install(TARGETS
  monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ../config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ../launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
