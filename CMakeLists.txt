cmake_minimum_required(VERSION 3.10)
project(placo_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization system)
find_package(pinocchio REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(keisan REQUIRED)

# INCLUDE PATH UNTUK PLACO
set(PLACO_INCLUDE_DIR /home/mbsaloka/Ichiro/walk/placo/src)
set(PLACO_LIB /usr/local/lib/liblibplaco.so)

include_directories(
  include
  ${PLACO_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)

add_executable(placo_cpp_node src/placo_cpp_node.cpp)

target_link_libraries(placo_cpp_node
  ${PLACO_LIB}
  ${pinocchio_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

ament_target_dependencies(
  placo_cpp_node
  rclcpp
  tachimawari_interfaces
  jitsuyo
  keisan
)

install(TARGETS placo_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
