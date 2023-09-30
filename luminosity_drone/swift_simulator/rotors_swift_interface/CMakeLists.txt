cmake_minimum_required(VERSION 2.8.3)
project(rotors_swift_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  trajectory_msgs
  swift_msgs

)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs trajectory_msgs swift_msgs
  LIBRARIES
)

add_definitions(-std=c++11)

add_executable(rotors_swift_interface src/swift_trans.cpp)
add_dependencies(rotors_swift_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_swift_interface ${catkin_LIBRARIES})

install(TARGETS rotors_swift_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
