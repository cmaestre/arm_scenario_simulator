cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

include_directories(include
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)


add_library(DREAM_tablepocket_sensor SHARED ContactPlugin.cc)

target_link_libraries(DREAM_tablepocket_sensor
  ${GAZEBO_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${roscpp_LIBRARIES} 
  ${catkin_LIBRARIES})

add_dependencies(
  DREAM_tablepocket_sensor
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS DREAM_tablepocket_sensor
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
