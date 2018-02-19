cmake_minimum_required(VERSION 2.8.3)
project(photoneo_virtual_robot_module)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    pho_robot_loader
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES photoneo_virtual_robot_module
  CATKIN_DEPENDS pho_diagnostics pho_robot_loader
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#target_link_libraries(
#  ${catkin_LIBRARIES})

add_library(virtual_robot_plugin src/virtual_robot_plugin.cpp)
target_link_libraries(virtual_robot_plugin ${catkin_LIBRARIES})
add_dependencies(virtual_robot_plugin ${catkin_EXPORTED_TARGETS})

## trajectory statistic
add_executable(trajectory_statistics src/trajectory_statistics.cpp)
target_link_libraries(trajectory_statistics ${catkin_LIBRARIES})
add_dependencies(trajectory_statistics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS virtual_robot_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES virtual_robot_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
