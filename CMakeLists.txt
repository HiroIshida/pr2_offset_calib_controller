cmake_minimum_required(VERSION 3.0.2)
project(pr2_offset_calib_controller)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
   std_msgs 
   pr2_controller_interface 
   roscpp 
   pr2_mechanism_model 
   realtime_tools 
   robot_mechanism_controllers 
   pr2_mechanism_controllers 
   pluginlib)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS 
    std_msgs 
    pr2_controller_interface 
    roscpp
    pr2_mechanism_model
    realtime_tools
    robot_mechanism_controllers
    pr2_mechanism_controllers
    pluginlib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/offset_calib_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
