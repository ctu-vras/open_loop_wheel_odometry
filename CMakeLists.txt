cmake_minimum_required(VERSION 3.7.2)
project(open_loop_wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  diff_drive_controller
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  topic_tools
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp topic_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} PRIVATE Eigen3::Eigen)

add_executable(diff_drive_odom src/diff_drive_odom.cpp)
add_dependencies(diff_drive_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive_odom ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS diff_drive_odom ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment, so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_ROOT})
    roslint_custom(catkin_lint "-W2" .)
  endif()

  add_rostest(test/test_diff_drive.test)
endif()
