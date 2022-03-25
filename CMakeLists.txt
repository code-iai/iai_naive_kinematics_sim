cmake_minimum_required(VERSION 3.0.2)
project(iai_naive_kinematics_sim)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  urdf
  sensor_msgs
  std_msgs)

add_message_files(DIRECTORY msg
  FILES
  ProjectionClock.msg)

add_service_files(DIRECTORY srv
  FILES
  SetJointState.srv
  UpdateTransform.srv)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_generation message_runtime urdf sensor_msgs std_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(simulator 
  src/${PROJECT_NAME}/simulator_main.cpp)
add_dependencies(simulator 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator
  ${catkin_LIBRARIES})

set(TEST_SRCS
  test/${PROJECT_NAME}/main.cpp
  test/${PROJECT_NAME}/simulator.cpp
  test/${PROJECT_NAME}/watchdog.cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

catkin_install_python(PROGRAMS 
  script/fake_base_controller.py
  script/map_odom_transform_publisher.py
  script/navp_faker_tf.py
  script/wsg_50_faker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
