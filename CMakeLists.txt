cmake_minimum_required(VERSION 3.8)
project(qrb_ros_amr VERSION 1.0.4)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/opt/ros/jazzy/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(qrb_ros_amr_msgs REQUIRED)
find_package(qrb_ros_navigation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(qrb_ros_robot_base_msgs REQUIRED)
find_package(qrb_ros_slam_msgs REQUIRED)
find_package(qrb_amr_manager REQUIRED)
find_package(lifecycle_msgs REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${qrb_amr_manager_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/topic
    ${PROJECT_SOURCE_DIR}/include/service
    ${PROJECT_SOURCE_DIR}/include/action
    ${PROJECT_SOURCE_DIR}/include/amr_manager/
    ${EIGEN3_INCLUDE_DIRS}
    )
include_directories(${JSON_INC_PATH})

set(source
  src/main.cpp
  src/amr_controller.cpp
  src/topic/amr_status_transporter.cpp
  src/topic/amr_developer_mode_subscriber.cpp
  src/topic/amr_exception_subscriber.cpp
  src/service/api_service_server.cpp
  src/service/sub_cmd_service_server.cpp
  src/service/cartographer_service_client.cpp
  src/service/amr_mode_service_client.cpp
  src/service/node_manager_service_client.cpp
  src/service/mapping_service_server.cpp
  src/action/auto_mapper_action_client.cpp
  src/action/cmd_action_server.cpp
  src/action/navigation_action_client.cpp
)


aux_source_directory(./src SRC_LIST)

ament_auto_add_executable(qrb_ros_amr ${SRC_LIST} ${source})

ament_target_dependencies(qrb_ros_amr
  "qrb_ros_amr_msgs"
  "qrb_ros_navigation_msgs"
  "nav_msgs"
  "nav2_msgs"
  "nav_2d_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "rcutils"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "sensor_msgs"
  "qrb_ros_robot_base_msgs"
  "qrb_ros_slam_msgs"
  "qrb_amr_manager"
  "lifecycle_msgs"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(TARGETS qrb_ros_amr
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE launch)
