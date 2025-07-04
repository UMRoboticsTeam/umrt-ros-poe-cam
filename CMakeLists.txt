cmake_minimum_required(VERSION 3.10.2)
project(umrt-ros-poe-cam VERSION 2.9.0 LANGUAGES CXX C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
      cmake_policy(SET CMP0057 NEW)
endif()

# FIXED - For CLion to find headers
# Fixes intellisense not finding ros2 headers https://youtrack.jetbrains.com/issue/CPP-29747/Certain-ROS2-package-headers-missing-from-Intellisense-when-using-a-Docker-toolchain
include_directories(SYSTEM /opt/ros/$ENV{ROS_DISTRO}/include)

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
    set(_opencv_version 3)
    find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

# find_package(depthai CONFIG REQUIRED PATHS "/home/sachin/Desktop/luxonis/depthai-core/build/install/lib/cmake/depthai")
set(mobilenet_blob_name "mobilenet-ssd_openvino_2021.2_6shave.blob")

set(mobilenet_blob "${PROJECT_SOURCE_DIR}/resources/${mobilenet_blob_name}")
file(DOWNLOAD "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_6shave.blob"
        ${mobilenet_blob}
        TIMEOUT 60  # seconds
        EXPECTED_HASH SHA1=f0e14978b3f77a4f93b9f969cd39e58bb7aef490
        TLS_VERIFY ON)

find_package(ament_cmake REQUIRED)

message(STATUS "------------------------------------------")
message(STATUS "Depthai Bridge is being built using AMENT.")
message(STATUS "------------------------------------------")

set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_lifecycle
        camera_info_manager
        cv_bridge
        depthai
        depthai_ros_msgs
        depthai_bridge
        foxglove_msgs
        sensor_msgs
        stereo_msgs
        std_msgs
        vision_msgs
        )

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
        find_package(${Dependency} REQUIRED)
endforeach()

# find_package(camera_info_manager REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(depthai CONFIG REQUIRED)
# find_package(depthai_ros_msgs REQUIRED)
# find_package(depthai_bridge REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(stereo_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(vision_msgs REQUIRED)
# find_package(foxglove_msgs REQUIRED)

# set(dependencies
#         camera_info_manager
#         cv_bridge
#         depthai_ros_msgs
#         depthai_bridge
#         rclcpp
#         sensor_msgs
#         stereo_msgs
#         std_msgs
#         vision_msgs
#         foxglove_msgs
# )

include_directories(
        include
        ${ament_INCLUDE_DIRS}
)

# 
configure_file( Fill version number into package.xml
        package.in.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        @ONLY
)



# For Nodes

# Macro to add nodes
macro(dai_add_node_ros2 node_name node_src)
  add_executable("${node_name}" "${node_src}")

  target_link_libraries("${node_name}"
          depthai::core
          opencv_imgproc
          opencv_highgui)

  ament_target_dependencies("${node_name}"
          ${THIS_PACKAGE_INCLUDE_DEPENDS})

endmacro()

# Add MobileNet Publisher Node
dai_add_node_ros2(mobilenet_node src/mobilenet_publisher.cpp)

target_compile_definitions(mobilenet_node PRIVATE BLOB_NAME="${mobilenet_blob_name}")

# Install parameters, resources, and launch files
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")

# Install targets
install(TARGETS
        mobilenet_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
