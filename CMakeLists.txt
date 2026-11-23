cmake_minimum_required(VERSION 3.10.2)
project(umrt-ros-poe-cam VERSION 2.9.0 LANGUAGES CXX C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 14)

# TO-DO: Test without the following, will build but unsure if it affects anything
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# add_compile_options(-g)

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
        depthai_v3
        depthai_ros_msgs_v3
        depthai_bridge_v3
        foxglove_msgs
        sensor_msgs
        stereo_msgs
        std_msgs
        vision_msgs
        foxglove_compressed_video_transport
        image_transport
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
        find_package(${Dependency} REQUIRED)
endforeach()

# ==============================================================================
# TARGET 1: Foxglove Republisher Node
# ==============================================================================

set(republisher_node_target republisher_node)

add_executable(${republisher_node_target}
        src/foxglove_republisher_node.cpp 
        src/foxglove_republisher.cpp
)

target_compile_features(${republisher_node_target} PUBLIC cxx_std_17)

target_include_directories(${republisher_node_target} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${republisher_node_target} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# ==============================================================================
# TARGET 2: MobileNet Publisher Node
# ==============================================================================

set(mobilenet_node_target mobilenet_node)

add_executable(${mobilenet_node_target}
        src/mobilenet_publisher.cpp
        src/mobilenet_publisher_node.cpp
)

target_compile_features(${mobilenet_node_target} PUBLIC cxx_std_17)

target_include_directories(${mobilenet_node_target} BEFORE PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ${depthai_bridge_v3_INCLUDE_DIRS}
        ${depthai_ros_msgs_v3_INCLUDE_DIRS}
        ${depthai_v3_INCLUDE_DIRS}
)

target_compile_definitions(${mobilenet_node_target} PRIVATE 
        BLOB_NAME="${mobilenet_blob_name}"
)

target_link_libraries(${mobilenet_node_target} PRIVATE
        depthai_v3::core
        opencv_imgproc
        opencv_highgui
)

ament_target_dependencies(${mobilenet_node_target} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS} 
)

# ==============================================================================
# Install & Export
# ==============================================================================

# Parse version number from the file
file(READ "version" version_input)
string(REGEX MATCH "([0-9]+)\.([0-9]+)\.([0-9]+)" _ ${version_input})

# Fill version number into package.xml
configure_file( 
        package.in.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        @ONLY
)

# Install parameters, resources, and launch files
install(DIRECTORY 
        params
        resources
        launch
        DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
        republisher_node
        mobilenet_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
