/*
 * Copyright 2024 Edcel Abanto, University of Manitoba Robotics Team
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Created on 2026-29-05 by ea.
 */

#ifndef UMRT_ROS_POE_CAM__MOBILENET_PUBLISHER_HPP
#define UMRT_ROS_POE_CAM__MOBILENET_PUBLISHER_HPP

#include <cstdio>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"

/**
 * @class MobileNetPublisherNode
 * @brief Manages a Luxonis OAK-D PoE camera pipeline.
 *
 * @details This node interfaces with the DepthAI-V3 API to construct and run a hardware-accelerated 
 * encoded video stream, stereo depth, and IMU pipeline on an OAK-D PoE camera.
 *
 */
class MobileNetPublisherNode : public rclcpp::Node
{

public:

    /**
     * Initializes a MobileNetPublisherNode.
     */
    MobileNetPublisherNode();

private:

    /**
     * A minimum amount of values that both the IMU and Depth have to receive before starting to publish ROS2 data
     */
    static constexpr int MAX_WARMUP_COUNT = 50;

    /**
     * The report rate the IMU will output, meaning it will output every 5 milliseconds. 
     */
    static constexpr int IMU_REPORT_RATE_Hz = 200; 

    /**
     * The Color Camera FPS output 
     */
    static constexpr int CAMERA_FPS = 15;

    /**
     * The Depth Camera FPS output 
     */
    static constexpr int DEPTH_FPS = 15;

    /**
     * Setup the Camera pipeline, specifically the H.264 encoded frames,
     * and configures the Camera
     *
     * @param pipeline the pipeline network the node belongs to
     */
    void SetupCameraPipeline(dai::Pipeline& pipeline);

    /**
     * Setup the IMU pipeline, which also has configures the IMU
     * 
     * @param pipeline the pipeline network the node belongs to
     */
    void SetupImuPipeline(dai::Pipeline& pipeline);

    /**
     * Setup the Depth pipeline, which also configures the stereo cameras and StereoDepth
     *
     * @param pipeline the pipeline network the node belongs to
     */
    void SetupDepthPipeline(dai::Pipeline& pipeline);

    /**
     * Setup the Foxglove CompressedVideo ROS2 publisher
     */
    void SetupVideoPublishers();

    /**
     * Setup the Telemetry publisher, which includes both the IMU and Depth ROS2 publisher
     */
    void SetupTelemetryPublishers();

    /**
     * The DepthAI Device
     */
    std::shared_ptr<dai::Device> device_;

    /**
     * The DepthAI vision pipeline
     */
    std::shared_ptr<dai::Pipeline> pipeline_;

    /**
     * The RGB camera node, that interfaces with the device color camera in the pipeline
     */
    std::shared_ptr<dai::node::Camera> color_cam_;

    /**
     * VideoEncoder node, that encodes the camera feed using H.264 
     */
    std::shared_ptr<dai::node::VideoEncoder> video_enc_;

    /**
     * IMU node, which interfaces with the device IMU in the pipeline 
     */
    std::shared_ptr<dai::node::IMU> imu_node_;

    /**
     * Left stereo camera node, that interfaces with the device left stereo camera
     */
    std::shared_ptr<dai::node::Camera> mono_left_;

    /**
     * Right stereo camera node, that interfaces with the device left stereo camera
     */
    std::shared_ptr<dai::node::Camera> mono_right_;

    /**
     * Stereo Depth node, calculates the depth of from a pair of stereo cameras
     */
    std::shared_ptr<dai::node::StereoDepth> stereo_depth_;

    /**
     * Encoded video message queue
     */
    std::shared_ptr<dai::MessageQueue> encoded_q_;

    /**
     * IMU message queue
     */
    std::shared_ptr<dai::MessageQueue> imu_q_;

    /**
     * Depth message queue
     */
    std::shared_ptr<dai::MessageQueue> depth_q_;

    /**
     * Foxglove CompressedVideo ROS2 topic publisher
     */
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;

    /**
     * IMU message ROS2 topic publisher 
     */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    /**
     * Depth message ROS2 topic publisher
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    /**
     * Depth camera info message ROS2 topic publisher 
     */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

    /**
     * ROS2 Camera calibration parameters for the depth sensor, and depth image rectification.
     */
    sensor_msgs::msg::CameraInfo depth_info_msg_;

    /**
     * DepthAI IMU data converter
     */
    std::shared_ptr<depthai_bridge::ImuConverter> imu_conv_;

    /**
     * DepthAI Depth data converter
     */
    std::shared_ptr<depthai_bridge::ImageConverter> depth_conv_;

    /**
     * A counter to ensure that IMU publisher has gone through a certain amount of data before starting to publish
     */
    int imu_warmup_count_;

    /**
     * A counter to ensure that Depth publisher has gone through a certain amount of data before starting to publish
     */
    int depth_warmup_count_;

};

#endif //UMRT_ROS_POE_CAM__MOBILENET_PUBLISHER
