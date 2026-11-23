/*
 * Copyright 2024 Edcel Abanto, University of Manitoba Robotics Team
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Created on 2026-29-05 by ea.
 */

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
#include "umrt-ros-poe-cam/mobilenet_publisher.hpp"


MobileNetPublisherNode::MobileNetPublisherNode() : Node("mobilenet_publisher_node") {

    //  Initialize the counters
    imu_warmup_count_=0;
    depth_warmup_count_=0;

    //  Initialize hardware connection
    device_ = std::make_shared<dai::Device>();
    pipeline_ = std::make_shared<dai::Pipeline>(device_);

    //  Build the pipeline
    SetupCameraPipeline(*pipeline_);
    SetupImuPipeline(*pipeline_);
    SetupDepthPipeline(*pipeline_);

    //  Start the pipeline
    pipeline_->start();

    RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", pipeline_->isRunning() ? "yes" : "no");

    if (!pipeline_->isRunning()) {
        RCLCPP_ERROR(this->get_logger(), "umrt-ros-poe-cam: DepthAI Pipeline failed to start or stopped unexpectedly! Shutting down node...");
        rclcpp::shutdown();
        return; 
    }

    RCLCPP_INFO(this->get_logger(), "Pipeline running successfully.");

    //  Setup the ROS2 publishers
    SetupVideoPublishers();
    SetupTelemetryPublishers();

}

void MobileNetPublisherNode::SetupCameraPipeline(dai::Pipeline& pipeline) {
    color_cam_ = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    video_enc_ = pipeline.create<dai::node::VideoEncoder>();

    //  VideoEncoder configurations
    video_enc_->setDefaultProfilePreset(CAMERA_FPS, dai::VideoEncoderProperties::Profile::H264_BASELINE);

    // Disable B-frames to eliminate frame reordering delays
    video_enc_->setNumBFrames(0);

    //set I-frame interval (GOP size) to match FPS
    video_enc_->setKeyframeFrequency(CAMERA_FPS);

    //enforce constant bitrate (CBR)
    video_enc_->setRateControlMode(dai::VideoEncoderProperties::RateControlMode::CBR);
    video_enc_->setBitrateKbps(2000); //4000 Kbps (4 Mbps), a stable baseline for 1080p

    //  For Encoder Stream
    auto cam_out = color_cam_->requestOutput({1920, 1080}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, CAMERA_FPS);
    cam_out->link(video_enc_->input);

    //  The 30 here indicates the max size of the output queue
    encoded_q_ = video_enc_->out.createOutputQueue(2, false);
}   //  SetupCameraPipeline()

void MobileNetPublisherNode::SetupImuPipeline(dai::Pipeline& pipeline) {
    imu_node_ = pipeline.create<dai::node::IMU>();

    //  IMU configurations
    imu_node_->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},  IMU_REPORT_RATE_Hz);
    imu_node_->setBatchReportThreshold(1);
    imu_node_->setMaxBatchReports(10);

    //  The 8 here indicates the max size of the output queue
    imu_q_ = imu_node_->out.createOutputQueue(8, false);
}   //  SetupImuPipeline()

void MobileNetPublisherNode::SetupDepthPipeline(dai::Pipeline& pipeline) {

    stereo_depth_ = pipeline.create<dai::node::StereoDepth>();

    //  Depth configurations
    stereo_depth_->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::ROBOTICS);
    // mono_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    // mono_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    // Throttle the VIO/Depth cameras to 15 FPS to guarantee bandwidth for the 200Hz IMU
    mono_left_ = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, DEPTH_FPS);
    mono_right_ = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, DEPTH_FPS);

    auto left_out = mono_left_->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, DEPTH_FPS);
    auto right_out = mono_right_->requestOutput({640, 400}, dai::ImgFrame::Type::RAW8, dai::ImgResizeMode::CROP, DEPTH_FPS);

    left_out->link(stereo_depth_->left);
    right_out->link(stereo_depth_->right);

    //  The 2 here indicates the max size of the output queue
    depth_q_ = stereo_depth_->depth.createOutputQueue(2, false);
}   //  SetupDepthPipeline()

void MobileNetPublisherNode::SetupVideoPublishers() {

    //  Best Effort QoS
    rclcpp::QoS best_effort_qos(10);
    best_effort_qos.best_effort();
    best_effort_qos.durability_volatile();

    encoded_pub_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>("encoded_video", best_effort_qos);

    encoded_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
        if (daiMsg) {
            foxglove_msgs::msg::CompressedVideo msg;
            msg.timestamp = this->get_clock()->now();
            msg.frame_id = "oakd_camera";
            msg.format = "h264";
            msg.data.assign(daiMsg->getData().begin(), daiMsg->getData().end());
            encoded_pub_->publish(std::move(msg));
        }
    });

}   //  SetupVideoPublishers()

void MobileNetPublisherNode::SetupTelemetryPublishers() {

    //  Initialize Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 30);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/camera_info", 10);

    //  Initialize the Bridge Converter to handle the math and sync
    depthai_bridge::ImuSyncMethod imuMode = depthai_bridge::ImuSyncMethod::COPY;
    imu_conv_ = std::make_shared<depthai_bridge::ImuConverter>("oakd_camera", imuMode);
    depth_conv_ = std::make_shared<depthai_bridge::ImageConverter>("oakd_camera", false);

    //  Depth Cam Info
    dai::CalibrationHandler calibData = device_->readCalibration();
    depth_info_msg_ = depth_conv_->calibrationToCameraInfo(calibData, dai::CameraBoardSocket::CAM_C, 640, 400);
    depth_info_msg_.header.frame_id = "oakd_camera";

    //  IMU Callback - Pushes data through the converter, and publishes to topic
    imu_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::IMUData>(data);
        if (daiMsg) {

            if (imu_warmup_count_ < MAX_WARMUP_COUNT) {
                imu_warmup_count_++;
                return; // Skip
            }

            std::deque<sensor_msgs::msg::Imu> rosMsgs;            
            // Let the bridge do the complex synchronization and math
            imu_conv_->toRosMsg(daiMsg, rosMsgs);

            // Publish the resulting valid ROS messages
            for (auto& msg : rosMsgs) {
                imu_pub_->publish(std::move(msg));
            }
        }
    });

    //  Depth Callback - Pushes data through the converter, and publishes to topic 
    depth_q_->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        auto daiMsg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        if (daiMsg) {

            if (depth_warmup_count_ < MAX_WARMUP_COUNT) {
                depth_warmup_count_++;
                return; // Skip
            }

            std::deque<sensor_msgs::msg::Image> rosMsgs;
            // Convert to ROS Image message
            depth_conv_->toRosMsg(daiMsg, rosMsgs);
            
            // Publish
            for (auto& msg : rosMsgs) {
                depth_pub_->publish(std::move(msg));
                
                depth_info_msg_.header.stamp = msg.header.stamp;
                depth_info_pub_->publish(depth_info_msg_);
            }
        }
    });

}   //  SetupTelemetryPublishers()
