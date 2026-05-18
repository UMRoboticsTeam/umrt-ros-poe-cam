#include <cstdio>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "depthai/depthai.hpp"

// DepthAI Pipeline
dai::Pipeline createPipeline() {

    // Initialize Pipeline
    dai::Pipeline pipeline;

    //  Create the Camera Input, the VideoEncoder, and then the xLinkOut for Video 
    auto color_cam = pipeline.create<dai::node::ColorCamera>();
    auto video_enc = pipeline.create<dai::node::VideoEncoder>();
    auto xlink_out_encoded = pipeline.create<dai::node::XLinkOut>();
    auto xlink_out_raw = pipeline.create<dai::node::XLinkOut>();
    
    xlink_out_encoded->setStreamName("encoded_video");
    xlink_out_raw->setStreamName("raw_video");

    color_cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    color_cam->setFps(30);
    color_cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    color_cam->setInterleaved(true);    //  Check if ROS2 requires this
    color_cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    color_cam->setPreviewSize(640, 360);

    // Setting to 26 FPS will trigger error so set to 25, none the less try 30 FPS
    video_enc->setDefaultProfilePreset(color_cam->getFps(), dai::VideoEncoderProperties::Profile::H264_MAIN);
    //video_enc->setQuality(60); only for MJPEG
    video_enc->setKeyframeFrequency(30);  // Force an IDR frame every 30 frames (~1/sec @ 30fps)
    video_enc->setFrameRate(30);

    // Link the color_cam Video output to the VideoEncoder Input
    // Link the video bitstream output to the xLinkOut Input 
    //  Encoded Video Output
    color_cam->video.link(video_enc->input);
    video_enc->out.link(xlink_out_encoded->input);

    //  Raw Video Output 
    //  Limit the Raw Video Output to 10 FPS to reduce bandwidth (only used for ArUco)
    color_cam->preview.link(xlink_out_raw->input);
    // xlink_out_raw->setFpsLimit(10);    

    return pipeline;
}

/**
 *  MobileNetPublisherNode Class - Create a node that publishes to a topic called "/encoded_video"
 */
class MobileNetPublisherNode : public rclcpp::Node {
public:
    MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
        encoded_pub_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>(
            "encoded_video",
            // Custom QoS for Best Effort
            rclcpp::QoS(rclcpp::KeepLast(1))
                    .best_effort()
                    .durability_volatile());
    
        raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "raw_video",
            // Custom QoS for Best Effort
            rclcpp::QoS(rclcpp::KeepLast(1))
                    .best_effort()
                    .durability_volatile());

        pipeline = createPipeline();
        device = std::make_shared<dai::Device>(pipeline);
        RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", device->isPipelineRunning() ? "yes" : "no");

        auto names = device->getOutputQueueNames();
        for(const auto& name : names) {
            RCLCPP_INFO(this->get_logger(), "Available Queue: %s", name.c_str());
        }

        encoded_queue = device->getOutputQueue("encoded_video", 5, false);
        raw_queue = device->getOutputQueue("raw_video", 1, false);

        encoded_thread_ = std::thread(&MobileNetPublisherNode::encodedLoop, this);
        raw_thread_ = std::thread(&MobileNetPublisherNode::rawLoop, this);

    }

    ~MobileNetPublisherNode() {
        if (encoded_thread_.joinable()) encoded_thread_.join();
        if (raw_thread_.joinable()) raw_thread_.join();
    }

private: 

    // Loop for the 30 FPS H.264 stream
    void encodedLoop() {
        while (rclcpp::ok()) {
            auto frame = encoded_queue->get<dai::EncodedFrame>();
            if (frame) {
                publishEncodedImage(frame);
            }
        }
    }

    // Loop for the 10 FPS Raw stream (ArUco)
    void rawLoop() {
            while (rclcpp::ok()) {
                auto frame = raw_queue->tryGet<dai::ImgFrame>(); 
                if (frame) {
                    publishRawImage(frame);
                }
            }
    }

    /**
     *  PublishEncodedImage Function - This is a ROS2 function for the MobileNetPublisher Node which will 
     *  grab a frame from the EncodedFrame from the VideoEncoder and format it to the CompressedVideo message 
     *  from Foxglove and then publish it to the topic
     */
    void publishEncodedImage(std::shared_ptr<dai::EncodedFrame> frame) {
        // Initialize CompressedVideo Message 
        auto msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
        
        //  Ensure it's not a Null frame
        if (frame == nullptr) {
            RCLCPP_WARN(rclcpp::get_logger("logger"), "Null frame! Checking next...");
            return;
        }

        //  Setup the CompressedVideo Message (timestamp, frame_id, format, and data)
        msg->timestamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );  
        msg->frame_id = "oakd_camera";
        msg->format = "h264";
        msg->data = frame->getData();

        //  Publish the CompressedVideo Message to Topic /encoded_video 
        encoded_pub_->publish(std::move(msg));
    }

    /**
     *  publishRawImage Function - This is a ROS2 function for the MobileNetPublisher Node which will 
     *  grab a frame from the PoE camera pipeline outpu, format it to the CompressedImage message 
     *  then publish it to the topic
     */
    void publishRawImage(std::shared_ptr<dai::ImgFrame> frame) {
        // Initialize Image Message 
        auto msg = std::make_unique<sensor_msgs::msg::Image>();

        if (frame == nullptr) {
            RCLCPP_WARN(rclcpp::get_logger("logger"), "Null frame! Checking next...");
            return;
        }
        
        // Setup header
        msg->header.stamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );  
        msg->header.frame_id = "oakd_camera";
        
        msg->height = frame->getHeight();
        msg->width = frame->getWidth();
        msg->encoding = "bgr8"; // Match the DepthAI output
        msg->is_bigendian = false;
        msg->step = msg->width * 3; 
        msg->data = frame->getData();

        raw_pub_->publish(std::move(msg));
    }   //  publishRawImage()

    // Variables 
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    // rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<dai::DataOutputQueue> encoded_queue;
    std::shared_ptr<dai::DataOutputQueue> raw_queue;
    std::thread encoded_thread_;
    std::thread raw_thread_;
    std::shared_ptr<dai::Device> device;
    dai::Pipeline pipeline;

};

//  Main - Spin the Node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileNetPublisherNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}