#include <cstdio>
#include <iostream>

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include "depthai/depthai.hpp"

// DepthAI Pipeline
dai::Pipeline createPipeline() {

    // Initialize Pipeline
    dai::Pipeline pipeline;

    //  Create the Camera Input, the VideoEncoder, and then the xLinkOut
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xLinkOut = pipeline.create<dai::node::XLinkOut>();

    xLinkOut->setStreamName("encoded_video");

    colorCam->setFps(30);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    colorCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    colorCam->setInterleaved(true);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Setting to 26 FPS will trigger error so set to 25, none the less try 30 FPS
    videoEnc->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::H264_MAIN);
    //videoEnc->setQuality(60); only for MJPEG
    videoEnc->setKeyframeFrequency(30);  // Force an IDR frame every 30 frames (~1/sec @ 30fps)
    videoEnc->setFrameRate(30);

    // Link the colorCam Video output to the VideoEncoder Input
    colorCam->video.link(videoEnc->input);
    // Link the video bitstream output to the xLinkOut Input 
    // This one i have to consider since bitstream and out is mutually exclusive
    videoEnc->out.link(xLinkOut->input);

    return pipeline;
}

/*
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

        pipeline = createPipeline();
        device = std::make_shared<dai::Device>(pipeline);
        RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", device->isPipelineRunning() ? "yes" : "no");

        encodedQueue = device->getOutputQueue("encoded_video", 30, false);
        
        //  Timer
        timer = this->create_wall_timer(
                std::chrono::milliseconds(33),
                std::bind(&MobileNetPublisherNode::publishEncodedImage, this));
    }

private: 
    //  PublishEncodedImage Function - This is a ROS2 function for the MobileNetPublisher Node which will 
    //  grab a frame from the EncodedFrame from the VideoEncoder and format it to the CompressedVideo message 
    //  from Foxglove and then publish it to the topic
    void publishEncodedImage() {
        // Get Frame
        // auto frame = encodedQueue->get<dai::ImgFrame>(); // this was bitstream which is mutually exclusive to out 
        auto frame = encodedQueue->get<dai::EncodedFrame>();

        // Setup 
        auto msg = std::make_shared<foxglove_msgs::msg::CompressedVideo>();

        if (frame == nullptr) {
            std::cout << "Null Frame! Checking next..." << std::endl;
            return;
        }

        //  
        msg->timestamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );  
        msg->frame_id = "oakd_camera";
        msg->format = "h264";
        msg->data = frame->getData();

        //  Publish the 
        encoded_pub_->publish(*msg);
    }

    // Variables 
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> encodedQueue;
    dai::Pipeline pipeline;

};

//  Main - Spin the Node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileNetPublisherNode>());
    rclcpp::shutdown();

    return 0;
}