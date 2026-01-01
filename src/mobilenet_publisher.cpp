#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include "depthai/depthai.hpp"

// DepthAI Pipeline
dai::Pipeline createPipeline() {

    // Initialize Pipeline
    dai::Pipeline pipeline;

    //  Create the Camera Input, the VideoEncoder, and then the xLinkOut
    auto color_cam = pipeline.create<dai::node::ColorCamera>();
    auto video_enc = pipeline.create<dai::node::VideoEncoder>();
    auto xlink_out = pipeline.create<dai::node::XLinkOut>();

    xlink_out->setStreamName("encoded_video");

    color_cam->setFps(30);
    color_cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    color_cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    color_cam->setInterleaved(true);
    color_cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Setting to 26 FPS will trigger error so set to 25, none the less try 30 FPS
    video_enc->setDefaultProfilePreset(color_cam->getFps(), dai::VideoEncoderProperties::Profile::H264_MAIN);
    //video_enc->setQuality(60); only for MJPEG
    video_enc->setKeyframeFrequency(30);  // Force an IDR frame every 30 frames (~1/sec @ 30fps)
    video_enc->setFrameRate(30);

    // Link the color_cam Video output to the VideoEncoder Input
    color_cam->video.link(video_enc->input);
    // Link the video bitstream output to the xLinkOut Input 
    // This one i have to consider since bitstream and out is mutually exclusive
    video_enc->out.link(xlink_out->input);

    return pipeline;
}

/**
 *  MobileNetPublisherNode Class - Create a node that publishes to a topic called "/encoded_video"
 */
class MobileNetPublisherNode : public rclcpp::Node {
public:
    MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
        encoded_pub_ = this->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
                "encoded_video",
                // Custom QoS for Best Effort
                rclcpp::QoS(rclcpp::KeepLast(1))
                        .best_effort()
                        .durability_volatile());

        pipeline = createPipeline();
        device = std::make_shared<dai::Device>(pipeline);
        RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", device->isPipelineRunning() ? "yes" : "no");

        encoded_queue = device->getOutputQueue("encoded_video", 30, false);
        
        //  Timer
        timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]() {
            publishEncodedImage(); 
            });
    }

private: 

    /**
     *  PublishEncodedImage Function - This is a ROS2 function for the MobileNetPublisher Node which will 
     *  grab a frame from the EncodedFrame from the VideoEncoder and format it to the FFMPEGPacket message 
     *  then publish it to the topic.
     */
    void publishEncodedImage() {
        // Get Frame
        // auto frame = encodedQueue->get<dai::ImgFrame>(); // this was bitstream which is mutually exclusive to out 
        auto frame = encoded_queue->get<dai::EncodedFrame>();

        // Initialize FFMPEGPacket Message 
        auto msg = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
        
        //  Ensure it's not a Null frame
        if (frame == nullptr) {
            RCLCPP_WARN(rclcpp::get_logger("logger"), "Null frame! Checking next...");
            return;
        }

        //  Setup the FFMPEG Packet Message
        // 1. Header
        msg->header.stamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );
        msg->header.frame_id = "oakd_camera";

        /*
        msg->timestamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );
        */  

        // 2. Metadata (HxW are from params)
        msg->encoding = "h264";
        msg->width = 1280;
        msg->height = 720; 

        // 3. Timing
        msg->pts = frame->getSequenceNum();

        // 4. Flags (Keyframes and IDR frames)
        // Important for H264 
        // 0 = P-Frame
        // 1 = I-Frame/Keyframe
        msg->flags = (frame->getFrameType() == dai::EncodedFrame::FrameType::I) ? 1 : 0;

        // 5. The data
        msg->data = frame->getData();

        //  Publish the FFMPEGPacket Message to Topic /encoded_video 
        encoded_pub_->publish(std::move(msg));
    }

    // Variables 
    rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr encoded_pub_;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> encoded_queue;
    dai::Pipeline pipeline;

};

//  Main - Spin the Node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileNetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}