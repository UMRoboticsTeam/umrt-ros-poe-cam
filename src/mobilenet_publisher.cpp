#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <foxglove_msgs/msg/compressed_video.hpp>
#include "depthai/depthai.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/pose_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>



// DepthAI Pipeline
dai::Pipeline createPipeline() {
    // Initialize Pipeline
    dai::Pipeline pipeline;

    //  Create the Camera Input, the VideoEncoder, and then the xLinkOut
    auto color_cam = pipeline.create<dai::node::ColorCamera>();
    auto video_enc = pipeline.create<dai::node::VideoEncoder>();
    auto xlink_out_encoded = pipeline.create<dai::node::XLinkOut>();
    auto xlink_out_aruco = pipeline.create<dai::node::XLinkOut>();
    
    xlink_out_encoded->setStreamName("encoded_video");
    xlink_out_aruco->setStreamName("aruco_stream");

    color_cam->setFps(30);
    color_cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    color_cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    color_cam->setInterleaved(true);
    color_cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Setting to 26 FPS will trigger error so set to 25, none the less try 30 FPS
    video_enc->setDefaultProfilePreset(color_cam->getFps(), dai::VideoEncoderProperties::Profile::H264_MAIN);
    video_enc->setKeyframeFrequency(30);  // Force an IDR frame every 30 frames (~1/sec @ 30fps)
    video_enc->setFrameRate(30);

    // Link the color_cam Video output to the VideoEncoder Input
    color_cam->video.link(video_enc->input);
    
    // Link aruco_stream for ArUco detection (uncompressed)
    color_cam->preview.link(xlink_out_aruco->input);

    // Link the video bitstream output to the xLinkOut Input 
    // This one i have to consider since bitstream and out is mutually exclusive
    video_enc->out.link(xlink_out_encoded->input);
    
    return pipeline;
}

/**
 *  MobileNetPublisherNode Class - Create a node that publishes to a topic called "/encoded_video"
 */
class MobileNetPublisherNode : public rclcpp::Node {
public:
    MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
        // Publisher for compressed video
        encoded_pub_ = this->create_publisher<foxglove_msgs::msg::CompressedVideo>(
                "encoded_video",
                rclcpp::QoS(rclcpp::KeepLast(1))
                        .best_effort()
                        .durability_volatile());

        aruco_marker_id_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "aruco_marker_ids",
                rclcpp::QoS(rclcpp::KeepLast(10))
                        .best_effort()
                        .durability_volatile());


        // Initialize ArUco detector and uses 4 bytes_100
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        aruco_params_ = cv::aruco::DetectorParameters::create();

        pipeline = createPipeline();
        device = std::make_shared<dai::Device>(pipeline);
        RCLCPP_INFO(this->get_logger(), "Pipeline running: %s", device->isPipelineRunning() ? "yes" : "no");

        encoded_queue = device->getOutputQueue("encoded_video", 30, false);
        aruco_stream_queue = device->getOutputQueue("aruco_stream", 10, false);
        
        //  Timer
        timer = this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]() {
                publishEncodedImage();
                processArUcoDetection();
            });
    }

private: 
    /**
     *  PublishEncodedImage Function - This is a ROS2 function for the MobileNetPublisher Node which will 
     *  grab a frame from the EncodedFrame from the VideoEncoder and format it to the CompressedVideo message 
     *  from Foxglove and then publish it to the topic
     */
    void publishEncodedImage() {
        auto frame = encoded_queue->get<dai::EncodedFrame>();

        auto msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
        
        if (frame == nullptr) {
            RCLCPP_WARN(rclcpp::get_logger("logger"), "Null encoded frame! Checking next...");
            return;
        }

        msg->timestamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            frame->getTimestamp().time_since_epoch()).count()
        );  
        msg->frame_id = "oakd_camera";
        msg->format = "h264";
        msg->data = frame->getData();

        encoded_pub_->publish(std::move(msg));
    }

    /**
     * Process ArUco detection on uncompressed aruco_stream frames - This is a ROS2 function for the MobileNetPublisher Node which will 
     *  grab a raw (uncompressed) frame from the ArUco stream queue, convert it to an OpenCV Mat, and detect
     *  ArUco markers using the specified dictionary and parameters. For each detected marker, it calculates
     *  the 2D center position, creates a simple 3D pose (with a fixed Z-depth for visualization), and 
     *  generates a visualization marker showing the marker ID as text. The markers are aggregated into a 
     *  MarkerArray message and published to the visualization topic so the markers can be displayed in 
     *  Foxglove Studio.
     */
    void processArUcoDetection() {
        auto frame = aruco_stream_queue->get<dai::ImgFrame>();
        
        if (frame == nullptr) return;

        // Convert DepthAI frame to OpenCV Mat
        cv::Mat cv_frame(frame->getHeight(), frame->getWidth(), CV_8UC3, frame->getData().data());
        
        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected_candidates;
        
        cv::aruco::detectMarkers(cv_frame, aruco_dict_, marker_corners, marker_ids, aruco_params_, rejected_candidates);
        
        if (!marker_ids.empty()) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu ArUco markers", marker_ids.size());
            
            // //Create MarkerArray message for visualization
            auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
            
            // For each detected marker
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                
                // Calculate marker center
                cv::Point2f center(0, 0);
                for (const auto& corner : marker_corners[i]) {
                    center += corner;
                }   
                center /= 4.0f;
                
                // Simple pose (you'd want to do proper pose estimation with camera calibration)
                geometry_msgs::msg::Pose pose;
                pose.position.x = center.x;
                pose.position.y = center.y;
                pose.position.z = 0.05;
                
                //Create visualization marker for the ID text
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = rclcpp::Time(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                    frame->getTimestamp().time_since_epoch()).count()
                );
                marker.header.frame_id = "oakd_camera";
                marker.ns = "aruco_id";
                marker.id = marker_ids[i];
                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = pose;
                marker.scale.z = 0.05;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.text = std::to_string(marker_ids[i]);
                marker_array_msg->markers.push_back(marker);
            }
            // Publish the marker array
            aruco_marker_id_pub_->publish(std::move(marker_array_msg));
        }
    }

    // Variables 
    rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr encoded_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr aruco_marker_id_pub_;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> encoded_queue;
    std::shared_ptr<dai::DataOutputQueue> aruco_stream_queue;
    dai::Pipeline pipeline;
    
    // ArUco detection
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
};

//  Main - Spin the Node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileNetPublisherNode>());
    rclcpp::shutdown();
    return 0;
}