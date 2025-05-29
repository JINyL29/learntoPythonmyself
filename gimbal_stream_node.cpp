#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class GimbalStreamNode : public rclcpp::Node
{
public:
    GimbalStreamNode()
    : Node("a8mini_gimbal_node"), running_(true)
    {
        // ROS2 이미지 퍼블리셔 생성
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gimbal/image_raw", 10);

        // 비디오 스트리밍 스레드 시작
        video_thread_ = std::thread(&GimbalStreamNode::stream_video, this);

        RCLCPP_INFO(this->get_logger(), "✅ A8 Mini Gimbal Video Streaming Initialized");
    }

    ~GimbalStreamNode()
    {
        shutdown();
        if (video_thread_.joinable()) {
            video_thread_.join();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::thread video_thread_;
    std::atomic<bool> running_;

    void stream_video()
    {
        // 비디오 장치 오픈
        cv::VideoCapture cap("/dev/video2");
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Gimbal Camera Video Stream Failed");
            return;
        }

        // ✅ 해상도 설정: 1920x1080
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        RCLCPP_INFO(this->get_logger(), "📷 Gimbal Camera Video Streaming Started (1920x1080)");

        cv::Mat frame;
        cv_bridge::CvImage img_bridge;
        sensor_msgs::msg::Image::SharedPtr img_msg;

        while (rclcpp::ok() && running_)
        {
            cap >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Failed to Read Frame from Gimbal Camera");
                continue;
            }

            img_bridge.header.stamp = this->now();
            img_bridge.header.frame_id = "camera_frame";
            img_bridge.encoding = "bgr8";
            img_bridge.image = frame;

            img_msg = img_bridge.toImageMsg();
            image_pub_->publish(*img_msg);

            cv::imshow("Gimbal Camera Stream", frame);
            if (cv::waitKey(1) == 27) {  // ESC 키 누르면 종료
                running_ = false;
                break;
            }
        }

        cap.release();
        cv::destroyAllWindows();
    }

    void shutdown()
    {
        running_ = false;
        RCLCPP_INFO(this->get_logger(), "🛑 A8 Mini Gimbal Video Streaming Stopped");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalStreamNode>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
