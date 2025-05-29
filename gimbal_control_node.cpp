// gimbal_control_node.cpp (최적화 버전)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>
#include <algorithm>

class GimbalControlNode : public rclcpp::Node
{
public:
    GimbalControlNode()
    : Node("gimbal_control_node"), uart_fd_(-1),
      gimbal_max_speed_(50),
      current_zoom_int_(1), prev_zoom_axis_state_(0), running_(true)
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&GimbalControlNode::joy_callback, this, std::placeholders::_1)
        );
        gimbal_cmd_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "/gimbal/command_velocity", 10);

        RCLCPP_INFO(this->get_logger(), "Joystick subscription started: /joy");

        uart_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UART open failed: %s", strerror(errno));
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(uart_fd_);
            uart_fd_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "UART port /dev/ttyUSB0 connection successful");

        send_absolute_zoom(current_zoom_int_, 0);

        angle_request_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GimbalControlNode::send_angle_request, this));

        uart_thread_ = std::thread(&GimbalControlNode::uart_receive_loop, this);
    }

    ~GimbalControlNode()
    {
        running_ = false;
        if (uart_thread_.joinable())
            uart_thread_.join();
        if (uart_fd_ >= 0)
            close(uart_fd_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr gimbal_cmd_pub_;
    rclcpp::TimerBase::SharedPtr angle_request_timer_;
    std::thread uart_thread_;
    std::atomic<bool> running_;

    int uart_fd_;
    int gimbal_max_speed_;
    int current_zoom_int_;
    int prev_zoom_axis_state_;
    std::vector<uint8_t> recv_buffer_;

    void uart_write(const std::vector<uint8_t> &packet)
    {
        if (uart_fd_ >= 0) {
            write(uart_fd_, packet.data(), packet.size());
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Joystick axes length insufficient");
            return;
        }

        if (msg->buttons.size() > 1 && msg->buttons[1] == 1) {
            uart_write(create_center_packet());
            RCLCPP_INFO(this->get_logger(), "Button 1 -> Center command sent");
            return;
        }

        double joy_x = msg->axes[0];
        double joy_y = msg->axes[1];
        double zoom_axis = msg->axes[5];

        int yaw_speed = static_cast<int>(-joy_x * gimbal_max_speed_);
        int pitch_speed = static_cast<int>(-joy_y * gimbal_max_speed_);

        RCLCPP_INFO(this->get_logger(), "joy cmd - Yaw_speed: %d, Pitch_speed: %d", yaw_speed, pitch_speed);

        std_msgs::msg::Int16MultiArray cmd_msg;
        cmd_msg.data = {yaw_speed, pitch_speed};
        gimbal_cmd_pub_->publish(cmd_msg);

        uart_write(create_gimbal_packet(yaw_speed, pitch_speed));

        if (zoom_axis > 0.8 && prev_zoom_axis_state_ != 1) {
            if (current_zoom_int_ < 6) {
                current_zoom_int_++;
                send_absolute_zoom(current_zoom_int_, 0);
            }
            prev_zoom_axis_state_ = 1;
        } else if (zoom_axis < -0.8 && prev_zoom_axis_state_ != -1) {
            if (current_zoom_int_ > 1) {
                current_zoom_int_--;
                send_absolute_zoom(current_zoom_int_, 0);
            }
            prev_zoom_axis_state_ = -1;
        } else if (zoom_axis > -0.2 && zoom_axis < 0.2) {
            prev_zoom_axis_state_ = 0;
        }
    }

    void send_angle_request()
    {
        std::vector<uint8_t> packet = {
            0x55, 0x66, 0x01, 0x02, 0x02, 0x00, 0x00, 0x0E,
            0x00, 0x00, 0x00, 0x00
        };
        append_crc16(packet);
        uart_write(packet);
    }

    void uart_receive_loop()
    {
        uint8_t buffer[64];
        while (running_) {
            ssize_t len = read(uart_fd_, buffer, sizeof(buffer));
            if (len > 0) {
                recv_buffer_.insert(recv_buffer_.end(), buffer, buffer + len);
                while (recv_buffer_.size() >= 15) {
                    auto it = std::find(recv_buffer_.begin(), recv_buffer_.end(), 0x55);
                    if (it == recv_buffer_.end()) {
                        recv_buffer_.clear();
                        break;
                    }
                    size_t idx = std::distance(recv_buffer_.begin(), it);
                    if (recv_buffer_.size() - idx < 15)
                        break;
                    if (recv_buffer_[idx + 1] == 0x66 && recv_buffer_[idx + 7] == 0x0E) {
                        int16_t yaw = (recv_buffer_[idx + 8] << 8) | recv_buffer_[idx + 9];
                        int16_t pitch = (recv_buffer_[idx + 10] << 8) | recv_buffer_[idx + 11];
                        int16_t roll = (recv_buffer_[idx + 12] << 8) | recv_buffer_[idx + 13];
                        RCLCPP_INFO(this->get_logger(), "gimbal anglespeed - Yaw: %.1f°, Pitch: %.1f°, Roll: %.1f°",
                                    yaw / 10.0, pitch / 10.0, roll / 10.0);
                        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + idx + 15);
                    } else {
                        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + idx + 1);
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    std::vector<uint8_t> create_gimbal_packet(int yaw, int pitch)
    {
        std::vector<uint8_t> packet = {
            0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00,
            0x07, static_cast<uint8_t>(yaw & 0xFF), static_cast<uint8_t>(pitch & 0xFF)
        };
        append_crc16(packet);
        return packet;
    }

    std::vector<uint8_t> create_center_packet()
    {
        std::vector<uint8_t> packet = {
            0x55, 0x66, 0x01, 0x01, 0x00, 0x00, 0x00,
            0x08, 0x01
        };
        append_crc16(packet);
        return packet;
    }

    void send_absolute_zoom(int zoom_int, int zoom_float)
    {
        std::vector<uint8_t> packet = {
            0x55, 0x66, 0x01, 0x02, 0x00, 0x00, 0x00,
            0x0F, static_cast<uint8_t>(zoom_int), static_cast<uint8_t>(zoom_float)
        };
        append_crc16(packet);
        uart_write(packet);
        RCLCPP_INFO(this->get_logger(), "Zoom command sent: %d.%d", zoom_int, zoom_float);
    }

    void append_crc16(std::vector<uint8_t> &packet)
    {
        uint16_t crc = crc16_ccitt(packet);
        packet.push_back(crc & 0xFF);
        packet.push_back((crc >> 8) & 0xFF);
    }

    uint16_t crc16_ccitt(const std::vector<uint8_t> &data)
    {
        static const uint16_t table[256] = {
            #include "crc16_table.inc"
        };
        uint16_t crc = 0x0000;
        for (auto b : data) {
            crc = (crc << 8) ^ table[((crc >> 8) ^ b) & 0xFF];
        }
        return crc;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
