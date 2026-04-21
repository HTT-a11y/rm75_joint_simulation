#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class MockTaskPublisher : public rclcpp::Node
{
public:
    MockTaskPublisher() : Node("mock_task_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("task_command", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&MockTaskPublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "模拟发布者已启动，每5秒发送一次目标位姿...");
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Pose();
        
        // 模拟一个位于机械臂前方的目标位置
        message.position.x = 0.35;
        message.position.y = 0.1; 
        message.position.z = 0.4;
        
        // 保持末端垂直向下（根据 RM75 的坐标系设定）
        message.orientation.w = 1.0; 

        RCLCPP_INFO(this->get_logger(), "正在发布模拟目标: [x:0.35, y:0.1, z:0.4]");
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockTaskPublisher>());
    rclcpp::shutdown();
    return 0;
}
