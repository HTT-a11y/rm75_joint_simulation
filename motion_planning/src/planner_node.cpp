#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class MotionPlanner : public rclcpp::Node
{
public:
    MotionPlanner() : Node("motion_planner_node")
    {
        // 1. 初始化订阅者，监听 "task_command" 话题
        // 这里使用标准 Pose 消息，实际开发中可替换为同事定义的自定义消息
        target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "task_command", 10, std::bind(&MotionPlanner::targetCallback, this, std::placeholders::_1));
        
        planning_group_ = "rm_group"; 
    }

    void init()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group_);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        RCLCPP_INFO(this->get_logger(), "规划框架已就绪，等待 task_command 话题数据...");
    }

private:
    // 2. 订阅者回调函数
    void targetCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到新目标位置: [x:%.2f, y:%.2f, z:%.2f]", 
                    msg->position.x, msg->position.y, msg->position.z);
        
        // 触发规划与执行
        this->planAndExecuteToPose(*msg);
    }

    bool planAndExecuteToPose(const geometry_msgs::msg::Pose& target_pose)
    {
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行运动...");
            move_group_->execute(my_plan);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败！请检查目标点是否在工作空间内。");
            return false;
        }
    }

    std::string planning_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_; // 订阅者句柄
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanner>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    std::thread spinner_thread([&executor]() { executor.spin(); });

    node->init();

    // 阻塞主线程，保持节点运行
    spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}
