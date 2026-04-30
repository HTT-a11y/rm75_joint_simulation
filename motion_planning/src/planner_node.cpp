#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

// 💥 新增：笛卡尔坐标与四元数转换必备头文件 💥
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_dual_arm_planner", options);

    // 开启一个独立线程来处理 ROS 状态回调
    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::thread([&executor, &node]() {
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
    });

    RCLCPP_INFO(node->get_logger(), "MTC 笛卡尔空间双臂规划节点已启动...");

    // 1. 创建任务并加载机器人模型
    Task task;
    task.stages()->setName("Dual Arm Cartesian Move");
    task.loadRobotModel(node);

    // 获取当前状态作为起点
    auto current_state = std::make_unique<stages::CurrentState>("Current State");
    task.add(std::move(current_state));

    // 2. 设置通用规划器 (使用 OMPL 进行逆运动学求解和避障)
    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline_planner->setPlannerId("RRTConnectkConfigDefault");

    // ===================================================================
    // 🎭 Stage 1: 左臂运动到笛卡尔目标点
    // ===================================================================
    auto move_left = std::make_unique<stages::MoveTo>("Left Arm Cartesian", pipeline_planner); 
    move_left->setGroup("left_arm");

// 💥 新增：明确指定以哪个连杆去对齐目标点 💥
    // 这里使用 left_Link7，请确保大小写与你的 URDF/SRDF 完全一致
    move_left->setIKFrame("left_Link7");

    geometry_msgs::msg::PoseStamped left_pose_msg;
    left_pose_msg.header.frame_id = "world"; // 以世界坐标系为基准
    
    // 设置左臂目标 XYZ 位置 (注意：请确保此点在机械臂的物理工作空间内！)
    // 假设桌子在 z=0.5，这里定在桌面上方 30cm 处
    left_pose_msg.pose.position.x = 0.302; 
    left_pose_msg.pose.position.y = 0.070; 
    left_pose_msg.pose.position.z = 1.448; 
    
    // 设置末端姿态 (RPY 转 四元数)
    tf2::Quaternion left_q;
    left_q.setRPY(-0.000, 0.028, -0.001); // Pitch旋转90度，让夹爪垂直朝下
    left_pose_msg.pose.orientation = tf2::toMsg(left_q);

    move_left->setGoal(left_pose_msg);
    task.add(std::move(move_left));

    // ===================================================================
    // 🎭 Stage 2: 右臂运动到笛卡尔目标点
    // ===================================================================
    auto move_right = std::make_unique<stages::MoveTo>("Right Arm Cartesian", pipeline_planner);
    move_right->setGroup("right_arm");

// 💥 新增：明确指定右臂的末端连杆 💥
    move_right->setIKFrame("right_Link7");
    
    geometry_msgs::msg::PoseStamped right_pose_msg;
    right_pose_msg.header.frame_id = "world";
    
    // 设置右臂目标 XYZ (通常与左臂 Y 轴对称)
    right_pose_msg.pose.position.x = 0.324; 
    right_pose_msg.pose.position.y = -0.019; 
    right_pose_msg.pose.position.z = 1.438; 
    
    // 设置末端姿态
    tf2::Quaternion right_q;
    right_q.setRPY(-1.797, 0.027, -0.001); 
    right_pose_msg.pose.orientation = tf2::toMsg(right_q);

    move_right->setGoal(right_pose_msg);
    task.add(std::move(move_right));

    // ===================================================================
    // 🚀 执行推演与规划
    // ===================================================================
    try {
        task.init();
        
        RCLCPP_INFO(node->get_logger(), "开始推演...");
        if (task.plan(5)) { // 最多生成 5 种备选方案
            RCLCPP_INFO(node->get_logger(), "推演成功！准备执行...");
            task.execute(*task.solutions().front()); // 执行最优解
            RCLCPP_INFO(node->get_logger(), "执行完成！");
        } else {
            RCLCPP_ERROR(node->get_logger(), "推演失败！(可能是因为坐标超出了工作空间，或者处于奇异点)");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "MTC 异常: %s", e.what());
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
