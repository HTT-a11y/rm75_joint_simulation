#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

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

    RCLCPP_INFO(node->get_logger(), "MTC 双臂规划节点已启动，正在构建 Stage 1...");

    // 1. 创建任务并加载机器人模型
    Task task;
    task.stages()->setName("Dual Arm Pre-Grasp");
    task.loadRobotModel(node);

    // 2. 设置基础规划器 (使用 OMPL)
    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline_planner->setPlannerId("");

    // ===================================================================
    // 🎭 Stage 0: 获取当前机器人的真实状态
    // ===================================================================
    task.add(std::make_unique<stages::CurrentState>("Current State"));

    // ===================================================================
    // 🎭 Stage 1-A: 左臂独立运动到预备点
    // ===================================================================
    // 💥 新增：创建一个自带时间参数化的关节插值求解器 💥
    auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

    // ===================================================================
    // 🎭 Stage 1-A: 左臂独立运动到预备点
    // ===================================================================
    // 💥 注意：这里把第二个参数换成了 interpolation_planner 💥
    auto move_left = std::make_unique<stages::MoveTo>("Left Arm Pre-Grasp", interpolation_planner); 
    move_left->setGroup("left_arm");
    // 使用字典直接设定关节目标角度 (单位: 弧度)
    // 这里设定一个假想的“向内靠拢”的预备姿态，你可以根据实际情况调整数值
    std::map<std::string, double> left_target = {
        {"left_joint1", 0.5},
        {"left_joint2", 0.5},
        {"left_joint3", 0.0},
        {"left_joint4", 0.5},
        {"left_joint5", 0.0},
        {"left_joint6", 0.0},
        {"left_joint7", 0.0}
    };
    move_left->setGoal(left_target);
    task.add(std::move(move_left));

    // ===================================================================
    // 🎭 Stage 1-B: 右臂独立运动到预备点
    // ===================================================================
    auto move_right = std::make_unique<stages::MoveTo>("Right Arm Pre-Grasp", interpolation_planner);
    move_right->setGroup("right_arm");
    
    // 右臂同样向内靠拢，形成准备合抱的姿态
    std::map<std::string, double> right_target = {
        {"right_joint1", -0.5},
        {"right_joint2", 0.5},
        {"right_joint3", 0.0},
        {"right_joint4", 0.5},
        {"right_joint5", 0.0},
        {"right_joint6", 0.0},
        {"right_joint7", 0.0}
    };
    move_right->setGoal(right_target);
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
            RCLCPP_ERROR(node->get_logger(), "推演失败，请检查碰撞或奇异点。");
        }
    } catch (const InitStageException& e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "初始化失败: " << e.what());
    }

    // 优雅退出
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
