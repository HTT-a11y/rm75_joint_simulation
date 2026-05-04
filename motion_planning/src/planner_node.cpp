#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_state.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>
#include <future>
#include <sstream>

using namespace moveit::task_constructor;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// 从完整 RobotTrajectory 中提取指定 group 的子轨迹
trajectory_msgs::msg::JointTrajectory extractSubTrajectory(
    const robot_trajectory::RobotTrajectory& full_traj,
    const moveit::core::JointModelGroup* jmg)
{
    trajectory_msgs::msg::JointTrajectory result;
    result.joint_names = jmg->getActiveJointModelNames();

    std::vector<double> prev_positions;

    for (size_t i = 0; i < full_traj.getWayPointCount(); ++i) {
        const auto& waypoint = full_traj.getWayPoint(i);

        std::vector<double> positions;
        waypoint.copyJointGroupPositions(jmg, positions);

        double secs = full_traj.getWayPointDurationFromStart(i);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;

        // 从相邻路径点计算速度（第一个点不设速度）
        if (i > 0) {
            double dt = secs - full_traj.getWayPointDurationFromStart(i - 1);
            if (dt > 1e-9) {
                point.velocities.resize(positions.size(), 0.0);
                for (size_t j = 0; j < positions.size(); ++j)
                    point.velocities[j] = (positions[j] - prev_positions[j]) / dt;
            }
        }

        point.time_from_start.sec = static_cast<int32_t>(secs);
        point.time_from_start.nanosec = static_cast<uint32_t>((secs - point.time_from_start.sec) * 1e9);

        result.points.push_back(point);
        prev_positions = std::move(positions);
    }
    return result;
}

// 发送轨迹到指定控制器，返回 result future
// 注意：client 必须由调用方持有以保证生命周期覆盖 async_get_result 的等待期
std::shared_future<rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult>
sendTrajectory(
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
    const trajectory_msgs::msg::JointTrajectory& traj)
{
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        throw std::runtime_error("Action server not available");
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;

    auto goal_handle_future = client->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        throw std::runtime_error("Goal send timeout");
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        throw std::runtime_error("Goal rejected");
    }

    return client->async_get_result(goal_handle);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("mtc_dual_arm_planner", options);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto spin_thread = std::thread([&executor, &node]() {
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
    });

    RCLCPP_INFO(node->get_logger(), "MTC 双臂同步笛卡尔空间规划节点已启动...");

    // 1. 创建任务并加载机器人模型
    Task task;
    task.stages()->setName("Dual Arm Simultaneous Cartesian Move");
    task.loadRobotModel(node);

    auto current_state = std::make_unique<stages::CurrentState>("Current State");
    task.add(std::move(current_state));

    // 2. 设置通用规划器
    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline_planner->setPlannerId("RRTConnectkConfigDefault");

    // ===================================================================
    // Stage: 预计算双臂 IK，使用 dual_arm 组单次 MoveTo 实现同步规划
    // ===================================================================

    const auto& robot_model = task.getRobotModel();
    const moveit::core::JointModelGroup* left_jmg  = robot_model->getJointModelGroup("left_arm");
    const moveit::core::JointModelGroup* right_jmg = robot_model->getJointModelGroup("right_arm");
    const moveit::core::JointModelGroup* dual_jmg  = robot_model->getJointModelGroup("dual_arm");

    if (!left_jmg || !right_jmg || !dual_jmg) {
        RCLCPP_FATAL(node->get_logger(), "无法获取关节模型组，请检查 SRDF 配置。");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    moveit::core::RobotState ik_state(robot_model);
    ik_state.setToDefaultValues();

    // 左臂笛卡尔目标位姿
    geometry_msgs::msg::PoseStamped left_pose_msg;
    left_pose_msg.header.frame_id = "world";
    left_pose_msg.pose.position.x = 0.302;
    left_pose_msg.pose.position.y = 0.070;
    left_pose_msg.pose.position.z = 1.448;
    {
        tf2::Quaternion q;
        q.setRPY(-0.000, 0.028, -0.001);
        left_pose_msg.pose.orientation = tf2::toMsg(q);
    }

    if (!ik_state.setFromIK(left_jmg, left_pose_msg.pose, 0.1)) {
        RCLCPP_ERROR(node->get_logger(),
            "左臂 IK 求解失败：目标 [%.3f, %.3f, %.3f] 可能超出工作空间。",
            left_pose_msg.pose.position.x,
            left_pose_msg.pose.position.y,
            left_pose_msg.pose.position.z);
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "左臂 IK 求解成功。");

    // 右臂笛卡尔目标位姿
    geometry_msgs::msg::PoseStamped right_pose_msg;
    right_pose_msg.header.frame_id = "world";
    right_pose_msg.pose.position.x = 0.324;
    right_pose_msg.pose.position.y = -0.019;
    right_pose_msg.pose.position.z = 1.438;
    {
        tf2::Quaternion q;
        q.setRPY(-1.797, 0.027, -0.001);
        right_pose_msg.pose.orientation = tf2::toMsg(q);
    }

    if (!ik_state.setFromIK(right_jmg, right_pose_msg.pose, 0.1)) {
        RCLCPP_ERROR(node->get_logger(),
            "右臂 IK 求解失败：目标 [%.3f, %.3f, %.3f] 可能超出工作空间。",
            right_pose_msg.pose.position.x,
            right_pose_msg.pose.position.y,
            right_pose_msg.pose.position.z);
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "右臂 IK 求解成功。");

    // 构建 is_diff = true 的 RobotState 消息 —— 仅包含 dual_arm 关节
    moveit_msgs::msg::RobotState goal_msg;
    goal_msg.is_diff = true;

    std::vector<double> dual_joint_values;
    ik_state.copyJointGroupPositions(dual_jmg, dual_joint_values);
    goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
    goal_msg.joint_state.position = dual_joint_values;

    RCLCPP_INFO(node->get_logger(), "构建 dual_arm 关节空间目标，共 %zu 个关节。",
                dual_joint_values.size());

    auto move_dual = std::make_unique<stages::MoveTo>("Dual Arm Simultaneous", pipeline_planner);
    move_dual->setGroup("dual_arm");
    move_dual->setGoal(goal_msg);
    task.add(std::move(move_dual));

    // ===================================================================
    // 执行推演与规划
    // ===================================================================
    try {
        task.init();

        RCLCPP_INFO(node->get_logger(), "开始推演...");
        if (!task.plan(5)) {
            RCLCPP_ERROR(node->get_logger(), "推演失败！(目标可能发生碰撞或超出工作空间)");
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "推演成功！提取轨迹并同步执行...");

        // 从 MTC solution 中提取 dual_arm 轨迹
        auto& solution = task.solutions().front();
        robot_trajectory::RobotTrajectoryConstPtr full_traj;

        auto* seq = dynamic_cast<const SolutionSequence*>(solution.get());
        if (seq) {
            for (auto& sub : seq->solutions()) {
                auto* sub_traj = dynamic_cast<const SubTrajectory*>(sub);
                if (sub_traj && sub_traj->trajectory()) {
                    full_traj = sub_traj->trajectory();
                    break;
                }
            }
        }

        if (!full_traj || full_traj->getWayPointCount() == 0) {
            RCLCPP_ERROR(node->get_logger(), "无法从 MTC 解中获取轨迹。");
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "轨迹包含 %zu 个路径点，持续 %.2f 秒。",
                    full_traj->getWayPointCount(),
                    full_traj->getWayPointDurationFromStart(full_traj->getWayPointCount() - 1));

        // 拆分为左右臂子轨迹
        auto left_traj  = extractSubTrajectory(*full_traj, left_jmg);
        auto right_traj = extractSubTrajectory(*full_traj, right_jmg);

        RCLCPP_INFO(node->get_logger(), "左臂子轨迹: %zu 个路径点, %zu 个关节。",
                    left_traj.points.size(), left_traj.joint_names.size());
        RCLCPP_INFO(node->get_logger(), "右臂子轨迹: %zu 个路径点, %zu 个关节。",
                    right_traj.points.size(), right_traj.joint_names.size());

        // 打印关节名和目标位置以验证正确性
        {
            std::ostringstream oss;
            oss << "左臂关节: ";
            for (size_t j = 0; j < left_traj.joint_names.size(); ++j) {
                oss << left_traj.joint_names[j] << "=" << left_traj.points.back().positions[j];
                if (j < left_traj.joint_names.size() - 1) oss << ", ";
            }
            RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
        }
        {
            std::ostringstream oss;
            oss << "右臂关节: ";
            for (size_t j = 0; j < right_traj.joint_names.size(); ++j) {
                oss << right_traj.joint_names[j] << "=" << right_traj.points.back().positions[j];
                if (j < right_traj.joint_names.size() - 1) oss << ", ";
            }
            RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());
        }

        // 尝试使用 dual_arm_controller（14 关节一体）发送完整轨迹
        auto dual_client = rclcpp_action::create_client<FollowJointTrajectory>(
            node, "/dual_arm_controller/follow_joint_trajectory");
        bool use_dual = dual_client->wait_for_action_server(std::chrono::seconds(2));

        if (use_dual) {
            RCLCPP_INFO(node->get_logger(), "检测到 dual_arm_controller，使用单轨迹同步执行。");
            // 提取完整 14 关节轨迹
            trajectory_msgs::msg::JointTrajectory dual_traj;
            dual_traj.joint_names = dual_jmg->getActiveJointModelNames();
            for (size_t i = 0; i < full_traj->getWayPointCount(); ++i) {
                std::vector<double> positions;
                full_traj->getWayPoint(i).copyJointGroupPositions(dual_jmg, positions);
                double secs = full_traj->getWayPointDurationFromStart(i);
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = positions;
                if (i > 0) {
                    double dt = secs - full_traj->getWayPointDurationFromStart(i - 1);
                    if (dt > 1e-9) {
                        point.velocities.resize(positions.size(), 0.0);
                        std::vector<double> prev;
                        full_traj->getWayPoint(i - 1).copyJointGroupPositions(dual_jmg, prev);
                        for (size_t j = 0; j < positions.size(); ++j)
                            point.velocities[j] = (positions[j] - prev[j]) / dt;
                    }
                }
                point.time_from_start.sec = static_cast<int32_t>(secs);
                point.time_from_start.nanosec = static_cast<uint32_t>((secs - point.time_from_start.sec) * 1e9);
                dual_traj.points.push_back(point);
            }
            RCLCPP_INFO(node->get_logger(), "dual_arm 轨迹: %zu 个路径点, %zu 个关节。",
                        dual_traj.points.size(), dual_traj.joint_names.size());
            auto dual_future = sendTrajectory(dual_client, dual_traj);
            RCLCPP_INFO(node->get_logger(), "dual_arm 轨迹已发送，等待执行完成...");
            auto status = dual_future.wait_for(std::chrono::seconds(30));
            bool ok = (status == std::future_status::ready &&
                       dual_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
            if (ok)
                RCLCPP_INFO(node->get_logger(), "双臂同步执行完成！");
            else
                RCLCPP_ERROR(node->get_logger(), "dual_arm 执行失败或超时。");
        } else {
            RCLCPP_INFO(node->get_logger(), "未检测到 dual_arm_controller，拆分为左右臂子轨迹。");
            // 创建 action client（在 main 作用域保持生命周期）
            auto left_client = rclcpp_action::create_client<FollowJointTrajectory>(
                node, "/left_arm_controller/follow_joint_trajectory");
            auto right_client = rclcpp_action::create_client<FollowJointTrajectory>(
                node, "/right_arm_controller/follow_joint_trajectory");

            // 同时发送给两个控制器
            auto left_future = sendTrajectory(left_client, left_traj);
            auto right_future = sendTrajectory(right_client, right_traj);

            RCLCPP_INFO(node->get_logger(), "双臂轨迹已发送，等待执行完成...");

            // 等待双臂均完成
            auto left_status  = left_future.wait_for(std::chrono::seconds(30));
            auto right_status = right_future.wait_for(std::chrono::seconds(30));

            bool left_ok  = (left_status  == std::future_status::ready &&
                             left_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
            bool right_ok = (right_status == std::future_status::ready &&
                             right_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);

            if (left_ok && right_ok) {
                RCLCPP_INFO(node->get_logger(), "双臂同步执行完成！");
            } else {
                if (!left_ok)  RCLCPP_ERROR(node->get_logger(), "左臂执行失败或超时。");
                if (!right_ok) RCLCPP_ERROR(node->get_logger(), "右臂执行失败或超时。");
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "异常: %s", e.what());
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
