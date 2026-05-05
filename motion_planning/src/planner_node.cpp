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

#include <geometry_msgs/msg/pose.hpp>
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

// 获取 MTC 解中的 SubTrajectory
robot_trajectory::RobotTrajectoryConstPtr getSolutionTrajectory(const Task& task)
{
    auto& solution = task.solutions().front();
    auto* seq = dynamic_cast<const SolutionSequence*>(solution.get());
    if (!seq) return nullptr;

    for (auto& sub : seq->solutions()) {
        auto* sub_traj = dynamic_cast<const SubTrajectory*>(sub);
        if (sub_traj && sub_traj->trajectory())
            return sub_traj->trajectory();
    }
    return nullptr;
}

// 执行 dual_arm 轨迹：尝试 dual_arm_controller，否则拆分为左右臂
bool executeDualArmTrajectory(
    rclcpp::Node::SharedPtr node,
    const robot_trajectory::RobotTrajectory& full_traj,
    const moveit::core::JointModelGroup* dual_jmg,
    const moveit::core::JointModelGroup* left_jmg,
    const moveit::core::JointModelGroup* right_jmg)
{
    auto dual_client = rclcpp_action::create_client<FollowJointTrajectory>(
        node, "/dual_arm_controller/follow_joint_trajectory");
    bool use_dual = dual_client->wait_for_action_server(std::chrono::seconds(2));

    if (use_dual) {
        RCLCPP_INFO(node->get_logger(), "使用 dual_arm_controller 发送完整轨迹。");
        trajectory_msgs::msg::JointTrajectory dual_traj;
        dual_traj.joint_names = dual_jmg->getActiveJointModelNames();
        std::vector<double> prev;

        for (size_t i = 0; i < full_traj.getWayPointCount(); ++i) {
            std::vector<double> positions;
            full_traj.getWayPoint(i).copyJointGroupPositions(dual_jmg, positions);
            double secs = full_traj.getWayPointDurationFromStart(i);
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = positions;

            if (i > 0) {
                double dt = secs - full_traj.getWayPointDurationFromStart(i - 1);
                if (dt > 1e-9) {
                    point.velocities.resize(positions.size(), 0.0);
                    for (size_t j = 0; j < positions.size(); ++j)
                        point.velocities[j] = (positions[j] - prev[j]) / dt;
                }
            }
            point.time_from_start.sec = static_cast<int32_t>(secs);
            point.time_from_start.nanosec = static_cast<uint32_t>((secs - point.time_from_start.sec) * 1e9);
            dual_traj.points.push_back(point);
            prev = std::move(positions);
        }

        auto future = sendTrajectory(dual_client, dual_traj);
        auto status = future.wait_for(std::chrono::seconds(30));
        return (status == std::future_status::ready &&
                future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
    } else {
        RCLCPP_INFO(node->get_logger(), "拆分为左右臂子轨迹。");
        auto left_client = rclcpp_action::create_client<FollowJointTrajectory>(
            node, "/left_arm_controller/follow_joint_trajectory");
        auto right_client = rclcpp_action::create_client<FollowJointTrajectory>(
            node, "/right_arm_controller/follow_joint_trajectory");

        auto left_traj  = extractSubTrajectory(full_traj, left_jmg);
        auto right_traj = extractSubTrajectory(full_traj, right_jmg);

        auto lf = sendTrajectory(left_client, left_traj);
        auto rf = sendTrajectory(right_client, right_traj);

        auto ls = lf.wait_for(std::chrono::seconds(30));
        auto rs = rf.wait_for(std::chrono::seconds(30));

        return (ls == std::future_status::ready && lf.get().code == rclcpp_action::ResultCode::SUCCEEDED &&
                rs == std::future_status::ready && rf.get().code == rclcpp_action::ResultCode::SUCCEEDED);
    }
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

    RCLCPP_INFO(node->get_logger(), "MTC 双臂协同搬运规划节点已启动...");

    // 加载机器人模型获取 JointModelGroup
    Task boot_task;  // 仅用于加载 RobotModel
    boot_task.loadRobotModel(node);
    const auto& robot_model = boot_task.getRobotModel();

    const moveit::core::JointModelGroup* left_jmg  = robot_model->getJointModelGroup("left_arm");
    const moveit::core::JointModelGroup* right_jmg = robot_model->getJointModelGroup("right_arm");
    const moveit::core::JointModelGroup* dual_jmg  = robot_model->getJointModelGroup("dual_arm");

    if (!left_jmg || !right_jmg || !dual_jmg) {
        RCLCPP_FATAL(node->get_logger(), "无法获取关节模型组，请检查 SRDF 配置。");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    // IK 种子状态
    moveit::core::RobotState ik_state(robot_model);
    ik_state.setToDefaultValues();

    // ===================================================================
    // Phase 1: 双臂运动到初始夹取目标点
    // ===================================================================
    RCLCPP_INFO(node->get_logger(), "========== Phase 1: 接近目标点 ==========");

    geometry_msgs::msg::Pose left_pose1;
    left_pose1.position.x = 0.194;
    left_pose1.position.y = 0.052;
    left_pose1.position.z = 1.330;
    {
        tf2::Quaternion q;
        q.setRPY(1.669, 0.022, -0.001);
        left_pose1.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose right_pose1;
    right_pose1.position.x = 0.270;
    right_pose1.position.y = -0.014;
    right_pose1.position.z = 1.445;
    {
        tf2::Quaternion q;
        q.setRPY(-1.747, 0.020, -0.001);
        right_pose1.orientation = tf2::toMsg(q);
    }

    if (!ik_state.setFromIK(left_jmg, left_pose1, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 1 左臂 IK 求解失败。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 1 左臂 IK 求解成功。");

    if (!ik_state.setFromIK(right_jmg, right_pose1, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 1 右臂 IK 求解失败。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 1 右臂 IK 求解成功。");

    // 构建 joint-space goal
    {
        // ---- MTC 规划 ----
        auto pipeline_planner1 = std::make_shared<solvers::PipelinePlanner>(node);
        pipeline_planner1->setPlannerId("RRTConnectkConfigDefault");

        Task task1;
        task1.stages()->setName("Phase 1 - Approach");
        task1.loadRobotModel(node);
        task1.add(std::make_unique<stages::CurrentState>("Current State"));

        moveit_msgs::msg::RobotState goal_msg;
        goal_msg.is_diff = true;
        std::vector<double> vals;
        ik_state.copyJointGroupPositions(dual_jmg, vals);
        goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
        goal_msg.joint_state.position = vals;

        auto move1 = std::make_unique<stages::MoveTo>("Approach", pipeline_planner1);
        move1->setGroup("dual_arm");
        move1->setGoal(goal_msg);
        task1.add(std::move(move1));

        try {
            task1.init();
            if (!task1.plan(5)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 1 推演失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }

            auto traj = getSolutionTrajectory(task1);
            if (!traj || traj->getWayPointCount() == 0) {
                RCLCPP_ERROR(node->get_logger(), "Phase 1 无法获取轨迹。");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }

            RCLCPP_INFO(node->get_logger(), "Phase 1 轨迹: %zu 点, %.2f 秒。",
                        traj->getWayPointCount(),
                        traj->getWayPointDurationFromStart(traj->getWayPointCount() - 1));

            if (!executeDualArmTrajectory(node, *traj, dual_jmg, left_jmg, right_jmg)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 1 执行失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Phase 1 执行完成！");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Phase 1 异常: %s", e.what());
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
    }

    // ===================================================================
    // Phase 2: 停留后保持末端相对位姿，协同转移到新位置
    // ===================================================================
    RCLCPP_INFO(node->get_logger(), "========== 停留 3 秒 ==========");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(node->get_logger(), "========== Phase 2: 协同转移 ==========");

    // 计算 Phase 1 末端位姿 (FK)
    ik_state.updateLinkTransforms();
    Eigen::Isometry3d T_L1 = ik_state.getGlobalLinkTransform("left_Link7");
    Eigen::Isometry3d T_R1 = ik_state.getGlobalLinkTransform("right_Link7");

    RCLCPP_INFO(node->get_logger(), "左末端: [%.3f, %.3f, %.3f]",
                T_L1.translation().x(), T_L1.translation().y(), T_L1.translation().z());
    RCLCPP_INFO(node->get_logger(), "右末端: [%.3f, %.3f, %.3f]",
                T_R1.translation().x(), T_R1.translation().y(), T_R1.translation().z());

    // 保持相对位姿：T_left_to_right = inv(T_L) * T_R
    Eigen::Isometry3d T_left_to_right = T_L1.inverse() * T_R1;

    // 定义左臂新目标（世界坐标系位移，模拟将物体向上提升 10cm）
    Eigen::Isometry3d T_L2 = T_L1;
    T_L2.translation() += Eigen::Vector3d(0.0, 0.0, 0.10);

    // 右臂新目标 = T_L2 * T_left_to_right（保持相对位姿不变）
    Eigen::Isometry3d T_R2 = T_L2 * T_left_to_right;

    RCLCPP_INFO(node->get_logger(), "Phase 2 左末端目标: [%.3f, %.3f, %.3f]",
                T_L2.translation().x(), T_L2.translation().y(), T_L2.translation().z());
    RCLCPP_INFO(node->get_logger(), "Phase 2 右末端目标: [%.3f, %.3f, %.3f]",
                T_R2.translation().x(), T_R2.translation().y(), T_R2.translation().z());

    // Eigen::Isometry3d → geometry_msgs::msg::Pose 手动转换
    auto eigenToPose = [](const Eigen::Isometry3d& t) {
        geometry_msgs::msg::Pose p;
        p.position.x = t.translation().x();
        p.position.y = t.translation().y();
        p.position.z = t.translation().z();
        Eigen::Quaterniond q(t.linear());
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        return p;
    };
    geometry_msgs::msg::Pose left_pose2  = eigenToPose(T_L2);
    geometry_msgs::msg::Pose right_pose2 = eigenToPose(T_R2);

    // 用 Phase 1 的关节值作为 IK 种子
    if (!ik_state.setFromIK(left_jmg, left_pose2, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 2 左臂 IK 求解失败，目标可能超出工作空间。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 2 左臂 IK 求解成功。");

    if (!ik_state.setFromIK(right_jmg, right_pose2, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 2 右臂 IK 求解失败，目标可能超出工作空间。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 2 右臂 IK 求解成功。");

    // ---- MTC 规划 ----
    {
        auto pipeline_planner2 = std::make_shared<solvers::PipelinePlanner>(node);
        pipeline_planner2->setPlannerId("RRTConnectkConfigDefault");

        Task task2;
        task2.stages()->setName("Phase 2 - Coordinated Transfer");
        task2.loadRobotModel(node);
        task2.add(std::make_unique<stages::CurrentState>("Current State"));

        moveit_msgs::msg::RobotState goal_msg;
        goal_msg.is_diff = true;
        std::vector<double> vals;
        ik_state.copyJointGroupPositions(dual_jmg, vals);
        goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
        goal_msg.joint_state.position = vals;

        auto move2 = std::make_unique<stages::MoveTo>("Transfer", pipeline_planner2);
        move2->setGroup("dual_arm");
        move2->setGoal(goal_msg);
        task2.add(std::move(move2));

        try {
            task2.init();
            if (!task2.plan(5)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 2 推演失败！(可能被障碍物阻挡)");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }

            auto traj = getSolutionTrajectory(task2);
            if (!traj || traj->getWayPointCount() == 0) {
                RCLCPP_ERROR(node->get_logger(), "Phase 2 无法获取轨迹。");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }

            RCLCPP_INFO(node->get_logger(), "Phase 2 轨迹: %zu 点, %.2f 秒。",
                        traj->getWayPointCount(),
                        traj->getWayPointDurationFromStart(traj->getWayPointCount() - 1));

            if (!executeDualArmTrajectory(node, *traj, dual_jmg, left_jmg, right_jmg)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 2 执行失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Phase 2 执行完成！双臂协同搬运结束。");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Phase 2 异常: %s", e.what());
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
