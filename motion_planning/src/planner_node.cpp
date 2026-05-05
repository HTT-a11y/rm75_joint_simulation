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

// ---------------------------------------------------------------------------
// 工具函数
// ---------------------------------------------------------------------------

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

// 发送轨迹到指定控制器
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

// 执行 dual_arm 轨迹：优先 dual_arm_controller，否则拆分为左右臂
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

// Eigen::Isometry3d → geometry_msgs::msg::Pose
geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& t)
{
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
}

// ---------------------------------------------------------------------------
// 协同运动阶段：保持左右末端相对位姿, 按位移生成新目标, IK, 规划, 执行
// ---------------------------------------------------------------------------
bool executeCoordinatedPhase(
    rclcpp::Node::SharedPtr node,
    const std::string& phase_name,
    moveit::core::RobotState& ik_state,
    const moveit::core::JointModelGroup* left_jmg,
    const moveit::core::JointModelGroup* right_jmg,
    const moveit::core::JointModelGroup* dual_jmg,
    const Eigen::Vector3d& displacement)
{
    RCLCPP_INFO(node->get_logger(), "========== %s ==========", phase_name.c_str());

    // FK
    ik_state.updateLinkTransforms();
    Eigen::Isometry3d T_L = ik_state.getGlobalLinkTransform("left_Link7");
    Eigen::Isometry3d T_R = ik_state.getGlobalLinkTransform("right_Link7");

    RCLCPP_INFO(node->get_logger(), "当前左末端: [%.3f, %.3f, %.3f]  右末端: [%.3f, %.3f, %.3f]",
                T_L.translation().x(), T_L.translation().y(), T_L.translation().z(),
                T_R.translation().x(), T_R.translation().y(), T_R.translation().z());

    // 相对位姿
    Eigen::Isometry3d T_left_to_right = T_L.inverse() * T_R;

    // 新目标
    Eigen::Isometry3d T_L_new = T_L;
    T_L_new.translation() += displacement;
    Eigen::Isometry3d T_R_new = T_L_new * T_left_to_right;

    RCLCPP_INFO(node->get_logger(), "左末端目标:  [%.3f, %.3f, %.3f]  右末端目标:  [%.3f, %.3f, %.3f]",
                T_L_new.translation().x(), T_L_new.translation().y(), T_L_new.translation().z(),
                T_R_new.translation().x(), T_R_new.translation().y(), T_R_new.translation().z());

    // IK
    geometry_msgs::msg::Pose left_pose  = eigenToPose(T_L_new);
    geometry_msgs::msg::Pose right_pose = eigenToPose(T_R_new);

    if (!ik_state.setFromIK(left_jmg, left_pose, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "%s 左臂 IK 求解失败，目标可能超出工作空间。", phase_name.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "%s 左臂 IK 求解成功。", phase_name.c_str());

    if (!ik_state.setFromIK(right_jmg, right_pose, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "%s 右臂 IK 求解失败，目标可能超出工作空间。", phase_name.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "%s 右臂 IK 求解成功。", phase_name.c_str());

    // MTC 规划 + 执行
    auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
    pipeline_planner->setPlannerId("RRTConnectkConfigDefault");

    Task task;
    task.stages()->setName(phase_name);
    task.loadRobotModel(node);
    task.add(std::make_unique<stages::CurrentState>("Current State"));

    moveit_msgs::msg::RobotState goal_msg;
    goal_msg.is_diff = true;
    std::vector<double> vals;
    ik_state.copyJointGroupPositions(dual_jmg, vals);
    goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
    goal_msg.joint_state.position = vals;

    auto move = std::make_unique<stages::MoveTo>("Move", pipeline_planner);
    move->setGroup("dual_arm");
    move->setGoal(goal_msg);
    task.add(std::move(move));

    try {
        task.init();
        if (!task.plan(5)) {
            RCLCPP_ERROR(node->get_logger(), "%s 推演失败！(可能发生碰撞)", phase_name.c_str());
            return false;
        }

        auto traj = getSolutionTrajectory(task);
        if (!traj || traj->getWayPointCount() == 0) {
            RCLCPP_ERROR(node->get_logger(), "%s 无法获取轨迹。", phase_name.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "%s 轨迹: %zu 点, %.2f 秒。",
                    phase_name.c_str(),
                    traj->getWayPointCount(),
                    traj->getWayPointDurationFromStart(traj->getWayPointCount() - 1));

        if (!executeDualArmTrajectory(node, *traj, dual_jmg, left_jmg, right_jmg)) {
            RCLCPP_ERROR(node->get_logger(), "%s 执行失败！", phase_name.c_str());
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "%s 执行完成！", phase_name.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "%s 异常: %s", phase_name.c_str(), e.what());
        return false;
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
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
    Task boot_task;
    boot_task.loadRobotModel(node);
    const auto& robot_model = boot_task.getRobotModel();

    const moveit::core::JointModelGroup* left_jmg  = robot_model->getJointModelGroup("left_arm");
    const moveit::core::JointModelGroup* right_jmg = robot_model->getJointModelGroup("right_arm");
    const moveit::core::JointModelGroup* dual_jmg  = robot_model->getJointModelGroup("dual_arm");

    if (!left_jmg || !right_jmg || !dual_jmg) {
        RCLCPP_FATAL(node->get_logger(), "无法获取关节模型组，请检查 SRDF 配置。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // IK 种子状态
    moveit::core::RobotState ik_state(robot_model);
    ik_state.setToDefaultValues();

    // =======================================================================
    // Phase 1: 接近 (Approach) —— 双臂运动到初始夹取目标点
    // =======================================================================
    RCLCPP_INFO(node->get_logger(), "========== Phase 1: 接近 (Approach) ==========");

    geometry_msgs::msg::Pose left_pose_approach;
    left_pose_approach.position.x = 0.194;
    left_pose_approach.position.y = 0.052;
    left_pose_approach.position.z = 1.330;
    {
        tf2::Quaternion q; q.setRPY(1.669, 0.022, -0.001);
        left_pose_approach.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose right_pose_approach;
    right_pose_approach.position.x = 0.270;
    right_pose_approach.position.y = -0.014;
    right_pose_approach.position.z = 1.445;
    {
        tf2::Quaternion q; q.setRPY(-1.747, 0.020, -0.001);
        right_pose_approach.orientation = tf2::toMsg(q);
    }

    if (!ik_state.setFromIK(left_jmg, left_pose_approach, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 1 左臂 IK 求解失败。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 1 左臂 IK 求解成功。");

    if (!ik_state.setFromIK(right_jmg, right_pose_approach, 0.1)) {
        RCLCPP_ERROR(node->get_logger(), "Phase 1 右臂 IK 求解失败。");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Phase 1 右臂 IK 求解成功。");

    {
        auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
        pipeline_planner->setPlannerId("RRTConnectkConfigDefault");

        Task task;
        task.stages()->setName("Phase 1 - Approach");
        task.loadRobotModel(node);
        task.add(std::make_unique<stages::CurrentState>("Current State"));

        moveit_msgs::msg::RobotState goal_msg;
        goal_msg.is_diff = true;
        std::vector<double> vals;
        ik_state.copyJointGroupPositions(dual_jmg, vals);
        goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
        goal_msg.joint_state.position = vals;

        auto move1 = std::make_unique<stages::MoveTo>("Approach", pipeline_planner);
        move1->setGroup("dual_arm");
        move1->setGoal(goal_msg);
        task.add(std::move(move1));

        try {
            task.init();
            if (!task.plan(5)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 1 推演失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            auto traj = getSolutionTrajectory(task);
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

    // =======================================================================
    // Phase 2: 闭合 (Grasp) —— 停留模拟夹具闭合夹取
    // =======================================================================
    RCLCPP_INFO(node->get_logger(), "========== Phase 2: 闭合 (Grasp) 停留 2 秒 ==========");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // =======================================================================
    // Phase 3: 提起 (Lift) —— 保持相对位姿，向上提起 10cm
    // =======================================================================
    if (!executeCoordinatedPhase(node, "Phase 3: 提起 (Lift)",
                                 ik_state, left_jmg, right_jmg, dual_jmg,
                                 Eigen::Vector3d(0.0, 0.0, 0.10))) {
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // =======================================================================
    // Phase 4: 转移 (Transfer) —— 保持相对位姿，平移至新位置
    // =======================================================================
    if (!executeCoordinatedPhase(node, "Phase 4: 转移 (Transfer)",
                                 ik_state, left_jmg, right_jmg, dual_jmg,
                                 Eigen::Vector3d(-0.04, 0.02, 0.0))) {
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // =======================================================================
    // Phase 5: 放置 (Place) —— 保持相对位姿，下降放物
    // =======================================================================
    if (!executeCoordinatedPhase(node, "Phase 5: 放置 (Place)",
                                 ik_state, left_jmg, right_jmg, dual_jmg,
                                 Eigen::Vector3d(0.0, 0.0, -0.08))) {
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }

    // =======================================================================
    // Phase 6: 复位 (Retract) —— 双臂回到 SRDF home 位姿
    // =======================================================================
    RCLCPP_INFO(node->get_logger(), "========== Phase 6: 复位 (Retract) ==========");

    ik_state.setToDefaultValues(left_jmg, "home");
    ik_state.setToDefaultValues(right_jmg, "home");

    {
        auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node);
        pipeline_planner->setPlannerId("RRTConnectkConfigDefault");

        Task task;
        task.stages()->setName("Phase 6 - Retract");
        task.loadRobotModel(node);
        task.add(std::make_unique<stages::CurrentState>("Current State"));

        moveit_msgs::msg::RobotState goal_msg;
        goal_msg.is_diff = true;
        std::vector<double> vals;
        ik_state.copyJointGroupPositions(dual_jmg, vals);
        goal_msg.joint_state.name    = dual_jmg->getActiveJointModelNames();
        goal_msg.joint_state.position = vals;

        auto move6 = std::make_unique<stages::MoveTo>("Retract", pipeline_planner);
        move6->setGroup("dual_arm");
        move6->setGoal(goal_msg);
        task.add(std::move(move6));

        try {
            task.init();
            if (!task.plan(5)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 6 推演失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            auto traj = getSolutionTrajectory(task);
            if (!traj || traj->getWayPointCount() == 0) {
                RCLCPP_ERROR(node->get_logger(), "Phase 6 无法获取轨迹。");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Phase 6 轨迹: %zu 点, %.2f 秒。",
                        traj->getWayPointCount(),
                        traj->getWayPointDurationFromStart(traj->getWayPointCount() - 1));
            if (!executeDualArmTrajectory(node, *traj, dual_jmg, left_jmg, right_jmg)) {
                RCLCPP_ERROR(node->get_logger(), "Phase 6 执行失败！");
                rclcpp::shutdown(); spin_thread.join(); return 1;
            }
            RCLCPP_INFO(node->get_logger(), "Phase 6 执行完成！");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Phase 6 异常: %s", e.what());
            rclcpp::shutdown(); spin_thread.join(); return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(), "全部 6 个阶段执行完毕！");
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
