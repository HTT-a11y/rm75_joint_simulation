#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

using namespace std::chrono_literals;

class MTCPlannerNode : public rclcpp::Node
{
public:
    MTCPlannerNode(const rclcpp::NodeOptions &options)
        : Node("mtc_planner_node", options)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/task_command", 10,
            std::bind(&MTCPlannerNode::taskCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "MTC 规划节点已启动，等待接收任务点...");
    }

private:
    void taskCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到任务目标，开始构建 MTC 抓取放置流水线...");
        
        // --- 1. 创建 MTC 求解器 ---
        auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(this->shared_from_this(), "ompl");
        auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1); 

        // --- 2. 初始化整个任务剧本 ---
        moveit::task_constructor::Task task("pick_and_place_task");
        task.loadRobotModel(this->shared_from_this());

        const std::string arm_group_name = "rm_group"; 
        
        // 核心亮点：不要再去瞎猜模型里的名字，直接让底层动态查询你的 URDF
        const std::string root_frame = task.getRobotModel()->getModelFrame();
        const std::string eef_name = task.getRobotModel()->getJointModelGroup(arm_group_name)->getLinkModelNames().back();
        RCLCPP_INFO(this->get_logger(), "自动推断成功！根坐标系: [%s], 末端连杆: [%s]", root_frame.c_str(), eef_name.c_str());

        // --- 3. 往剧本里添加 Stage ---
        // 阶段 0：Current State
        task.add(std::make_unique<moveit::task_constructor::stages::CurrentState>("Current State"));

        // 阶段 1：飞向抓取前置点
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("Move To Pre-Pick", sampling_planner);
            stage->setGroup(arm_group_name);
            stage->setIKFrame(eef_name);
            
            geometry_msgs::msg::Pose pre_pick_pose = *msg;
            pre_pick_pose.position.z += 0.1; // 目标上方 10cm
            
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = root_frame;
            pose_stamped.pose = pre_pick_pose;
            
            stage->setGoal(pose_stamped);
            task.add(std::move(stage));
        }

        // 阶段 2：笛卡尔直线下降去抓
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach Pick", cartesian_planner);
            stage->setGroup(arm_group_name);
            stage->setIKFrame(eef_name);
            
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = root_frame;
            direction.vector.z = -0.05; // 垂直下降
            
            stage->setDirection(direction);
            task.add(std::move(stage));
        }

        // 阶段 3：笛卡尔直线上升提起
        {
            auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("Lift Object", cartesian_planner);
            stage->setGroup(arm_group_name);
            stage->setIKFrame(eef_name);
            
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = root_frame;
            direction.vector.z = 0.15; // 垂直上升
            
            stage->setDirection(direction);
            task.add(std::move(stage));
        }

        // --- 4. 执行规划与发布 ---
        try {
            RCLCPP_INFO(this->get_logger(), "剧本编写完毕，开始在头脑中推演 (Plan)...");
            if (task.plan(1)) { 
                RCLCPP_INFO(this->get_logger(), "推演成功！开始向底层发送执行指令 (Execute)...");
                task.execute(*task.solutions().front());
                RCLCPP_INFO(this->get_logger(), "执行完成。");
            } else {
                RCLCPP_ERROR(this->get_logger(), "推演失败，机械臂在某个阶段卡住了。");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MTC 报错: %s", e.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    
    auto mtc_node = std::make_shared<MTCPlannerNode>(options);

    // MTC 强烈建议使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
