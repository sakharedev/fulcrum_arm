// ROS
#include <rclcpp/node.hpp>

// Moveit
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <pick_place_parameter.hpp>

#pragma once

namespace fulcrumArm_memento {
    using namespace moveit::task_constructor;

    // Prepare a demo env for ROS parameters under Node
    void setupMementoScene(const memento::Params& params);

    class MementoTask
    {
        public:
            MementoTask(const std::string& task_name);
            ~MementoTask() = default;

            bool init(const rclcpp::Node::SharedPtr& node, const memento::Params& params);

            bool plan(const std::size_t max_solutions);

            bool execute();

        private:
            sts::string task_name_;
            moveit::task_constructor::TaskPtr task_;
    };
}  // namespace fulcrumArm_memento