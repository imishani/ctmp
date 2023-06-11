//
// Created by itamar on 5/28/23.
//

/// Standard includes
#include <queue>
#include <numeric>
#include <unordered_set>
/// include package for ROS
#include <ros/ros.h>
#include <ros/package.h>

/// inlclude package for moveit planning
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>

/// include package for moveit msgs
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include <manipulation_planning/common/utils.hpp>
#include <utility>


struct State{
    moveit_msgs::RobotState state;
    int depth;
    bool visited = false;
    State(moveit_msgs::RobotState  state, int depth) : state(std::move(state)), depth(depth) {}
    bool operator==(const State& other) const {
        return state == other.state;
    }
};
// hash function for State by the state values  (joint values)
struct StateHash{
    std::size_t operator()(const State& state) const {
        std::size_t seed = 0;
        for (auto& value : state.state.joint_state.position){
            seed ^= std::hash<double>()(value) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};



/// @brief For a given state, check for reachability using breadth-first search.
/// @return The radius of the reachability hypersphere.
double reachabilityState(const moveit_msgs::RobotState& state,
                         planning_scene::PlanningScene& planning_scene,
                         moveit::planning_interface::MoveGroupInterface& move_group){
    // get the planning scene
//    moveit_msgs::PlanningScene planning_scene;
//    planning_scene.robot_state = state;
    moveit::core::RobotState robot_state(move_group.getRobotModel());
    double radius = 0.0;
    double max_radius = 100 * M_PI / 180;
    int index = 0;
    std::vector<double> discretization(state.joint_state.position.size(), 20);
    ims::deg2rad(discretization);
    // open list as FIFO queue
    std::queue<std::pair<std::shared_ptr<State>, int>> open_list;
    State start_state(state, 0);
    open_list.emplace(std::make_shared<State>(start_state), 0);
    // closed list as hash set
    std::unordered_set<std::shared_ptr<State>, StateHash> closed_list;

    bool state_valid = true;
    size_t count = 0;
    std::vector<std::chrono::steady_clock::duration> for_loop_avg;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (state_valid && radius < max_radius){
        // get the next state from the queue
        auto current_state = open_list.front();
        open_list.pop();
        count++;


//        if (current_state.second > index){
//            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//            std::cout << "Radius: " << radius << " Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
//            begin = std::chrono::steady_clock::now();
//            index = current_state.second;
//        }
        std::chrono::steady_clock::time_point begin_forLoop = std::chrono::steady_clock::now();
        // generate successor states
        for(size_t i = 0; i < current_state.first->state.joint_state.position.size(); i++){
            moveit_msgs::RobotState successor = current_state.first->state;
            successor.joint_state.position[i] += discretization[i];
            State successor_state(successor, current_state.second + 1);

            // check if the succ is in the hyperball
            double d = 0.0;
            for (size_t j = 0; j < successor.joint_state.position.size(); j++){
                d += std::pow(successor.joint_state.position[j] - state.joint_state.position[j], 2);
            }
            if (d > std::pow(max_radius, 2))
                continue;

            // check if the successor values are the same of any state in the closed list
//            if (closed_list.find(successor_state) != closed_list.end())
//                continue;

            robot_state.setVariablePositions(successor.joint_state.name, successor.joint_state.position);
            robot_state.update();
//            std::chrono::steady_clock::time_point begin_collision = std::chrono::steady_clock::now();

            if (!planning_scene.isStateValid(robot_state, "manipulator")){
                state_valid = false;
                break;
            }
//            std::chrono::steady_clock::time_point end_collision = std::chrono::steady_clock::now();
//            std::cout << "Collision check time = " << std::chrono::duration_cast<std::chrono::microseconds>(end_collision - begin_collision).count() <<std::endl;
            open_list.emplace(std::make_shared<State>(successor_state), current_state.second + 1);
            successor = current_state.first->state;
            successor.joint_state.position[i] -= discretization[i];
            robot_state.setVariablePositions(successor.joint_state.name, successor.joint_state.position);
            robot_state.update();
            State successor_state2(successor, current_state.second + 1);
            if (!planning_scene.isStateValid(robot_state, "manipulator")){
                state_valid = false;
                break;
            }
            open_list.emplace(std::make_shared<State>(successor_state2), current_state.second + 1);
        }
        std::chrono::steady_clock::time_point end_forLoop = std::chrono::steady_clock::now();
        for_loop_avg.push_back(end_forLoop - begin_forLoop);
        // if all successors are valid, increase the radius
        radius = std::max(current_state.second - 1, 0) * discretization[0];
        if (!state_valid){
            radius = std::max(current_state.second - 1, 0) * discretization[0];
            break;
        } else {
            radius = (current_state.second + 1) * discretization[0];
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
    std::cout << "Count = " << count << std::endl;
    std::cout << "Average time in for loop: " << std::chrono::duration_cast<std::chrono::microseconds>(std::accumulate(for_loop_avg.begin(), for_loop_avg.end(), std::chrono::steady_clock::duration(0)) / for_loop_avg.size()).count() << std::endl;
//    std::cout << "Branching factor = " << (1 + 12 + 12*12 + 12*12*12) << std::endl;
//    std::cout << "Branching factor = " << (1 + 12 + 12*12 + 12*12*12 + 12*12*12*12) << std::endl;
//    std::cout << "Branching factor = " << (1 + 12 + 12*12 + 12*12*12 + 12*12*12*12 + 12*12*12*12*12) << std::endl;
//    std::cout << "Branching factor = " << (1 + 12 + 12*12 + 12*12*12 + 12*12*12*12 + 12*12*12*12*12 + 12*12*12*12*12*12) << std::endl;
    return radius;
}


/// @brief For a given trajectory, check for each state a reachability hypersphere using breadth-first search.
/// When a state is invalid, terminate and BFS and cache the radius of the hypersphere.
void reachabilityTraj(planning_scene::PlanningScene& planning_scene,
                      moveit::planning_interface::MoveGroupInterface& move_group,
                      moveit_msgs::RobotTrajectory& trajectory,
                      std::vector<double>& sphs_radius){
    // loop through the trajectory points, and check using BFS the radius of reachability
    for (auto & point : trajectory.joint_trajectory.points){
        moveit_msgs::RobotState state;
        state.joint_state.name = trajectory.joint_trajectory.joint_names;
        state.joint_state.position = point.positions;
        double radius = reachabilityState(state, planning_scene, move_group);
        sphs_radius.push_back(radius);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "hyperballs_hypothesis");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /// create moveit objects
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // planning scene
    planning_scene::PlanningScene planning_scene(move_group.getRobotModel());
    ros::Duration(1.0).sleep();
    /// get the name of the planning group for this robot
    std::string planning_group = move_group.getName();

    /// get the name of the reference frame for this robot
    std::string reference_frame = move_group.getPlanningFrame();

    /// get the name of the end-effector link for this group
    std::string end_effector_link = move_group.getEndEffectorLink();

    /// set the planner
    move_group.setPlannerId("BiTRRT");

    /// get the current state of the robot an use it as an initial state in the planning
    robot_state::RobotState start_state(*move_group.getCurrentState());

    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // get a random goal state
    while (!success){
        move_group.setRandomTarget();
        success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    // create a trajectory object
    moveit_msgs::RobotTrajectory trajectory1;
    trajectory1 = my_plan.trajectory_;

    // create a second plan to a new random goal state
    bool success2 = false;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // get a random goal state
    while (!success2){
        move_group.setRandomTarget();
        success2 = (move_group.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    }
    // create a trajectory object
    moveit_msgs::RobotTrajectory trajectory2;
    trajectory2 = my_plan2.trajectory_;
    // loop through the trajectory points, and check using BFS the radius of reachability
    std::vector<double> sphs_radius1;
    std::vector<double> sphs_radius2;
    reachabilityTraj(planning_scene, move_group, trajectory1, sphs_radius1);

    reachabilityTraj(planning_scene, move_group, trajectory2, sphs_radius2);

    // create a new trajectory as follows:
    // 1. Push trajectory1 to the new trajectory
    // 2. for each point in the reversed trajectory1, check if the distance to the closest point in trajectory2. If the distance
    //    is less than the sum of the radiuses of the reachability hyperspheres, add the reversed trajectory up to this point
    //    to the new trajectory
    // 3. add a linear interpolation between the last point in the new trajectory and the point we found in step 2 (on trajectory 2, if any)
    // 4. add the rest of trajectory2 to the new trajectory
    moveit_msgs::RobotTrajectory new_trajectory;
    new_trajectory.joint_trajectory.joint_names = trajectory1.joint_trajectory.joint_names;
    new_trajectory.joint_trajectory.header = trajectory1.joint_trajectory.header;
    new_trajectory.joint_trajectory.points = trajectory1.joint_trajectory.points;
    // reverse trajectory1
    std::reverse(trajectory1.joint_trajectory.points.begin(), trajectory1.joint_trajectory.points.end());
    // loop through the trajectory points, and check the radius of reachability
    for (int i {0}; i < trajectory1.joint_trajectory.points.size(); i++){
        // get the closest point in trajectory2
        double min_dist {std::numeric_limits<double>::max()};
        int min_index {-1};
        for (int j {(int)trajectory2.joint_trajectory.points.size() - 1}; j >= 0; j--){
            double dist {0};
            for (int k {0}; k < trajectory1.joint_trajectory.points[i].positions.size(); k++){
                dist += std::pow(trajectory1.joint_trajectory.points[i].positions[k] - trajectory2.joint_trajectory.points[j].positions[k], 2);
            }
            dist = std::sqrt(dist);
            if (dist < min_dist){
                min_dist = dist;
                min_index = j;
            }
        }
        // Check if the distance is less than the sum of the radiis of the reachability hyperspheres
        bool overlap = (min_dist < sphs_radius1[i] + sphs_radius2[min_index]);
        if (overlap){
            // print the indexes of the overlapping points
            std::cout << "Overlap between point " << i << " in trajectory 1 and point " << min_index << " in trajectory 2" << std::endl;
            std::cout << "Distance: " << min_dist << std::endl;
            // add the reversed trajectory up to this point to the new trajectory
            for (int l {0}; l <= i; l++){
                new_trajectory.joint_trajectory.points.push_back(trajectory1.joint_trajectory.points[l]);
                // reverse the trajectory back (velocity and acceleration should be reversed)
                for (int k {0}; k < trajectory1.joint_trajectory.points[l].velocities.size(); k++){
                    new_trajectory.joint_trajectory.points.back().velocities[k] *= -1;
                    new_trajectory.joint_trajectory.points.back().accelerations[k] *= -1;
                }
            }
            // add a linear interpolation between the last point in the new trajectory and the point we found in step 2 (on trajectory 2, if any)
            if (min_index != -1){
                // create a new point
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions.resize(trajectory1.joint_trajectory.points[i].positions.size());
                point.velocities.resize(trajectory1.joint_trajectory.points[i].velocities.size());
                point.accelerations.resize(trajectory1.joint_trajectory.points[i].accelerations.size());
//                point.effort.resize(trajectory1.joint_trajectory.points[i].effort.size());
                // interpolate
                double alpha {0.5};
                for (int k {0}; k < trajectory1.joint_trajectory.points[i].positions.size(); k++){
                    point.positions[k] = alpha * trajectory1.joint_trajectory.points[i].positions[k] + (1 - alpha) * trajectory2.joint_trajectory.points[min_index].positions[k];
                    point.velocities[k] = alpha * trajectory1.joint_trajectory.points[i].velocities[k] + (1 - alpha) * trajectory2.joint_trajectory.points[min_index].velocities[k];
                    point.accelerations[k] = alpha * trajectory1.joint_trajectory.points[i].accelerations[k] + (1 - alpha) * trajectory2.joint_trajectory.points[min_index].accelerations[k];
//                    point.effort[k] = alpha * trajectory1.joint_trajectory.points[i].effort[k] + (1 - alpha) * trajectory2.joint_trajectory.points[min_index].effort[k];
                }
                // add the point to the new trajectory
                new_trajectory.joint_trajectory.points.push_back(point);
            }
            // add the rest of trajectory2 to the new trajectory
            for (int j {min_index}; j < trajectory2.joint_trajectory.points.size(); j++){
                new_trajectory.joint_trajectory.points.push_back(trajectory2.joint_trajectory.points[j]);
            }
            break;
        }
    }

    if (new_trajectory.joint_trajectory.points.size() > trajectory1.joint_trajectory.points.size()){
        // create a new plan
        moveit::planning_interface::MoveGroupInterface::Plan new_plan;
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), new_trajectory);
        // reprofile the trajectory
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 1.0, 1.0);
        rt.getRobotTrajectoryMsg(new_plan.trajectory_);
        // execute the plan
        move_group.execute(new_plan);
    }
    else{
        ROS_INFO("No overlap found");
    }

    spinner.stop();
    return 0;
}

