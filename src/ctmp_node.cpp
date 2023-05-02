/*
 * Copyright (C) 2023, Itamar Mishani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   actionSpace.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/20/23
*/

#include <ctmp/zero_time_planner.hpp>

#include <ros/ros.h>
// include tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



int main(int argc, char** argv) {

    ros::init(argc, argv, "ctmp_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Before I will load it with launch file, lets set params to ROS server of the limits of the goal region:
//    ros::param::set("/ctmp_test/pick", false);
//    ros::param::set("/ctmp_test/regions/pick_region/min_limits",
//                    std::vector<double>{-0.5, 0.4, 0.9, 0, 0, 0.0});
//    ros::param::set("/ctmp_test/regions/pick_region/max_limits",
//                    std::vector<double>{0.4, 0.9, 0.9, 0, 0, 0.0});
//    ros::param::set("/ctmp_test/regions/place_region/min_limits",
//                    std::vector<double>{0.7, -0.25, 0.75, 0, 0, 0});
//    ros::param::set("/ctmp_test/regions/place_region/max_limits",
//                    std::vector<double>{1.2, 0.25, 0.75, 0, 0, 0});

    ros::param::set("/manipulator_2/pick", true);
    ros::param::set("/manipulator_2/regions/pick_region/min_limits",
                    std::vector<double>{1.60, -1.24, 0.75 , 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_2/regions/pick_region/max_limits",
                    std::vector<double>{2.0, -0.74, 0.75, 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_2/regions/place_region/min_limits",
                    std::vector<double>{0.7, -0.25, 0.7, 0.0, 0.0, 1.570796});
    ros::param::set("/manipulator_2/regions/place_region/max_limits",
                    std::vector<double>{1.2, 0.25, 0.7, 0.0, 0.0, 1.570796});

//    ros::param::set("/ctmp_test/pick", true);
//    ros::param::set("/ctmp_test/regions/pick_region/min_limits",
//                    std::vector<double>{1.60, 0.74, 0.80 , 0.0, 0.0, 0.0});
//    ros::param::set("/ctmp_test/regions/pick_region/max_limits",
//                    std::vector<double>{1.90, 1.24, 0.80, 0.0, 0.0, 0.0});
//    ros::param::set("/ctmp_test/regions/place_region/min_limits",
//                    std::vector<double>{0.7, -0.25, 0.74, 0.0, 0.0, -1.570796});
//    ros::param::set("/ctmp_test/regions/place_region/max_limits",
//                    std::vector<double>{1.2, 0.25, 0.74, 0.0, 0.0, -1.570796});
    // get manipulation_planning package path
    auto full_path = ros::package::getPath("manipulation_planning");
    std::string path_mprim = full_path + "/config/ws.mprim";

    // Define Robot inteface to give commands and get info from moveit:
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_2");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Test AStar in configuration space
    // @{
    auto* heuristic = new ims::SE3HeuristicRPY;
    ims::BestFirstSearchParams params(heuristic);

    ims::MoveitInterface scene_interface ("manipulator_2");
    ims::ctmpActionType action_type(path_mprim);

    stateType discretization {0.02, 0.02, 0.02,
                              M_PI/180,
                              M_PI/180,
                              M_PI/180};
    action_type.Discretization(discretization);
    action_type.setSpaceType(ims::manipulationType::spaceType::WorkSpace); // Already the default

    std::shared_ptr<ims::ctmpActionSpace> action_space = std::make_shared<ims::ctmpActionSpace>(scene_interface, action_type);


    stateType start_state {0, 0, 0, 0, 0, 0};
    // get the current end effector pose
    // go to "ready" pose first
    move_group.setNamedTarget("ready2");
    move_group.move();
    ros::Duration(1).sleep();

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();  // "arm_1tool0"

    start_state[0] = current_pose.pose.position.x;
    start_state[1] = current_pose.pose.position.y;
    start_state[2] = current_pose.pose.position.z;

    // If using hopf coordinates:
    Eigen::Quaterniond current_pose_eigen;
    tf::quaternionMsgToEigen(current_pose.pose.orientation, current_pose_eigen);

    ims::get_euler_zyx(current_pose_eigen, start_state[5], start_state[4], start_state[3]);
    ims::normalize_euler_zyx(start_state[5], start_state[4], start_state[3]);

//    stateType goal_state = {0.2, 0.5, 0.9, -0, 0, 0};
    stateType goal_state = {0.8, -0.1, 0.75, 0, 0, 0};
    // discrtize the goal state
    for (int i = 0; i < 6; i++) {
        goal_state[i] = std::round(goal_state[i] / discretization[i]) * discretization[i];
        start_state[i] = std::round(start_state[i] / discretization[i]) * discretization[i];
    }
    Eigen::Quaterniond start_pose_eigen;
    ims::from_euler_zyx(start_state[5], start_state[4], start_state[3], start_pose_eigen);
    geometry_msgs::Pose pose_check;
    pose_check.position.x = start_state[0]; pose_check.position.y = start_state[1]; pose_check.position.z = start_state[2];
    tf::quaternionEigenToMsg(start_pose_eigen, pose_check.orientation);


    std::cout << "rounded pose " << pose_check.position.x << " " << pose_check.position.y << " " << pose_check.position.z << " "
              << pose_check.orientation.x << " " << pose_check.orientation.y << " " << pose_check.orientation.z << " " << pose_check.orientation.w << std::endl;

    std::cout << "original pose " << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << current_pose.pose.position.z << " "
              << current_pose.pose.orientation.x << " " << current_pose.pose.orientation.y << " " << current_pose.pose.orientation.z << " " << current_pose.pose.orientation.w << std::endl;

    // check if the inverse kinematics solution exists for the current pose and check if the solution is equal to the current joint state
    std::vector<double> current_joint_state = move_group.getCurrentJointValues();
    std::vector<double> ik_solution;

    if (!scene_interface.calculateIK(current_pose.pose, current_joint_state, ik_solution)) {
        std::cout << "No IK solution for the current pose" << std::endl;
        return 0;
    }
    else {
        ims::rad2deg(ik_solution); ims::rad2deg(current_joint_state);
        std::cout << "IK solution for the current pose" << std::endl;
        for (int i = 0; i < ik_solution.size(); i++) {
            std::cout << "joint " << i << " " << ik_solution[i] << " " << current_joint_state[i] << std::endl;
        }
    }
    std::cout << "start state " << start_state[0] << " " << start_state[1] << " " << start_state[2] << " "
              << start_state[3] << " " << start_state[4] << " " << start_state[5] << std::endl;
    std::cout << "goal state " << goal_state[0] << " " << goal_state[1] << " " << goal_state[2] << " "
              << goal_state[3] << " " << goal_state[4] << " " << goal_state[5] << std::endl;

    bool query_mode = false;
    ZeroTimePlanner planner;
    if (!planner.initializePlanner(action_space, params,
                                  start_state, goal_state,
                                   query_mode, "RRTConnect")){
        //raise error
        std::cout << "Planner initialization failed" << std::endl;
        return 0;
    }

    std::vector<stateType> path_;
    if (!planner.plan(path_)) {
        if (query_mode)
            std::cout << "No path found" << std::endl;
        else
            std::cout << "\n" <<"Finished Preprocessing" << std::endl;
        return 0;
    }
    else {
        std::cout << "Path found" << std::endl;
    }


    std::vector<std::vector<double>> path_ee;
    // @}
    // Print nicely the path
    for (auto& state : path_) {
        for (auto& val : state) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }
    // delete the last state
//    path_.pop_back();

    // profile and execute the path
    // @{
    // execute in the joint space
    moveit_msgs::RobotTrajectory trajectory;
    ims::profileTrajectory(path_.at(0),
                           path_.back(),
                           path_,
                           move_group,
                           trajectory);

    move_group.execute(trajectory);

    // rerport stats
//    plannerStats stats = planner.reportStats();
//    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
//    std::cout << "cost: " << stats.cost << std::endl;
//    std::cout << "Path length: " << path_.size() << std::endl;
//    std::cout << "Number of nodes expanded: " << stats.numExpanded << std::endl;
//    std::cout << "Suboptimality: " << stats.subOptimality << RESET << std::endl;
    // @}


//    std::vector<geometry_msgs::Pose> waypoints;
//    for (auto& state : path_) {
//        geometry_msgs::Pose pose;
//        pose.position.x = state[0];
//        pose.position.y = state[1];
//        pose.position.z = state[2];
//        Eigen::Quaterniond quat_res;
//        ims::from_euler_zyx(state[5], state[4], state[3], quat_res);
//        pose.orientation.x = quat_res.x(); pose.orientation.y = quat_res.y();
//        pose.orientation.z = quat_res.z(); pose.orientation.w = quat_res.w();
//        waypoints.push_back(pose);
//    }
//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
//    std::cout << "fraction: " << fraction << std::endl;
//    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//    my_plan.trajectory_ = trajectory;
//    move_group.execute(my_plan);

    // rerport stats


    return 0;
}