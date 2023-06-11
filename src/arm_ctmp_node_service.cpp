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

// ROS includes
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <ctmp/ctmpAction.h>


class ctmpActionServer
{
protected:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<std::shared_ptr<actionlib::SimpleActionServer<ctmp::ctmpAction>>> as_; // NodeHandle instance must be created before this line.
                                                        // Otherwise strange error occurs.
    // create messages that are used to published feedback/result
    ctmp::ctmpActionFeedback feedback_;
    ctmp::ctmpResult result_;
    std::vector<moveit::planning_interface::MoveGroupInterfacePtr> move_groups;
    stateType discretization {};
    std::vector<std::string> robot_names;
    std::vector<std::shared_ptr<ims::MoveitInterface>> scene_interfaces;
    ims::ctmpActionType action_type;
    std::vector<std::shared_ptr<ims::ctmpActionSpace>> action_spaces;

public:

    ctmpActionServer(const std::vector<std::string>& robot_names,
                     ros::NodeHandle& nh,
                     ros::NodeHandle& pnh
                     ) :
            nh_(nh),
            pnh_(pnh),
            robot_names(robot_names)
    {

        // get manipulation_planning package path
        auto full_path = ros::package::getPath("manipulation_planning");
        std::string path_mprim = full_path + "/config/ws.mprim";

        action_type = ims::ctmpActionType(path_mprim);

        discretization = {0.02, 0.02, 0.02,
                          M_PI/180, M_PI/180, M_PI/180};
        action_type.Discretization(discretization);
        action_type.setSpaceType(ims::manipulationType::spaceType::WorkSpace); // Already the default

        // go to "ready" pose first
        // get the last char as string from the robot name
        for (auto& robot_name : robot_names) {
            // register the goal and feeback callbacks
            as_.emplace_back(std::make_shared<actionlib::SimpleActionServer<ctmp::ctmpAction>>(nh_,
                    robot_name + "/ctmp_action",
                    boost::bind(&ctmpActionServer::executeCB, this, _1), false));
            move_groups.emplace_back(std::make_shared<moveit::planning_interface::MoveGroupInterface>(robot_name));
            scene_interfaces.emplace_back(std::make_shared<ims::MoveitInterface>(robot_name));
            action_spaces.emplace_back(std::make_shared<ims::ctmpActionSpace>(*scene_interfaces.back(), action_type));
            move_groups.back()->setNamedTarget("ready" + robot_name.substr(robot_name.size() - 1));
            move_groups.back()->move();
            ros::Duration(0.1).sleep();
            as_.back()->start();
        }
//        as_.start();
    }

    ~ctmpActionServer() = default;

    bool getObjPose(const std::string& obj_name,
                    std::vector<double>& obj_pose)
    {
        // look up the object pose
        ROS_INFO("Getting object pose");
//        auto collision_objects = scene_interface.mPlanningSceneInterface->getObjects();
        auto planning_interface = moveit::planning_interface::PlanningSceneInterface();
        auto collision_objects = planning_interface.getObjects();
        ROS_INFO("Number of objects: %zu", collision_objects.size());
        for (auto& collision_obj : collision_objects) {
            ROS_INFO("Object name: %s", collision_obj.second.id.c_str());
            if (collision_obj.second.id == obj_name) {
                auto object_pose = collision_obj.second.pose;
                // get the relative pose of the object from param server
                geometry_msgs::Pose obj_pose_relative;
                if (!nh_.getParam(obj_name + "/relative_pose/position/x", obj_pose_relative.position.x)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/position/y", obj_pose_relative.position.y)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/position/z", obj_pose_relative.position.z)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/orientation/x", obj_pose_relative.orientation.x)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/orientation/y", obj_pose_relative.orientation.y)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/orientation/z", obj_pose_relative.orientation.z)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                if (!nh_.getParam(obj_name + "/relative_pose/orientation/w", obj_pose_relative.orientation.w)) {
                    ROS_ERROR_STREAM("Failed to get the relative pose of the object");
                    return false;
                }
                // transform the relative pose to the absolute pose
                ims::convertFromRelativePose(object_pose, obj_pose_relative, obj_pose);
                return true;
            }
        }
        return false;
    }

    void executeCB(const ctmp::ctmpGoalConstPtr &goal)
    {
        // helper variables
        bool success = true;
        // get the robot name
        std::string robot = goal->robot_name;
        // look for the index of the robot in the robot_names vector
        auto it = std::find(robot_names.begin(), robot_names.end(), robot);
        if (it == robot_names.end()) {
            ROS_ERROR_STREAM("Robot name not found");
            success = false;
            return;
        }
        auto robot_idx = std::distance(robot_names.begin(), it);
        // get the move_group and scene_interface
        auto& as = as_[robot_idx];
        auto& move_group = move_groups[robot_idx];
        auto& scene_interface = scene_interfaces[robot_idx];
        auto& action_space = action_spaces[robot_idx];

        ros::param::set(robot + "/pick", true);

        ROS_INFO("Initiated action space");
        std::vector<double> pick_pose; // x, y, z, r, p ,y
        if (!getObjPose(goal->pick_object_name, pick_pose)) {
            ROS_ERROR_STREAM("Failed to get the pose of the pick object");
            success = false;
            as->setAborted();
            return;
        }

        stateType start_state {0, 0, 0, 0, 0, 0};
        ROS_INFO("Getting current pose");
        geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();  // "arm_1tool0"

        start_state[0] = current_pose.pose.position.x;
        start_state[1] = current_pose.pose.position.y;
        start_state[2] = current_pose.pose.position.z;

        Eigen::Quaterniond current_pose_eigen;
        tf::quaternionMsgToEigen(current_pose.pose.orientation, current_pose_eigen);

        ims::get_euler_zyx(current_pose_eigen, start_state[5], start_state[4], start_state[3]);
        ims::normalize_euler_zyx(start_state[5], start_state[4], start_state[3]);

        stateType goal_state = pick_pose;

        // discretize the goal state
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
        std::vector<double> current_joint_state = move_group->getCurrentJointValues();
        std::vector<double> ik_solution;

        if (!scene_interface->calculateIK(current_pose.pose, current_joint_state, ik_solution)) {
            std::cout << "No IK solution for the current pose" << std::endl;
            // failure
            success = false;
            result_.success = success;
            as->setSucceeded(result_);
            return;
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


        auto* heuristic = new ims::SE3HeuristicRPY;
        ims::BestFirstSearchParams params(heuristic);

        bool query_mode = true;
        ZeroTimePlanner planner;
        if (!planner.initializePlanner(action_space, params,
                                       start_state, goal_state,
                                       query_mode, "RRTConnect")){
            //raise error
            std::cout << "Planner initialization failed" << std::endl;
            // failure
            success = false;
            result_.success = success;
            as->setSucceeded(result_);
            return;
        }

        std::vector<stateType> path_;
        if (!planner.plan(path_)) {
            if (query_mode)
                std::cout << "No path found" << std::endl;
            else
                std::cout << "\n" <<"Finished Preprocessing" << std::endl;
            // failure
            success = false;
            result_.success = success;
            as->setSucceeded(result_);
            return;
        }
        else {
            std::cout << "Path found" << std::endl;
        }

        geometry_msgs::Pose place_pose = goal->place_pose;
        goal_state[0] = place_pose.position.x;
        goal_state[1] = place_pose.position.y;
        goal_state[2] = place_pose.position.z;

        Eigen::Quaterniond place_pose_eigen;
        tf::quaternionMsgToEigen(place_pose.orientation, place_pose_eigen);
        ims::get_euler_zyx(place_pose_eigen, goal_state[5], goal_state[4], goal_state[3]);
        ims::normalize_euler_zyx(goal_state[5], goal_state[4], goal_state[3]);
        ROS_INFO("goal state %f %f %f %f %f %f", goal_state[0], goal_state[1], goal_state[2], goal_state[3], goal_state[4], goal_state[5]);
        // discretize the goal state
        for (int i = 0; i < 6; i++) {
            goal_state[i] = std::round(goal_state[i] / discretization[i]) * discretization[i];
        }
        ROS_INFO("goal state %f %f %f %f %f %f", goal_state[0], goal_state[1], goal_state[2], goal_state[3], goal_state[4], goal_state[5]);

        // check IK
        if (scene_interface->calculateIK(place_pose, ik_solution)) {
            std::cout << "IK solution for the place pose" << std::endl;
            for (int i = 0; i < ik_solution.size(); i++) {
                std::cout << "joint " << i << " " << ik_solution[i]*180/M_PI << " " << current_joint_state[i] << std::endl;
            }
        }
        else {
            std::cout << "No IK solution for the place pose" << std::endl;
            // failure
            success = false;
            as->setAborted();
        }

        ros::param::set(robot + "/pick", false);
        action_space->readGoalRegions();

        ZeroTimePlanner planner2;
        if (!planner2.initializePlanner(action_space, params,
                                       start_state, goal_state,
                                       query_mode, "RRTConnect")){
            //raise error
            std::cout << "Planner initialization failed" << std::endl;
            // failure
            result_.success = false;
            as->setSucceeded(result_);
            return;
        }

        std::vector<stateType> path_2;
        if (!planner2.plan(path_2)) {
            if (query_mode)
                std::cout << "No path found" << std::endl;
            else
                std::cout << "\n" <<"Finished Preprocessing" << std::endl;
            // failure
            success = false;
            // just send failure, dont crash
            result_.success = false;
            as->setSucceeded(result_);
            return;
        }
        else {
            std::cout << "Path found" << std::endl;
        }

        std::vector<stateType> path;
        path.insert(path.end(), path_.begin(), path_.end());

        moveit_msgs::RobotTrajectory trajectory;
        ims::profileTrajectory(path.at(0),
                               path.back(),
                               path,
                               *move_group,
                               trajectory);

        move_group->execute(trajectory);

        // go down and grasp the object
        // get the current pose of the end effector
        geometry_msgs::PoseStamped curr_pose = move_group->getCurrentPose();
        geometry_msgs::Pose grasp_pose = curr_pose.pose;
        // if manipulator_1, increment z by 0.2 else increment by 0.02
        if (robot == "manipulator_1") {
            grasp_pose.position.z -= 0.16;
            move_group->setPoseTarget(grasp_pose);
            move_group->move();
        }
        else if (robot == "manipulator_3") {
            // close the gripper using gripper_3 group
            // define the move group
            moveit::planning_interface::MoveGroupInterface gripper_3("gripper_3");
//            gripper_3.setNamedTarget("closed_3");
            gripper_3.setJointValueTarget("arm_3robotiq_85_left_knuckle_joint", 0.6);
            // increase the speed
            gripper_3.setMaxVelocityScalingFactor(0.8);
            gripper_3.move();
            ros::Duration(0.1).sleep();
        }

        // attach the object
        move_group->attachObject(goal->pick_object_name);
        ros::Duration(0.1).sleep();

        // go up
        if (robot == "manipulator_1")
        {
            move_group->setPoseTarget(curr_pose.pose);
            move_group->move();
        }

        // go to the place pose
        path.clear();
        std::reverse(path_.begin(), path_.end());
        path.insert(path.end(), path_.begin(), path_.end());
        path.insert(path.end(), path_2.begin(), path_2.end());
        // profile and execute the path
        // @{
        // execute in the joint space

        trajectory.joint_trajectory.points.clear();
        ims::profileTrajectory(path.at(0),
                               path.back(),
                               path,
                               *move_group,
                               trajectory);

        move_group->execute(trajectory);

        if (robot == "manipulator_3") {
            // close the gripper using gripper_3 group
            // define the move group
            moveit::planning_interface::MoveGroupInterface gripper_3("gripper_3");
            gripper_3.setNamedTarget("open_3");
            gripper_3.setMaxVelocityScalingFactor(0.8);
            gripper_3.move();
            ros::Duration(0.1).sleep();
        }
        // detach the object
        move_group->detachObject(goal->pick_object_name);
        ros::Duration(0.1).sleep();

        path.clear();

        std::reverse(path_2.begin(), path_2.end());
        path.insert(path.end(), path_2.begin(), path_2.end());

        trajectory.joint_trajectory.points.clear();
        ims::profileTrajectory(path.at(0),
                               path.back(),
                               path,
                               *move_group,
                               trajectory);
//        if (robot == "manipulator_1"){
//            ros::Duration(8.0).sleep();
//        }
        move_group->execute(trajectory);

        move_group->setNamedTarget("ready" + robot.substr(robot.size() - 1));
        move_group->move();

        if(success)
        {
            result_.success = true;
            ROS_INFO("%s: Succeeded", robot.c_str());
            // set the action state to succeeded
            as->setSucceeded(result_);
            return;
        }
    }
};



int main(int argc, char** argv) {

    std::vector<std::string> robots;
    // loop through the arguments and if there is an argument that starts with a "manipulator" then it is a robot name
    for (int i = 0; i < argc; i++) {
        std::string arg = argv[i];
        if (arg.find("manipulator") != std::string::npos) {
            robots.push_back(arg);
        }
    }
    // initialize ROS
    ros::init(argc, argv, "~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    ros::param::set("/manipulator_1/regions/pick_region/min_limits",
                    std::vector<double>{-0.5, 0.4, 0.9, 0, 0, 0.0});
    ros::param::set("/manipulator_1/regions/pick_region/max_limits",
                    std::vector<double>{0.4, 0.9, 0.9, 0, 0, 0.0});
    ros::param::set("/manipulator_1/regions/place_region/min_limits",
                    std::vector<double>{0.7, -0.25, 0.75, 0, 0, 0});
    ros::param::set("/manipulator_1/regions/place_region/max_limits",
                    std::vector<double>{1.2, 0.25, 0.75, 0, 0, 0});

    ros::param::set("/manipulator_2/regions/pick_region/min_limits",
                    std::vector<double>{1.60, -1.24, 0.75 , 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_2/regions/pick_region/max_limits",
                    std::vector<double>{2.0, -0.74, 0.75, 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_2/regions/place_region/min_limits",
                    std::vector<double>{0.7, -0.25, 0.7, 0.0, 0.0, 1.570796});
    ros::param::set("/manipulator_2/regions/place_region/max_limits",
                    std::vector<double>{1.2, 0.25, 0.7, 0.0, 0.0, 1.570796});

    ros::param::set("/manipulator_3/regions/pick_region/min_limits",
                    std::vector<double>{1.60, 0.74, 0.80 , 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_3/regions/pick_region/max_limits",
                    std::vector<double>{1.90, 1.24, 0.80, 0.0, 0.0, 0.0});
    ros::param::set("/manipulator_3/regions/place_region/min_limits",
                    std::vector<double>{0.7, -0.25, 0.74, 0.0, 0.0, -1.570796});
    ros::param::set("/manipulator_3/regions/place_region/max_limits",
                    std::vector<double>{1.2, 0.25, 0.74, 0.0, 0.0, -1.570796});

    // create the action server
    ros::NodeHandle pnh("~");
    ctmpActionServer ctmp_action_server(robots, nh, pnh);


    ros::waitForShutdown();
//    ros::spin();
    return 0;
}