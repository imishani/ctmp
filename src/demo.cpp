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
 * \date   5/1/23
*/


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ctmp/ctmpAction.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "ctmp_client_demo");
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<ctmp::ctmpAction> ac1("manipulator_1/ctmp_action", true);
    actionlib::SimpleActionClient<ctmp::ctmpAction> ac2("manipulator_2/ctmp_action", true);
    actionlib::SimpleActionClient<ctmp::ctmpAction> ac3("manipulator_3/ctmp_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac1.waitForServer(); //will wait for infinite time
    ac2.waitForServer(); //will wait for infinite time
    ac3.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action


    ctmp::ctmpGoal goal2;
    geometry_msgs::Pose place_pose2;
    goal2.robot_name = "manipulator_2";
    goal2.pick_object_name = "left_side_panel";
    place_pose2.position.x = 0.72; place_pose2.position.y = -0.1; place_pose2.position.z = 0.7;
    place_pose2.orientation.x = 0.0; place_pose2.orientation.y = 0.0;
    place_pose2.orientation.z = 0.7071068; place_pose2.orientation.w = 0.7071068;
    goal2.place_pose = place_pose2;

    ac2.sendGoal(goal2);;
    bool finished_before_timeout = ac2.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout && ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        actionlib::SimpleClientGoalState state = ac2.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");



    ctmp::ctmpGoal goal;
    goal.robot_name = "manipulator_1";
    goal.pick_object_name = "part_c1_2";
    geometry_msgs::Pose place_pose;
    place_pose.position.x = 0.8; place_pose.position.y = 0.0; place_pose.position.z = 0.75;
    place_pose.orientation.x = 0.0; place_pose.orientation.y = 0.0;
    place_pose.orientation.z = 0.0; place_pose.orientation.w = 1.0;
    goal.place_pose = place_pose;
    ac1.sendGoal(goal);
//    ac1.waitForResult();

    finished_before_timeout = ac1.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac1.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");


    ctmp::ctmpGoal goal3;
    geometry_msgs::Pose place_pose3;
    goal3.robot_name = "manipulator_3";
    goal3.pick_object_name = "screw_2";
    place_pose3.position.x = 0.72; place_pose3.position.y = 0.1; place_pose3.position.z = 0.76;
    place_pose3.orientation.x = 0.0; place_pose3.orientation.y = 0.0;
    place_pose3.orientation.z = -0.7071068; place_pose3.orientation.w = 0.7071068;
    goal3.place_pose = place_pose3;

    ac3.sendGoal(goal3);;
    finished_before_timeout = ac3.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac3.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    //exit
    return 0;
}