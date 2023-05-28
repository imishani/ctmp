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
 * \date   4/15/23
*/

#ifndef CTMP_CTMPACTIONSPACE_HPP
#define CTMP_CTMPACTIONSPACE_HPP

// standard includes
#include <iostream>
#include <vector>

// search includes
#include <heuristics/standardHeu.hpp>

// manipulation_planning includes
#include <manipulation_planning/common/utils.hpp>
#include <manipulation_planning/manipulationActionSpace.hpp>

// ctmp include
#include <ctmp/ctmp_utils.hpp>

// ROS includes
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// include Axes visualizer
#include <visualization_msgs/MarkerArray.h>


namespace ims{


    struct ctmpActionType: public manipulationType{
        ctmpActionType(): manipulationType("../config/ws.mprim"){
            setSpaceType(manipulationType::spaceType::WorkSpace);
        }

        explicit ctmpActionType(const std::string& mprimFile): manipulationType(mprimFile){
            setSpaceType(manipulationType::spaceType::WorkSpace);
        }
    };


    /// @class ctmpActionSpace
    /// @brief Action space for the CTMP planner
    class ctmpActionSpace : public ManipulationActionSpace{

    public:
        /// @brief Constructor
        /// @param moveitInterface The moveit interface
        /// @param manipulationType The manipulation type. IMPORTANT: The manipulation type here is in Workspace!
        ctmpActionSpace(const MoveitInterface& env,
                        const ctmpActionType& actions_ptr) :
                ManipulationActionSpace(env, actions_ptr) {
            m_pnh = ros::NodeHandle(env.mGroupName);
            readGoalRegions();
            // print m_pnh namespace
            std::cout << "m_pnh namespace: " << m_pnh.getNamespace() << std::endl;
            m_vis_pub = m_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        }

        /// @brief Get the planning group name
        /// @return The planning group name as a string
        std::string getPlanningGroupName() const {
            return mMoveitInterface->mGroupName;
        }


        /// @brief Reading goal regions from the parameter server as defined by the user
        /// @return True if the goal regions were read successfully, false otherwise
        bool readGoalRegions(){
            std::string region_type;

            bool pick;
            m_pnh.param("pick", pick, false);
            if (pick){
                region_type = "pick";
            }
            else{
                region_type = "place";
            }
            std::cout << "region_type: " << region_type << std::endl;
            XmlRpc::XmlRpcValue xlist;
            if (!m_pnh.getParam("regions/" + region_type + "_region/min_limits", xlist)) {
                ROS_WARN("Could not find start region min limits");
                return false;
            }

            if (xlist.size() != 6 + mMoveitInterface->num_joints - 6) {
                ROS_ERROR("min limits: %zu params required, %d provided", mMoveitInterface->num_joints, xlist.size());
                return false;
            }

            for (size_t i = 0; i < xlist.size(); ++i) {
                m_min_ws_limits.push_back(xlist[(int)i]);
            }

            if (!m_pnh.getParam("regions/" + region_type + "_region/max_limits", xlist)) {
                ROS_WARN("Could not find start region max limits");
                return false;
            }

            if (xlist.size() != 6 + mMoveitInterface->num_joints - 6) {
                ROS_ERROR("max limits: %zu params required, %d provided", mMoveitInterface->num_joints, xlist.size());
                return false;
            }

            for (size_t i = 0; i < xlist.size(); ++i) {
                m_max_ws_limits.push_back(xlist[(int)i]);
            }

            // normalize [-pi,pi]x[-pi/2,pi/2]x[-pi,pi]
            // center of cell
            normalize_euler_zyx(m_min_ws_limits[5], m_min_ws_limits[4], m_min_ws_limits[3]);
            normalize_euler_zyx(m_max_ws_limits[5], m_max_ws_limits[4], m_max_ws_limits[3]);
            roundStateToDiscretization(m_min_ws_limits, mManipulationType->mStateDiscretization);
            roundStateToDiscretization(m_max_ws_limits, mManipulationType->mStateDiscretization);

            for (size_t i = 0; i < m_min_ws_limits.size(); ++i) {
                if (m_min_ws_limits[i] > m_max_ws_limits[i]) {
                    ROS_ERROR("Min limit greater than max limit at index %zu", i);
                    return false;
                }
            }

            m_distribution.resize(6 + mMoveitInterface->num_joints - 6);

            for (int i = 0; i < m_distribution.size() ; ++i) {
                m_distribution[i] = std::uniform_real_distribution<double> (m_min_ws_limits[i], m_max_ws_limits[i]);
            }

            return true;

        }

        ///@brief Prune regions that are contained in other regions
        void PruneRegions(){
            std::vector<region> filtered_regions;
            for (const auto& r1 : *m_regions_ptr){
                bool keep = true;
                for (const auto& r2 : *m_regions_ptr){
                    double epsilon = 1e-4;
                    double dis {INT_MAX};
                    getSE3RPYdistance(r1.state, r2.state, dis);
                    if (dis == 0){
                        continue;
                    } else if (r2.radius - r1.radius > dis){    //  + epsilon
                        keep = false;
                        break;
                    }
                }
                if (keep){
                    filtered_regions.push_back(r1);
                }
            }
            ROS_INFO("Filtered regions from %zu to %zu", m_regions_ptr->size(), filtered_regions.size());
            *m_regions_ptr = filtered_regions;
        }

        /// @brief Get  an IK solution for a given state
        /// @param state_val The state to get the IK solution for
        /// @param joint_state The IK solution
        /// @return True if an IK solution was found, false otherwise
        bool getIKSolution(const stateType& state_val, stateType& joint_state) {
            // check if the state is valid
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    // raise error
                    ROS_ERROR("IK solution is irrelevant for configuration space");
                    return false;
                case manipulationType::spaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    return mMoveitInterface->calculateIK(pose, joint_state);
            }
            return false;
        }

        /// @brief Get  an IK solution for a given state
        /// @param state_val The state to get the IK solution for
        /// @param joint_state The IK solution
        /// @return True if an IK solution was found, false otherwise
        bool getIKSolution(const stateType& state_val, stateType& seed, stateType& joint_state) {
            // check if the state is valid
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    // raise error
                    ROS_ERROR("IK solution is irrelevant for configuration space");
                    return false;
                case manipulationType::spaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    return mMoveitInterface->calculateIK(pose, seed, joint_state);
            }
            return false;
        }


        /// @brief Check if the state is in some of the regions which were preprocessed
        /// @param valid True if the state is valid, false otherwise
        /// @param state_id The state id
        /// @return True if the state is in a region, false otherwise
        bool IsStateCovered(bool valid, const int state_id){
            std::vector<region>* regions_ptr;
            if (valid)
                regions_ptr = m_regions_ptr;
            else
                regions_ptr = m_iregions_ptr;

            for (const auto& r : *regions_ptr) {
                int center_state_id = getOrCreateState(r.state);
                // use getHeuristic function from ims::SE3Heuristic
                double dsum {INT_MAX};
                getSE3RPYdistance(getState(state_id)->getState(),
                                  getState(center_state_id)->getState(),
                                  dsum);
                // if (valid)
                //     printf("dsum %d radius %u\n", dsum, r.radius);
                if (dsum < r.radius || dsum == 0) {
                    return true;
                }
            }
            return false;
        }

        /// @brief update the search mode (REACHABILITY)
        void UpdateSearchMode(int search_mode){m_search_mode = search_mode;}

        /// @brief Sample a state in the workspace as an attractor state
        /// @param state The sampled state
        /// @param max_tries The maximum number of tries
        /// @return int The sampled state id, -1 if no state was sampled
        int SampleAttractorState(stateType& state, const int max_tries){
            int attractor_state_id;
            int count {0};
            while (count < max_tries){
                count++;
                stateType joint_state, ws_state;
                if (!SampleRobotState(joint_state, ws_state)){
                    continue;
                }
                if (!isGraspable(joint_state, ws_state, ws_state)){
                    continue;
                }
                state = ws_state;
                attractor_state_id = getOrCreateState(state);
                // check if state is covered
                if (IsStateCovered(true, attractor_state_id)){
                    continue;
                }

                auto attractor_state = getState(attractor_state_id);
                attractor_state->setMappedState(joint_state);
                m_valid_front.insert(attractor_state);

                // set a goal?

                return attractor_state_id;
            }
            return -1;
        }

        /// @brief Sample a state in the workspace
        /// @param joint_state The ik solution
        /// @param workspace_state The sampled state
        /// @return True if the state is valid, false otherwise
        bool SampleRobotState(stateType& joint_state, stateType& workspace_state){
            workspace_state.resize(6 + mMoveitInterface->num_joints - 6);
            for (int i = 0; i < workspace_state.size(); ++i) {
                workspace_state[i] = m_distribution[i](m_generator);
            }
            normalize_euler_zyx(workspace_state[5], workspace_state[4], workspace_state[3]);
            roundStateToDiscretization(workspace_state, mManipulationType->mStateDiscretization);
            // check if state is valid
            if (!isStateValid(workspace_state, joint_state)) {
//                ROS_DEBUG_NAMED("CTMP", "Sampled state is not valid");
//                ROS_INFO_STREAM("Sampled state is not valid " << workspace_state[0] << " " << workspace_state[1] << " " << workspace_state[2] << " " << workspace_state[3] << " " << workspace_state[4] << " " << workspace_state[5]);
                return false;
            } else {
                return true;
            }
        }

        /// @brief Find region containing the state
        /// @param workspace_state The state
        /// @param position_only If true, heck only the position values. Default is false
        /// @return int The region id, -1 if no region was found
        int FindRegionContainingState_WS(const stateType& workspace_state,
                                         bool position_only=false){
            int query_state_id = getOrCreateState(workspace_state);
            bool covered = false;
            int reg_idx = 0;
            stateType goal_state;

            for (const auto& r : *m_regions_ptr) {
                int center_state_id = getOrCreateState(r.state);
                // use getHeuristic function from ims::SE3Heuristic
                double dsum {INT_MAX};
                if (!position_only){
                    getSE3RPYdistance(getState(query_state_id)->getState(), getState(center_state_id)->getState(), dsum);
                } else {
                    dsum = 0.0;
                    for (size_t i {0}; i < 3; ++i) {
                        double dj = (workspace_state[i] - r.state[i]);
                        dsum += dj*dj;
                    }
                    dsum = std::sqrt(dsum);
                }

                if (dsum < r.radius || dsum == 0) {
                    goal_state = r.state;
                    covered = true;
                    break;
                }
                reg_idx++;
            }

            if (covered) {
                ROS_DEBUG_NAMED("graph", "Attractor State of Containing Region %d", reg_idx);
                return reg_idx;
            }
            else {
                ROS_INFO_STREAM_NAMED("graph.expands", "  start workspace_state: " << workspace_state[0] << " "
                                                                                   << workspace_state[1] << " " << workspace_state[2] << " " << workspace_state[3] << " " <<
                                                                                   workspace_state[4] << " " << workspace_state[5]);
                return -1;
            }
        }

        /// @brief Check if the state is in the goal region
        /// @param workspace_state The state
        /// @return True if the state is in the goal region, false otherwise
        bool isStateInGoalRegion(const stateType& workspace_state){
            double eps {0.0001};
            for (int i {0}; i < workspace_state.size(); ++i) {
                if (workspace_state[i] < m_min_ws_limits[i] - eps || workspace_state[i] > m_max_ws_limits[i] + eps) {
                    return false;
                }
            }
            return true;
        }

        /// @brief Prune states that are covered by other regions
        void PruneCoveredStates(std::vector<stateType>& workspace_states){
            std::vector<stateType> pruned_states;
            for (const auto& s : workspace_states) {
                int state_id = getOrCreateState(s);
                if (!IsStateCovered(true, state_id) && !IsStateCovered(false, state_id)) {
                    pruned_states.push_back(s);
                }
            }
            workspace_states = pruned_states;
        }

        /// @brief Check for graspability using Runge-Kutta method (RK4).
        /// @param robot_state The robot state (configuration state) of th pre-grasp pose.
        /// @param pregrasp_pose The pre-grasp pose (x, y,z, roll, pitch, yaw).
        /// @param grasp_pose The grasp pose(x, y,z, roll, pitch, yaw).
        /// @return True if the object is graspable, false otherwise.
        bool isGraspable(const robot_state::RobotStatePtr& robot_state,
                         const stateType &pregrasp_pose,
                         const stateType &grasp_pose) {
            // initial state (configuration space)
            std::vector<double> theta0;
            Eigen::VectorXd grasp_vec(6);
            Eigen::VectorXd pre_grasp_vec(6);
            for (size_t i = 0; i < 6; ++i) {
                grasp_vec[i] = grasp_pose[i];
                pre_grasp_vec[i] = pregrasp_pose[i];
            }
            pre_grasp_vec[2] += 0.2;
            std::vector<stateType> path;
            robot_state->copyJointGroupPositions(getPlanningGroupName(), theta0);
            path.push_back(theta0);
            // get the Jacobian
            Eigen::MatrixXd J;
            auto joint_model_group = robot_state->getJointModelGroup(getPlanningGroupName());
            robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     Eigen::Vector3d(0, 0, 0), J);
//    // rotate the jacobian to the end effector frame
//    Eigen::Isometry3d T = robot_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back()).inverse(Eigen::Isometry);
            // Set the action type to configuration space, only for state checking purposes!
            auto action_type = getManipActionType();
            setManipActionType(manipulationType::spaceType::ConfigurationSpace);

            Eigen::MatrixXd J_pinv;
            double h = 1 / 10.0;
            auto theta = theta0;
            auto curr_grasp_vec = pre_grasp_vec;
            Eigen::MatrixXd J_inv;
            for (int i{0} ; i < 10; i++){
                // Compute the Jacobian
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
                double tolerance = std::max(J.cols(),
                                            J.rows()) *
                                   svd.singularValues().array().abs().maxCoeff() *
                                   std::numeric_limits<double>::epsilon();

                // Compute the inverse of a matrix given its SVD and a tolerance for singular values close to zero:
                J_pinv = svd.matrixV() *
                         svd.singularValues().array().abs().cwiseInverse().cwiseMax(tolerance).matrix().asDiagonal() *
                         svd.matrixU().adjoint();

//        J_pinv = svd.matrixV() * Eigen::MatrixXd(svd.singularValues().array().abs().inverse().matrix().asDiagonal()) * svd.matrixU().transpose();
                J_inv = J.transpose() * (J * J.transpose()).inverse();
//        const Eigen::MatrixXd U = svd.matrixU();
//        const Eigen::MatrixXd V = svd.matrixV();
//        const Eigen::VectorXd S = svd.singularValues();
//        Eigen::VectorXd Sinv = S;
//        static const double pinvtoler = std::numeric_limits<float>::epsilon();
//        double maxsv = 0.0;
//        for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
//            if (fabs(S(i)) > maxsv)
//                maxsv = fabs(S(i));
//        for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i){
//            // Those singular values smaller than a percentage of the maximum singular value are removed
//            if (fabs(S(i)) > maxsv * pinvtoler)
//                Sinv(i) = 1.0 / S(i);
//            else
//                Sinv(i) = 0.0;
//            }
//        Eigen::MatrixXd Jinv_moveit = (V * Sinv.asDiagonal() * U.transpose());
//        std::cout << "Jinv_moveit: " << std::endl << Jinv_moveit << std::endl;

                // check if J_pinv and J_inv are the same
                if ((J_pinv - J_inv).norm() > 0.0001){
                    ROS_ERROR_STREAM("J_pinv and J_inv are not the same");
                }

                // RK(4)
                auto k1 = h * (J_pinv * (grasp_vec - pre_grasp_vec));
                auto k2 = h * (J_pinv * (grasp_vec - pre_grasp_vec + k1 / 2.0));
                auto k3 = h * (J_pinv * (grasp_vec - pre_grasp_vec + k2 / 2.0));
                auto k4 = h * (J_pinv * (grasp_vec - pre_grasp_vec + k3));
                auto del_theta = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
                for (size_t j = 0; j < 6; ++j) {
                    theta[j] += del_theta[j];
                }
                path.push_back(theta);
                // update the robot state
                robot_state->setJointGroupPositions(getPlanningGroupName(), theta);
                robot_state->update();
                // check if the new state is in collision and out of bounds
                if (!robot_state->satisfiesBounds(joint_model_group)) {
                    ROS_WARN("Out of bounds");
                    setManipActionType(action_type);
                    return false;
                }
                if (!isStateValid(theta)) {
                    ROS_WARN("Grasping state is in collision!");
                    setManipActionType(action_type);
                    return false;
                }

//        robot_state->setFromDiffIK(robot_state->getJointModelGroup(arm_name), J_inv * (grasp_vec - curr_grasp_vec));

                // update the current grasp vector using the current robot state FK
//        auto curr_pose = robot_state->getGlobalLinkTransform(robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()));
//        curr_grasp_vec[0] = curr_pose.translation()[0]; curr_grasp_vec[1] = curr_pose.translation()[1]; curr_grasp_vec[2] = curr_pose.translation()[2];
//        auto euler = curr_pose.rotation().eulerAngles(2, 1, 0);
//        curr_grasp_vec[3] = euler[0]; curr_grasp_vec[4] = euler[1]; curr_grasp_vec[5] = euler[2];
//        ims::normalize_euler_zyx(curr_grasp_vec[5], curr_grasp_vec[4], curr_grasp_vec[3]);
                // update the Jacobian
                robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                         Eigen::Vector3d(0, 0, 0), J);
            }
            // print path int degrees
//            std::cout << "Path in degrees" << std::endl;
//            for (auto & i : path) {
//                for (size_t j = 0; j < 6; ++j) {
//                    i[j] = i[j] * 180.0 / M_PI;
//                    std::cout << i[j] << " ";
//                }
//                std::cout << std::endl;
//            }
            setManipActionType(action_type);
            return true;
        }

        /// @brief Fill the frontier lists with the given state ids
        /// @param state_ids The state ids
        void FillFrontierLists(const std::vector<int>& state_ids){
            for (const auto& state_id : state_ids) {
                auto entry = getState(state_id);
                stateType joint_space_state = entry->getMappedState();
                if (isStateValid(entry->getState(), joint_space_state)) {
                    m_valid_front.insert(entry);
                }
                else {
                    m_invalid_front.insert(entry);
                }
            }
        }

        /// @brief Check for graspability using Runge-Kutta method (RK4).
        /// @param joint_state The robot state of the pre-grasp pose.
        /// @param pregrasp_pose The pre-grasp pose (x, y,z, roll, pitch, yaw).
        /// @param grasp_pose The grasp pose(x, y,z, roll, pitch, yaw).
        /// @return True if the object is graspable, false otherwise.
        bool isGraspable(const stateType& robot_state,
                         const stateType& pregrasp_pose,
                         const stateType& grasp_pose) {
            robot_state::RobotStatePtr robot_state_ptr = mMoveitInterface->m_kinematic_state;
            robot_state_ptr->setJointGroupPositions(getPlanningGroupName(), robot_state);
            return isGraspable(robot_state_ptr, pregrasp_pose, grasp_pose);
        }

        /// @brief Set the attractor state to the first state in the valid frontier
        /// @return int The state id of the attractor state
        int SetAttractorState()
        {
            auto it = m_valid_front.begin();
            auto entry = *it;
            m_valid_front.erase(it);

            stateType workspace_state = entry->getState();

            return getOrCreateState(workspace_state);

        }

        int SetInvalidStartState() {
            auto it = m_invalid_front.begin();
            auto entry = *it;
            m_invalid_front.erase(it);

            if (entry->getMappedState().empty()){
                stateType joint_state;
                getIKSolution(entry->getState(), joint_state);
                entry->setMappedState(joint_state);
            }

            return entry->getStateId();
        }

        /// @brief Check if the query state is covered by a region
        /// @param full_start_state The start state
        /// @param goal The goal state
        /// @return True if the query is covered, false otherwise
        bool IsQueryCovered(
                const stateType& full_start_state,
                const stateType& goal)
        {
            /// TODO: Its in ws or cs? Check if the start state is covered by a region
            double eps = 0.25;
            for (size_t i = 0; i < full_start_state.size(); ++i) {
                if (fabs(full_start_state[i] - (*m_regions_ptr).front().start[i]) > eps) {
                    ROS_WARN("ZTP: start state is not preprocessed, index %zu is different", i);
                    return false;
                }
            }

            if (!isStateInGoalRegion(goal)) {
                ROS_WARN("ZTP: goal state is not covered in goal region");
                return false;
            }

            if (!ManipulationActionSpace::isStateValid(goal)) {
                ROS_WARN("The discretized goal state in collision");
                return false;
            }

            return true;
        }

        /// @brief Pass the regions to action space
        /// @param regions_ptr The regions
        /// @param iregions_ptr The invalid regions
        void PassRegions(std::vector<region>* regions_ptr,
                         std::vector<region>* iregions_ptr)
        {
            m_regions_ptr = regions_ptr;
            m_iregions_ptr = iregions_ptr;
        }

        /// @brief Visualize a state point in rviz for debugging
        /// @param state_id The state id
        /// @param type The type of state (greedy, attractor, etc)
        void VisualizePoint(int state_id, const std::string& type){
            auto* entry = getState(state_id);
            auto ws_parent = entry->getState();

            visualization_msgs::Marker marker;
            marker.header.frame_id = mMoveitInterface->mPlanningScene->getPlanningFrame();
            marker.header.stamp = ros::Time();
            marker.ns = "graph";
            marker.id = m_vis_id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = ws_parent[0]; marker.pose.position.y = ws_parent[1]; marker.pose.position.z = ws_parent[2];
            marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
            if (type == "greedy"){
                // green
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                marker.color.a = 0.5;
            } else if (type == "attractor"){
                // blue
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
                marker.color.a = 0.8;
            } else if (type == "exited"){
                // red
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                marker.color.a = 1.0;
            } else if (type == "invalid"){
                // black
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 0.0;
                marker.color.a = 1.0;
            }
            else{
                // yellow
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
                marker.color.a = 0.8;
            }
            // visualize
            m_vis_pub.publish(marker);
            m_vis_id++;
        }


        bool getSuccessorsWs(int curr_state_ind,
                                     std::vector<state*>& successors,
                                     std::vector<double>& costs) override {
            // get the current state
            auto curr_state = this->getState(curr_state_ind);
            auto curr_state_val = curr_state->getState();
            // get the actions
            auto actions = mManipulationType->getActions();
            // convert to quaternion
            Eigen::Quaterniond q_curr;
            from_euler_zyx(curr_state_val[5], curr_state_val[4], curr_state_val[3], q_curr);
            // get the successors
            stateType new_state_val;
            for (auto action : actions) {
                new_state_val.clear();
                // create a new state in the length of the current state
                new_state_val.resize(curr_state_val.size());
                // increment the xyz coordinates
                for (int i {0} ; i < 3 ; i++) {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }

                Eigen::Quaterniond q_action {action[6], action[3], action[4], action[5]};
                auto q_new = q_curr * q_action;

                // convert the quaternion to euler angles
                get_euler_zyx(q_new, new_state_val[5], new_state_val[4], new_state_val[3]);
                normalize_euler_zyx(new_state_val[5], new_state_val[4], new_state_val[3]);
                // discretize
                roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

//                bool succ; stateType mapped_state;
//                if (curr_state->getMappedState().empty()){
//                    succ = isStateValid(new_state_val,
//                                        mapped_state);
//                }
//                else
//                    succ = isStateValid(new_state_val,
//                                        curr_state->getMappedState(),
//                                        mapped_state);
//                if (succ) {
                    // create a new state
                int next_state_ind = getOrCreateState(new_state_val);
                auto new_state = this->getState(next_state_ind);
//                new_state->setMappedState(mapped_state);
                // add the state to the successors
                successors.push_back(new_state);
                // add the cost
                double cost {0};
                for (int i {0} ; i < 3 ; i++) {
                    cost += action[i]*action[i];
                }
                // add the cost of the rotation which is quaternion
                double r, p, y;
                get_euler_zyx(q_action, y, p, r);
                cost += r*r + p*p + y*y;
                costs.push_back(cost);
//                }
            }
            return true;
        }

        bool getSuccessorsCs(int curr_state_ind,
                                     std::vector<state*>& successors,
                                     std::vector<double>& costs) override{
            // get the current state
            auto curr_state = this->getState(curr_state_ind);
            auto curr_state_val = curr_state->getState();
            // get the actions
            auto actions = mManipulationType->getActions();
            // get the successors
            for (auto action : actions) {
                // create a new state in the length of the current state
                stateType new_state_val {};
                new_state_val.resize(curr_state_val.size());
                std::fill(new_state_val.begin(), new_state_val.end(), 0.0);

                for (int i {0} ; i < curr_state_val.size() ; i++) {
                    new_state_val[i] = curr_state_val[i] + action[i];
                }
                // normalize the angles
                normalizeAngles(new_state_val);
                // discretize the state
                roundStateToDiscretization(new_state_val, mManipulationType->mStateDiscretization);

                // if (isStateToStateValid(curr_state_val, new_state_val)) {
                if (isStateValid(new_state_val)) {
                    // create a new state
                    int next_state_ind = getOrCreateState(new_state_val);
                    auto new_state = this->getState(next_state_ind);
                    // add the state to the successors
                    successors.push_back(new_state);
                    // add the cost
                    // TODO: change this to the real cost
                    double norm = 0;
                    for (double i : action) {
                        norm += i * i;
                    }
                    costs.push_back(sqrt(norm));
                }
            }
            return true;
        }



        bool getSuccessors(int curr_state_ind,
                           std::vector<ims::state*>& successors,
                           std::vector<double>& costs) override{
            bool success;
            if (mManipulationType->getSpaceType() == manipulationType::spaceType::ConfigurationSpace) {
                success = getSuccessorsCs(curr_state_ind,
                                          successors,
                                          costs);
            } else {
                success = getSuccessorsWs(curr_state_ind,
                                          successors,
                                          costs);
            }
            auto curr_state = this->getState(curr_state_ind);
            // if REACHABILITY mode, check if successors in goal region.
            // If not, remove from successors
            if (m_search_mode == REACHABILITY) {
                std::vector<ims::state *> pruned_successors;
                std::vector<double> pruned_costs;
                std::vector<ims::state *> remove;
                for (size_t i = 0; i < successors.size(); ++i) {
                    auto *entry = successors[i];
                    auto ws_child = entry->getState();
                    if (isStateInGoalRegion(ws_child)) {
                        bool succ;
                        stateType mapped_state;
                        if (curr_state->getMappedState().empty()) {
                            succ = isStateValid(ws_child,
                                                mapped_state);
                        } else
                            succ = isStateValid(ws_child,
                                                curr_state->getMappedState(),
                                                mapped_state);
                        if (succ) {
                            entry->setMappedState(mapped_state);
                        }
                        pruned_successors.push_back(entry);
                        pruned_costs.push_back(costs[i]);
                    } else {
                        // delete entry;
                        remove.push_back(entry);
                    }
                }
                successors = pruned_successors;
                costs = pruned_costs;
                // delete successors
            }
//            } else if (m_search_mode == QUERY){
//                std::vector<ims::state*> pruned_successors;
//                std::vector<double> pruned_costs;
//                std::vector<ims::state*> remove;
//                for (size_t i = 0; i < successors.size(); ++i) {
//                    auto* entry = successors[i];
//                    auto ws_child = entry->getState();
//                    if (isStateInGoalRegion(ws_child)){
//                        pruned_successors.push_back(entry);
//                        pruned_costs.push_back(costs[i]);
//                    }
//                    else {
//                        // delete entry;
//                        remove.push_back(entry);
//                    }
//                }
//                successors = pruned_successors;
//                costs = pruned_costs;
//            }
            return success;
        }

        // valid/invalid uncovered frontier states
        std::set<ims::state*> m_valid_front;
        std::set<ims::state*> m_invalid_front;


    private:
        ros::NodeHandle m_pnh;
        ros::NodeHandle m_nh;

        std::vector<region>* m_regions_ptr;     // be careful
        std::vector<region>* m_iregions_ptr;

        std::vector<double> m_min_ws_limits;
        std::vector<double> m_max_ws_limits;
        std::vector<std::uniform_real_distribution<double>> m_distribution; // Inclusive Exclusive uniform distribution
        std::default_random_engine m_generator; // Pseudo random values generator (deterministic)

        int m_vis_id = 0;
        ros::Publisher m_vis_pub;
        int m_search_mode;
    };
}



#endif //CTMP_CTMPACTIONSPACE_HPP
