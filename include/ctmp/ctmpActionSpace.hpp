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


        /// @brief Check state validity (IK and collision) and saves the ik solution in joint_state
        /// @param state_val The state to check
        /// @param joint_state The ik solution
        /// @return True if the state is valid, false otherwise
        bool isStateValid(const stateType& state_val, stateType& joint_state) {
            // check if the state is valid
            switch (mManipulationType->getSpaceType()) {
                case manipulationType::spaceType::ConfigurationSpace:
                    return mMoveitInterface->isStateValid(state_val);
                case manipulationType::spaceType::WorkSpace:
                    geometry_msgs::Pose pose;
                    pose.position.x = state_val[0]; pose.position.y = state_val[1]; pose.position.z = state_val[2];
                    // Euler angles to quaternion
                    Eigen::Quaterniond q;
                    from_euler_zyx(state_val[5], state_val[4], state_val[3], q);
                    pose.orientation.x = q.x(); pose.orientation.y = q.y();
                    pose.orientation.z = q.z(); pose.orientation.w = q.w();
                    bool succ = mMoveitInterface->calculateIK(pose,joint_state);
                    if (!succ) {
                        ROS_INFO("IK failed");
                        return false;
                    }
                    else {
                        // print the joint state
//                        ROS_INFO("Joins state size: %zu, IK solution: %f %f %f %f %f %f",
//                                 joint_state.size(), joint_state[0]*180/M_PI, joint_state[1]*180/M_PI,
//                                 joint_state[2]*180/M_PI, joint_state[3]*180/M_PI,
//                                 joint_state[4]*180/M_PI, joint_state[5]*180/M_PI);
                        return mMoveitInterface->isStateValid(joint_state);
                    }
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
                ROS_DEBUG_NAMED("CTMP", "Sampled state is not valid");
                return false;
            } else {
                return true;
            }
        }

        /// @brief Find region containing the state
        /// @param workspace_state The state
        /// @param position_only If true, heck only the position values. Default is false
        /// @return int The region id, -1 if no region was found
        int FindRegionContainingState_WS(const stateType& workspace_state, bool position_only=false){
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


        bool getSuccessors(int curr_state_ind,
                           std::vector<ims::state*>& successors,
                           std::vector<double>& costs) override{
            bool success = ManipulationActionSpace::getSuccessors(curr_state_ind, successors, costs);
            // if REACHABILITY mode, check if successors in goal region. If not, remove from successors
            if (m_search_mode == REACHABILITY){
                std::vector<ims::state*> pruned_successors;
                std::vector<double> pruned_costs;
                std::vector<ims::state*> remove;
                for (size_t i = 0; i < successors.size(); ++i) {
                    auto* entry = successors[i];
                    auto ws_child = entry->getState();
                    if (isStateInGoalRegion(ws_child)){
                        pruned_successors.push_back(entry);
                        pruned_costs.push_back(costs[i]);
                    }
                    else {
                        // delete entry;
                        remove.push_back(entry);
                    }
                }
                successors = pruned_successors;
                costs = pruned_costs;
                // delete successors
            }
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
