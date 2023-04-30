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
 * \date   4/12/23
*/

//#include "ctmp/ctmpActionSpace.hpp"
#include <ctmp/ctmp_utils.hpp>
//#include <manipulation_planning/utils.hpp>
#include <ctmp/planner_zero.hpp>

// search includes
#include <common/types.hpp>
#include <common/state.hpp>


#ifndef CTMP_ZERO_TIME_PLANNER_HPP
#define CTMP_ZERO_TIME_PLANNER_HPP


class ZeroTimePlanner {
public:

    /// \brief The class constructor
    /// TODO: Implement the constructor
    ZeroTimePlanner() = default;

    /// \brief The class destructor
    /// TODO: Implement the destructor
    ~ZeroTimePlanner() = default;

    bool initializePlanner(const std::shared_ptr<ims::ctmpActionSpace>& actionSpacePtr,
                           const ims::BestFirstSearchParams &params,
                           const stateType& start, const stateType& goal,
                           bool query_mode = false, std::string planner_type = "RRTConnect");

    bool isQueryCovered(
            const stateType& full_start_state,
            const stateType& goal);

//    void setStartAndGoal(
//        const RobotState& start_state,
//        const GoalConstraint& goal);

    void setStartAndGoal(
            const stateType& start_state,
            const stateType& goal);

    /// @brief Pre-process the goal region according to the algorithm presented in the paper.
    /// @param full_start_state The start state.
    void PreProcess(const stateType& full_start_state);

    /// @brief For a specific query of end-effector pose, looks for a feasible path.
    void Query(std::vector<ims::state*>& path);

    /// @brief Takes the goal position (x, y, z) and iterate over all grasp options
    /// (preprocessed files). When finds a feasible solution, it returns the path.
    void GraspQuery(std::vector<stateType>& path,
                    std::string grasp_dir = "");

    bool plan(std::vector<stateType>& path);

private:

    void InitMoveitOMPL();

    /// @brief Plan a path from the start state to the attractor using OMPL.
    /// @param attractor The attractor state. IMPORTANT: The attractor state here is in configuration space!
    /// @param path The path from the start state to the attractor.
    /// @return True if a path was found, false otherwise.
    bool PlanPathFromStartToAttractorOMPL(const stateType & attractor, std::vector<stateType >& path);

    /// @brief Plan a path from the start state to the attractor using the ims search package.
    /// @param attractor The attractor state.
    /// @param path The path from the start state to the attractor.
    /// @return True if a path was found, false otherwise.
    bool PlanPathFromStartToAttractorIMS(const stateType & attractor, std::vector<stateType >& path);

    ros::NodeHandle m_nh;
    std::string m_pp_planner;
    bool m_query_mode{};

    stateType m_start_state;
    stateType m_goal;

//    ManipLattice* m_manip_space;
    std::shared_ptr<ims::ctmpActionSpace> m_task_space;


    std::shared_ptr<ims::plannerZero> m_planner_zero;

    std::vector<region> m_regions;
    std::vector<region> m_iregions;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_group;
    moveit::planning_interface::PlanningSceneInterface m_planning_scene_interface;

    std::string read_write_dir;
    std::string read_write_path;
    std::string arm_name;

    void WriteRegions(std::string path="");

    void ReadRegions(std::string path="");
};


#endif //CTMP_ZERO_TIME_PLANNER_HPP
