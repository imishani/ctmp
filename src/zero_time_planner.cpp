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

#include <memory>

#include "ctmp/zero_time_planner.hpp"


bool ZeroTimePlanner::initializePlanner(const std::shared_ptr<ims::ctmpActionSpace> &actionSpacePtr,
                                        const ims::BestFirstSearchParams &params, const stateType &start,
                                        const stateType &goal, bool query_mode,
                                        std::string planner_type) {
    m_task_space = actionSpacePtr;
    arm_name = m_task_space->getPlanningGroupName();
    m_start_state = start;
    m_goal = goal;
    m_query_mode = query_mode;
    m_pp_planner = planner_type;
    // initialize m_planner_zero
    m_planner_zero = std::make_shared<ims::plannerZero>(params);
    try {
        m_planner_zero->initializePlanner(actionSpacePtr,
                                          start, goal);
    }
    catch (std::exception &e) {
        ROS_ERROR("Failed to initialize planner zero");
        return false;
    }
    std::string current_dir = ros::package::getPath("ctmp");
    read_write_dir = current_dir + "/data/";
    bool m_pick;
    nh = ros::NodeHandle();
    m_nh = ros::NodeHandle(arm_name);
    m_nh.param("pick", m_pick, false);
    if (m_pick){
        // get current directory
        read_write_path = read_write_dir + arm_name + "_pick.dat";
    }
    else {
        read_write_path = read_write_dir + arm_name + "_place.dat";
    }
    ROS_INFO("Read write path: %s", read_write_path.c_str());
    if (m_regions.empty()) {
        ReadRegions();
        m_task_space->PassRegions(&m_regions, &m_iregions);
    }
    return true;
}


bool ZeroTimePlanner::isQueryCovered(const stateType &full_start_state, const stateType &goal) {
    // check if preprocessing covered this query
    if (!m_task_space->IsQueryCovered(full_start_state, goal)) {
        return false;
    }

    return true;
}

void ZeroTimePlanner::setStartAndGoal(const stateType &start_state, const stateType &goal) {
    m_start_state = start_state;
    m_goal = goal;
}

void ZeroTimePlanner::PreProcess(const stateType &full_start_state) {
    m_regions.clear();
    ROS_INFO("Preprocessing");
    if (m_pp_planner == "RRTConnect")
        InitMoveitOMPL();
    else
        InitwAstarIMS();

//        unsigned int radius_max_v = 1;
    double radius_max_v = 0.2;
//        unsigned int radius_max_i = 100;
    double radius_max_i = 0.2;
    ROS_INFO("Waiting for regions");
    m_task_space->PassRegions(&m_regions, &m_iregions);
    ROS_INFO("Preprocessing with %s", m_pp_planner.c_str());
    // 1. SAMPLE ATTRACTOR
    int maximum_tries = 10000;
    stateType sampled_state;
    // also sets the attractor as goal for heuristic functions
    int sampled_state_id = m_task_space->SampleAttractorState(sampled_state, maximum_tries);
    if (sampled_state_id == -1) {
        ROS_ERROR("Failed to sample first attractor");
        return;
    }
    m_task_space->VisualizePoint(sampled_state_id, "attractor");

    while (!m_task_space->m_valid_front.empty() || !m_task_space->m_invalid_front.empty()) {
        while (!m_task_space->m_valid_front.empty()) {

            int attractor_state_id = m_task_space->SetAttractorState();

            if (!m_planner_zero->is_state_covered(attractor_state_id)) {
                m_task_space->VisualizePoint(attractor_state_id, "attractor");
                std::vector<stateType> path;
                // 2. PLAN PATH TO ACTUAL GOAL

                bool ret;
                auto attr_state = m_task_space->getState(attractor_state_id);
                if (attr_state->getMappedState().empty()){
                    stateType mapped_state;
                    m_task_space->getIKSolution(attr_state->getState(), mapped_state);
                    attr_state->setMappedState(mapped_state);
                }
                if (m_pp_planner == "RRTConnect") {
                    ret = PlanPathFromStartToAttractorOMPL(attr_state->getMappedState(), path);
                } else {
                    ret = PlanPathFromStartToAttractorIMS(attr_state->getState(), path);
                }
                if (!ret){
                    continue;
                }

                // plan alternatives
//                std::vector<std::vector<stateType>> alternatives;
//                bool succ_alternatives = PlanAlternativePaths(attr_state->getMappedState(), path, alternatives);
//                robot_state::RobotStatePtr checking_state = m_group->getCurrentState();
//                std::vector<stateType> grasp_path;
//                checking_state->setJointGroupPositions(m_group->getName(), attr_state->getMappedState());
//                GetPathFromPreGraspToGrasp(attr_state->getState(),
//                                           checking_state, attr_state->getState(),
//                                           grasp_path);

                // getchar();
                // 3. COMPUTE REACHABILITY
                m_task_space->UpdateSearchMode(REACHABILITY);
                // reachability search
//                    unsigned int radius = m_planner_zero->compute_reachability(radius_max_v, attractor_state_id);
                double radius = m_planner_zero->compute_reachability(radius_max_v, attractor_state_id);

                // 4. ADD REGION
                region r;
                r.start = full_start_state;
                r.radius = radius;
                r.state = attr_state->getState();
                r.path = path;
                m_regions.push_back(r);

                ROS_INFO("Radius %f, Regions so far %zu", radius, m_regions.size());
                ROS_INFO("Path size: %zu", r.path.size());
                // ROS_INFO m_regions:

                std::vector<int> open;
                m_planner_zero->get_frontier_stateids(open);

                m_task_space->FillFrontierLists(open);

            }
        }
        while (!m_task_space->m_invalid_front.empty()) {

            int iv_start_state_id = m_task_space->SetInvalidStartState();

            if (!m_planner_zero->is_state_covered(iv_start_state_id)) {
                m_task_space->VisualizePoint(sampled_state_id, "attractor");

//                    int radius = m_planner_zero->search_for_valid_uncovered_states(radius_max_i, iv_start_state_id);
                double radius = m_planner_zero->search_for_valid_uncovered_states(radius_max_i, iv_start_state_id);
                region r;
                r.radius = radius;
                r.state = m_task_space->getState(iv_start_state_id)->getState();
                m_iregions.push_back(r);

                ROS_INFO("Radius %f, IRegions so far %zu", radius, m_iregions.size());

                std::vector<int> open;
                m_planner_zero->get_frontier_stateids(open);
                m_task_space->FillFrontierLists(open);

                if (!m_task_space->m_valid_front.empty()) {
                    break;
                }
            }
        }
    }
    m_task_space->PruneRegions();
    WriteRegions();
}

bool ZeroTimePlanner::PlanAlternativePaths(const stateType & attractor,
                                           const std::vector<stateType>& current_path,
                                           std::vector<std::vector<stateType>>& new_paths) {
    auto df = ims::getEmptyDistanceField();
    ros::Publisher one_marker_pub = nh.advertise<visualization_msgs::Marker>("occ_marker", 0);
    ros::Duration(0.1).sleep();
    int num_paths {1};
    for (auto& state : current_path){
        moveit::core::RobotState robot_state = *m_group->getCurrentState();
        // print the robot state joints
        robot_state.setJointGroupPositions(m_group->getName(), state);
        std::vector<std::vector<int>> occupancy_grid;
        ims::getRobotOccupancy(df, robot_state, m_group, occupancy_grid);

    }
    unsigned int count_occ_cells = ims::countOccupiedCells(*df);
    // visualize all occupied cells
    ims::visualizeOccupancy(df, one_marker_pub, m_group->getPlanningFrame(), num_paths++);
    ros::Duration(0.1).sleep();
    return true;

}


void ZeroTimePlanner::Query(std::vector<ims::state*> &path) {

}

void ZeroTimePlanner::GraspQuery(std::vector<stateType> &path, std::string grasp_dir) {
    m_task_space->UpdateSearchMode(QUERY);
    if (grasp_dir.empty()){
        grasp_dir = read_write_dir;
    }
    ROS_INFO("Grasp dir: %s", grasp_dir.c_str());
    boost::filesystem::path dir(grasp_dir);
    // Loop over all files in directory
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(dir); itr != end_itr; ++itr) {
        std::string grasp_dir_ = itr->path().string();
        std::string grasp_name = itr->path().filename().string();
        // check if group name is a part of the file name. If not, continue
        if (grasp_name.find(m_task_space->getPlanningGroupName()) == std::string::npos) {
            continue;
        }
        ROS_INFO("Grasp dir: %s", grasp_dir_.c_str());
        ROS_INFO("Grasp: %s", grasp_name.c_str());

        m_regions.clear(); m_iregions.clear();
        m_task_space->PassRegions(&m_regions, &m_iregions);

        ReadRegions(grasp_dir_);
        ROS_INFO("Regions: %zu", m_regions.size());

        // TODO: Add a check to see if goal is in global XYZ goal region
        // get current time
        auto now = std::chrono::system_clock::now();
        ROS_INFO("Looking for region containing start state");
        // TODO: Make sure about the false here. Do I want to look for the closest region with or without heuristic?
        int reg_idx = m_task_space->FindRegionContainingState_WS(m_goal, false);
        auto find_time = std::chrono::system_clock::now();
        std::chrono::duration<double> find_time_elapsed = find_time - now;
        ROS_INFO("Find time: %f", find_time_elapsed.count());
        if (reg_idx == -1){
            ROS_INFO_STREAM("Query state not covered in file: " << grasp_name);
            continue;
        }

        ROS_INFO("Region index: %d", reg_idx);
        auto region = m_regions[reg_idx];
        // get or create both states
        auto attr_state_ind = m_task_space->getOrCreateState(region.state);
        auto attr_state = m_task_space->getState(attr_state_ind);
        if (attr_state->getMappedState().empty()){
            // set the mapped state to be the last state in the path
            attr_state->setMappedState(region.path.back());
        }
        auto goal_state_ind = m_task_space->getOrCreateState(m_goal);
        auto goal_state = m_task_space->getState(goal_state_ind);
        // TODO: make sure the following is ok
//        if (goal_state->getMappedState().empty()){
//            goal_state->setMappedState(attr_state->getMappedState());
//        }

        std::vector<int> greedy_path;
        if (!m_planner_zero->findGreedyPath(goal_state_ind, attr_state_ind, greedy_path)){
            ROS_INFO("No greedy path found");
            continue;
        }
        ROS_INFO("Greedy path size: %zu", greedy_path.size());
        path = region.path;
        // loop over the greedy path and get the IK of al states. Add to greedy path to the path
        auto seed = attr_state->getMappedState();
        for (auto state_ind : greedy_path){
            auto state_ = m_task_space->getState(state_ind);
//            if (state_->getMappedState().empty()){
            stateType joint_state;
            m_task_space->getIKSolution(state_->getState(), seed, joint_state);
//                m_task_space->getIKSolution(state_->getState(), joint_state);
            state_->setMappedState(joint_state);
//            }
            path.push_back(state_->getMappedState());   // state_->getState()
            seed = state_->getMappedState();
        }
        break;
    }
}

void ZeroTimePlanner::InitMoveitOMPL() {
    m_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(arm_name);
    ROS_INFO("Planning path with OMPL");
    m_group->setPlanningTime(5.0);
    m_group->setPlannerId("BiTRRT");
}

void ZeroTimePlanner::InitwAstarIMS() {
    m_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(arm_name);
    ROS_INFO("Planning path with IMS");
    double weight = 10.0;
    auto* heu = new ims::SE3HeuristicRPY;
    ims::wAStarParams wAparams (heu, weight);
    m_wAStar_ptr = std::make_unique<ims::wAStar>(wAparams);
}

bool ZeroTimePlanner::PlanPathFromStartToAttractorOMPL(const stateType &attractor, std::vector<stateType> &path) {

    ROS_INFO("Planning path with OMPL");

    // start -> actual start
    // goal -> attractor

    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_state::RobotState goal_state(*m_group->getCurrentState());
    goal_state.setJointGroupPositions(arm_name, attractor);    // "manipulator_1"
    m_group->setJointValueTarget(goal_state);

    robot_state::RobotState start_state(*m_group->getCurrentState());
    // TODO: Add the IK from m_start_state to start_state here and make sure they are the same!
    m_group->setStartState(start_state);
    ROS_INFO_STREAM(m_group->getName());
    // plan
    ROS_INFO("Going to plan!");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto ret = m_group->plan(my_plan);
    sleep(1); // Wanted to 0.1 but 'sleep' takes unsigned ints only. TODO: Check other sleep method

    if (ret != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_WARN("OMPL failed to plan");
        return false;
    }
    else {
        ROS_INFO("Solution found by OMPL");
    }

    // fill path
    path.resize(my_plan.trajectory_.joint_trajectory.points.size());
    for (size_t i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); ++i) {
        auto positions = my_plan.trajectory_.joint_trajectory.points[i].positions;
        path[i] = positions;
    }
    spinner.stop();

    return true;
}

bool ZeroTimePlanner::PlanPathFromStartToAttractorIMS(const stateType &attractor, std::vector<stateType> &path) {
    stateType start_state;
    m_task_space->getWorkspaceState(start_state);
    ROS_INFO_STREAM("Start state: " << start_state[0] << " " << start_state[1] << " " << start_state[2]
                    << " " << start_state[3] << " " << start_state[4] << " " << start_state[5] << std::endl);
    ROS_INFO_STREAM("Attractor: " << attractor[0] << " " << attractor[1] << " " << attractor[2]
                    << " " << attractor[3] << " " << attractor[4] << " " << attractor[5] << std::endl);
    // TODO: Lets assume for the moment that the attractor is discretized
    try{
        m_wAStar_ptr->initializePlanner(m_task_space, start_state, attractor);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return false;
    }
    std::vector<ims::state*> ims_path;
    if (!m_wAStar_ptr->plan(ims_path)){
        ROS_WARN("IMS failed to plan");
        return false;
    }
    else {
        ROS_INFO("Solution found by IMS");
    }
    path.resize(ims_path.size());
    for (size_t i = 0; i < ims_path.size(); ++i) {
        auto state = ims_path[i];
        path[i] = state->getState();
    }
    return true;
}

void ZeroTimePlanner::WriteRegions(std::string path) {
    // sort
    std::sort(m_regions.begin(), m_regions.end(), [] (const region &a,
                                                      const region &b)
    {
        return (a.radius > b.radius);
    });
    if (path.empty()){
        path = read_write_path;
    }
    ROS_INFO("Writing regions to file");
    boost::filesystem::path myFile = path; //boost::filesystem::current_path() /
    std::cout << myFile;
    boost::filesystem::ofstream ofs(myFile);
    boost::archive::text_oarchive ta(ofs);
    ta << m_regions;
}


void ZeroTimePlanner::ReadRegions(std::string path) {

    ROS_INFO("Reading regions from file");
    // getchar();
    try {
        if (path.empty()){
            path = read_write_path;
        }
        boost::filesystem::path myFile = path; // boost::filesystem::current_path() /
        boost::filesystem::ifstream ifs(myFile/*.native()*/);
        boost::archive::text_iarchive ta(ifs);
        ta >> m_regions;
    }
    catch (...) {
        ROS_WARN("Unable to read preprocessed file");
    }
}

bool ZeroTimePlanner::plan(std::vector<stateType> &path) {
    if (!m_query_mode){
        ROS_INFO("Preprocessing goal regions");
        int start_ind = m_task_space->getOrCreateState(m_start_state);
        PreProcess(m_task_space->getState(start_ind)->getMappedState());
        return false;
    }
    else {
        ROS_INFO("Querying");
        GraspQuery(path);
        if (path.empty()) {
            ROS_WARN("No path found");
            return false;
        } else {
            ROS_INFO("Path found");
            return true;
        }
    }
}

bool ZeroTimePlanner::GetPathFromPreGraspToGrasp(const stateType &pre_grasp, robot_state::RobotStatePtr &robot_state,
                                                 const stateType &grasp, std::vector<stateType> &path) {
    // Assuming that the pre_grasp and grasp are (x, y, z, r, p, y)
    // initial state (configuration space)
    std::vector<double> theta0;
    Eigen::VectorXd grasp_vec(6);
    Eigen::VectorXd pre_grasp_vec(6);
    for (size_t i = 0; i < 6; ++i) {
        grasp_vec[i] = grasp[i];
        pre_grasp_vec[i] = pre_grasp[i];
    }
    pre_grasp_vec[2] += 0.2;


    robot_state->copyJointGroupPositions(arm_name, theta0);
    path.push_back(theta0);
    // get the Jacobian
    Eigen::MatrixXd J;
    auto joint_model_group = robot_state->getJointModelGroup(arm_name);
    robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             Eigen::Vector3d(0, 0, 0), J);
    // get the Jacobian pseudo inverse

//    Eigen::MatrixXd J_pinv;
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    double tolerance = std::max(J.cols(), J.rows()) * svd.singularValues().array().abs().maxCoeff() * std::numeric_limits<double>::epsilon();
//    J_pinv = svd.matrixV() * Eigen::MatrixXd(svd.singularValues().array().abs().inverse().matrix().asDiagonal()) * svd.matrixU().transpose();

    double h = 1 / 10.0;
    auto theta = theta0;
    auto curr_grasp_vec = pre_grasp_vec;
    Eigen::MatrixXd J_inv;
    for (int i{0} ; i < 10; i++){
        J_inv = J.transpose() * (J * J.transpose()).inverse();
        // RK(4)
        auto k1 = h * (J_inv * (grasp_vec - pre_grasp_vec));
        auto k2 = h * (J_inv * (grasp_vec - pre_grasp_vec + k1 / 2.0));
        auto k3 = h * (J_inv * (grasp_vec - pre_grasp_vec + k2 / 2.0));
        auto k4 = h * (J_inv * (grasp_vec - pre_grasp_vec + k3));
        auto del_theta = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
        for (size_t j = 0; j < 6; ++j) {
            theta[j] += del_theta[j];
        }
        path.push_back(theta);
        // update the robot state
        robot_state->setJointGroupPositions(arm_name, theta);
        robot_state->update();
        // check if the new state is in collision and out of bounds
        if (!robot_state->satisfiesBounds(joint_model_group)) {
            ROS_WARN("Out of bounds");
        }
        // check if the new state is in collision
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.group_name = arm_name;
        collision_result.clear();
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
    std::cout << "Path in degrees" << std::endl;
    for (auto & i : path) {
        for (size_t j = 0; j < 6; ++j) {
            i[j] = i[j] * 180.0 / M_PI;
            std::cout << i[j] << " ";
        }
        std::cout << std::endl;
    }

    return true;
}









