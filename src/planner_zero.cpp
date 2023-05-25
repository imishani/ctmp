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

#include "ctmp/planner_zero.hpp"

ims::plannerZero::plannerZero(const BestFirstSearchParams &params) :
    BestFirstSearch(params),
    m_iteration(1),
    m_call_number(0),
    m_reachability_expansions(0),
    m_h_max(0){
}


void ims::plannerZero::initializePlanner(const std::shared_ptr<ctmpActionSpace> &actionSpacePtr, const stateType &start,
                                     const stateType &goal) {
    // space pointer
    m_actionSpacePtr = actionSpacePtr;
    // check if start is valid
    stateType joint_state_start;
    m_actionSpacePtr->getJointStates(joint_state_start);
    if (!m_actionSpacePtr->isStateValid(start, joint_state_start)){
        ROS_ERROR("Start state is not valid");
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    stateType joint_state_goal = joint_state_start;
//    m_actionSpacePtr->getJointStates(joint_state_goal);
    if (!m_actionSpacePtr->isStateValid(goal, joint_state_goal)){
        ROS_ERROR("Goal state is not valid");
//        throw std::runtime_error("Goal state is not valid");
    }
    int m_start_ind = m_actionSpacePtr->getOrCreateState(start);
    printf("start ind: %d \n", m_start_ind);
    m_start = m_actionSpacePtr->getState(m_start_ind);
    m_start->setMappedState(joint_state_start);
    m_start->setParent(START);
    int m_goal_ind = m_actionSpacePtr->getOrCreateState(goal);
    m_goal = m_actionSpacePtr->getState(m_goal_ind);
//    m_goal->setMappedState(joint_state_goal);
    m_goal->setParent(GOAL);
    m_heuristic->setGoal(m_goal);
    // Evaluate the start state
    m_start->g = 0;
    m_start->h = computeHeuristic(m_start);
    m_start->f = m_start->h;
//    m_open.push(m_start);
//    m_start->setOpen();
    // Evaluate the goal state
    m_goal->h = 0;
}

void ims::plannerZero::reinitSearchState(int s_ind) {
    // check if state index has been initiated already
    if (m_state_initiated.find(s_ind) == m_state_initiated.end()){
        m_state_initiated.insert(s_ind);
        state* s = m_actionSpacePtr->getState(s_ind);
        s->covered_this = false;
        s->greedy = false;
        s->h = Infinity;
        s->f = Infinity;
        s->resetFlags();
    }
}


bool ims::plannerZero::is_state_covered(int id) {
    state* search_state = m_actionSpacePtr->getState(id);
    return search_state->covered;
}


//unsigned int ims::plannerZero::compute_reachability(unsigned int r_max, int attractor_state_id) {
double ims::plannerZero::compute_reachability(double r_max, int attractor_state_id) {
    ++m_call_number;
    m_iteration = 1;
    m_h_max = 0;
    m_h_max_states.clear();
    m_state_initiated.clear();
    m_open.clear();

    auto attractor_state = m_actionSpacePtr->getState(attractor_state_id);
    reinitSearchState(attractor_state->getStateId());
    attractor_state->greedy = true;
    attractor_state->covered = true;
    attractor_state->h = 0;
    attractor_state->setClosed();

    m_preds.clear();
    m_costs.clear();
    m_actionSpacePtr->getSuccessors(attractor_state_id, m_preds, m_costs);

    for (const auto& pred : m_preds) {
        reinitSearchState(pred->getStateId());
        pred->h = computeHeuristic(pred, attractor_state);
        pred->f = pred->h;
        if (m_open.contains(pred)) {
            m_open.decrease(pred);
        } else {
            m_open.push(pred);
        }
    }

//    unsigned int radius = 0;
    double radius = 0;

    while (radius <= r_max && !m_open.empty()) {
        // getchar();
        m_succs.clear();
        m_costs.clear();
        state* min_state = m_open.min();
        m_open.pop();
        min_state->setClosed();
        m_reachability_expansions++;
        if (min_state->h > m_h_max) {
            m_h_max_states.clear();
        }
        m_h_max = min_state->h;

        // if not previously covered
        if (!min_state->covered) {
            min_state->covered = true;
            min_state->covered_this = true;
        }

        m_h_max_states.push_back(min_state);
        ////
        // Check for min_state being in joint state:
        ///@{ Greedy successor --line 7
        auto start = std::chrono::system_clock::now();
        m_actionSpacePtr->getSuccessors(min_state->getStateId(),
                                        m_succs, m_costs);

        state* succ_state_g = nullptr;
        double min_h = Infinity;
        std::vector<state*> greedy_succs;

        for (const auto& succ_state : m_succs) {
            reinitSearchState(succ_state->getStateId());
//            succ_state->h = computeHeuristic(succ_state, attractor_state);
            if (succ_state->h < min_h) {
                min_h = succ_state->h;
                succ_state_g = succ_state;
            }
        }
        //tie_breaking
#ifdef false
        for (const auto& succ_id : m_succs) {
            SearchState* succ_state = getSearchState(succ_id);
            if (succ_state->h == min_h) {
                greedy_succs.push_back(succ_state);
            }
        }

        for (const auto& s : greedy_succs) {
            if (m_task_space->IsStateToStateValid(min_state->state_id, s->state_id)) {
                succ_state_g = s;
                break;
            }
        }
#endif
        std::chrono::duration<double, std::micro> duration = std::chrono::system_clock::now() - start;

        ///@}

        ///@{ Greedy set criteria --line 8-9
        start = std::chrono::system_clock::now();
        stateType joint_states_min = min_state->getMappedState();
        if (succ_state_g != nullptr && (succ_state_g->greedy &&
//            m_actionSpacePtr->isStateToStateValid(min_state->getState(), succ_state_g->getState()))) {
            !joint_states_min.empty())) {
            min_state->greedy = true;
            m_actionSpacePtr->VisualizePoint(min_state->getStateId(), "greedy");
        }
            ///@}

            ///@{ Terminating condition --line 10-11
//        else if (m_actionSpacePtr->isStateValid(min_state->getState(), joint_states_min)){
        else if (!joint_states_min.empty()){   // TODO: Its a hack. I check validity of the state when generating successors and  if valid then I save mappedstate.
            m_actionSpacePtr->VisualizePoint(min_state->getStateId(), "exited");
            radius = min_state->h;

            // unset covered
            for (auto s : m_h_max_states) {
                if (s->h == min_state->h) {
                    if (s->covered_this) {
                        s->covered = false;
                    }
                }
            }

            break;
        }
        else {
            m_actionSpacePtr->VisualizePoint(min_state->getStateId(), "non_greedy");
        }
        duration = std::chrono::system_clock::now() - start;
        ROS_DEBUG_STREAM("Greedy set criteria: " << duration.count());
        ///@}

        ///@{ Set radius --line 12
        radius = min_state->h;

        // if (m_reachability_expansions % 100 == 0)
        //     SMPL_INFO_NAMED(SRLOG, "Radius so far: %d, max: %d", radius, r_max);
        ///@}

        ///@{ Insert Preds in Open list --line 13
        start = std::chrono::system_clock::now();
        ROS_DEBUG_NAMED("reachability", "Inserting preds in Open");
        for (const auto& pred_state : m_succs) {   // because preds == succs
            // if (m_greedy.find(pred_id) == m_greedy.end()) {
            reinitSearchState(pred_state->getStateId());
            if (!pred_state->greedy) {
                if (!pred_state->isClosed()) {
                    pred_state->h = computeHeuristic(pred_state, attractor_state);;
                    pred_state->f = pred_state->h;
                    if (m_open.contains(pred_state)) {
                        m_open.decrease(pred_state);
                    } else {
                        m_open.push(pred_state);
                    }
                }
            }
        }
        duration = std::chrono::system_clock::now() - start;
        ROS_DEBUG_NAMED("REACHABILITY", "line 13: %f", duration.count());
        ///@}

        ROS_DEBUG_NAMED("REACHABILITY", "----------------------------------------");
    }

    if (m_open.empty()) {
        printf("Valid Open list got empty\n");
        radius += 0.02;
    }
    /// line 14
    return radius;
}

double ims::plannerZero::search_for_valid_uncovered_states(double r_max, int iv_start_state_id) {
    ++m_call_number;
    m_iteration = 1;
    m_h_max = 0;
    m_h_max_states.clear();
    m_state_initiated.clear();
    m_open.clear();

    auto iv_start_state = m_actionSpacePtr->getState(iv_start_state_id);
    reinitSearchState(iv_start_state->getStateId());
    iv_start_state->h = 0;
    iv_start_state->f = iv_start_state->h;
    m_open.push(iv_start_state);

    double radius = 0;
    while (radius <= r_max && !m_open.empty()){
        m_preds.clear(); m_costs.clear();
        auto min_state = m_open.min();
        m_open.pop();
        min_state->setClosed();

        if (min_state->h > m_h_max) {
            m_h_max_states.clear();
        }
        m_h_max = min_state->h;

        // if not previously covered
        if (!min_state->covered) {
            min_state->covered = true;
            min_state->covered_this = true;
        }

        m_h_max_states.push_back(min_state);

        m_actionSpacePtr->VisualizePoint(min_state->getStateId(), "invalid");
        stateType joint_states_min = min_state->getMappedState();
        if (m_actionSpacePtr->isStateValid(min_state->getState(), joint_states_min) &&
            !m_actionSpacePtr->IsStateCovered(true, min_state->getStateId())) {
            m_open.push(min_state);
            radius = min_state->h;
            // unset covered
            for (auto s : m_h_max_states) {
                if (s->h == min_state->h) {
                    if (s->covered_this) {
                        s->covered = false;
                    }
                }
            }
            break;
        }
        else {
            m_actionSpacePtr->getSuccessors(min_state->getStateId(), m_preds, m_costs);
            for (auto &m_pred: m_preds) {
                reinitSearchState(m_pred->getStateId());
                if (!m_pred->isClosed()) {
                    m_pred->h = computeHeuristic(m_pred, iv_start_state);
                    m_pred->f = m_pred->h;
                    if (m_open.contains(m_pred)) {
                        m_open.decrease(m_pred);
                    } else {
                        m_open.push(m_pred);
                    }
                }
            }
        }
        radius = min_state->h;
    }
    if (m_open.empty()) {
        printf("Invalid Open list got empty\n");
        radius += 0.02;
    }
    return radius;
}

//int ims::plannerZero::search_for_valid_uncovered_states(const unsigned int r_max, const int iv_start_state_id) {
//    return 0;
//}

void ims::plannerZero::get_frontier_stateids(std::vector<int> &state_ids) {
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        state_ids.push_back((*it)->getStateId());
    }
}

bool ims::plannerZero::findGreedyPath(int goal_state_id, int attr_state_id, std::vector<int>& path){
    auto goal_state = m_actionSpacePtr->getState(goal_state_id);
    auto attr_state = m_actionSpacePtr->getState(attr_state_id);
    goal_state->h = computeHeuristic(goal_state, attr_state);
    int state_ind = goal_state_id;
    // get the current time
    auto start_time = std::chrono::system_clock::now();
    // max time:
    double max_time = 3; // seconds
    while (state_ind != attr_state_id ) {
        m_preds.clear(); m_costs.clear();
        path.push_back(state_ind);
        auto state = m_actionSpacePtr->getState(state_ind);
        if (state->h < 1.79e-2) {
            break;
        }
        // check if time is more than max time
        auto current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = current_time - start_time;
        if (duration.count() > max_time) {
            ROS_WARN("Timeout in finding greedy path");
            ROS_INFO("Current h: %f", state->h);
            return false;
        }
        m_actionSpacePtr->getSuccessors(state_ind, m_preds, m_costs);
        double min_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < m_preds.size(); ++i) {
            // get the heuristic value of the predecessor
            auto pred_state = m_preds[(int)i];
            pred_state->h = computeHeuristic(pred_state, attr_state);
            if (pred_state->h < min_cost) {
                min_cost = pred_state->h;
                state_ind = pred_state->getStateId();
            }
        }
    }
    if (false) { // isTimeOut()
        ROS_WARN("Timeout in finding greedy path");
        return false;
    }
    else {
        // reverse the path
        std::reverse(path.begin(), path.end());
        return true;
    }
}


