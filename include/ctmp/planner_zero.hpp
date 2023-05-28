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

#ifndef CTMP_PLANNER_ZERO_HPP
#define CTMP_PLANNER_ZERO_HPP

#include <ctmp/ctmpActionSpace.hpp>

#include <planners/BestFirstSearch.hpp>
#include <common/state.hpp>
#include <utility>
#include <unordered_set>


namespace ims {

    class plannerZero : public BestFirstSearch{
    public:

        explicit plannerZero(const BestFirstSearchParams &params);

        void initializePlanner(const std::shared_ptr<ctmpActionSpace>& actionSpacePtr,
                               const stateType& start, const stateType& goal) ;

        void reinitSearchState(int s_ind);

        bool is_state_covered(int id);

//        unsigned int compute_reachability(unsigned int r_max,
//                                 int attractor_state_id);
        double compute_reachability(double r_max,
                                          int attractor_state_id);


        void get_frontier_stateids(std::vector<int>& state_ids);

//        int search_for_valid_uncovered_states(
//                unsigned int r_max,
//                int iv_start_state_id);

        double search_for_valid_uncovered_states(
                double r_max,
                int iv_start_state_id);

        /// @brief Find a greedy path from goal state to attractor state and then return the reverse path.
        /// @param goal_state_id The goal state id.
        /// @param attr_state_id The attractor state id.
        /// @param path The path from goal to attractor state.
        /// @return True if a path was found, false otherwise.
        bool findGreedyPath(int goal_state_id, int attr_state_id, std::vector<int>& path);


    protected:

        std::shared_ptr<ctmpActionSpace> m_actionSpacePtr;

        int m_call_number;

        int m_iteration;

        std::vector<state*> m_succs;
        std::vector<state*> m_preds;
        std::vector<double> m_costs;

        // reachability
        int m_reachability_expansions;
        int m_search_mode;
        state* m_best_state;
//        int m_h_max;
        double m_h_max;
        std::vector<state*> m_h_max_states;
        // object which contains all state index which where init during compute_reachability.
        // needs to be fat to look for a state
        std::unordered_set<int> m_state_initiated;

    };
}


#endif //CTMP_PLANNER_ZERO_HPP
