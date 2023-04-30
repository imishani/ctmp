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

#ifndef CTMP_CTMP_UTILS_HPP
#define CTMP_CTMP_UTILS_HPP


#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/serialization/vector.hpp>
#include <set>
#include <random>
#include <eigen_conversions/eigen_msg.h>

// system includes
#include <ros/ros.h>

#include <common/types.hpp>


namespace ims {
    inline void rad2deg(stateType& state) {
        for (auto& val : state) {
            val = val * 180 / M_PI;
        }
    }

    inline void deg2rad(stateType& state) {
        for (auto& val : state) {
            val = val * M_PI / 180;
        }
    }

    inline void getSE3RPYdistance(const stateType& s1, const stateType& s2, double& distance){
        Eigen::Vector3d pos1 {s1[0], s1[1], s1[2]};
        Eigen::Vector3d pos2 {s2[0], s2[1], s2[2]};
        // get the orientation of the states
        // Transform from RPY to quaternion
        Eigen::Quaterniond quat1 = Eigen::AngleAxisd(s1[5], Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(s1[4], Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(s1[3], Eigen::Vector3d::UnitX());
        Eigen::Quaterniond quat2 = Eigen::AngleAxisd(s2[5], Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(s2[4], Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(s2[3], Eigen::Vector3d::UnitX());
        // get the distance between the positions
        distance = (pos1 - pos2).norm();
        // get the distance between the orientations
        distance += 2 * std::acos(std::min(1.0, std::abs(quat1.dot(quat2))));
    }
}



struct region
{
    friend class boost::serialization::access;
    stateType start;
//    unsigned int radius;
    double radius;
    stateType state; // workspace state
    std::vector<stateType> path;

    template <typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & start;
        ar & radius;
        ar & state;
        ar & path;
    }
};

enum Modes
{
    REACHABILITY = 0,
    QUERY = 1
};



#endif //CTMP_CTMP_UTILS_HPP
