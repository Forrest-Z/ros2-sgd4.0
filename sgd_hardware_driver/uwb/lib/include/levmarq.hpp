// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UWB__LEVMARQ_HPP_
#define UWB__LEVMARQ_HPP_

#include "IMultilateration.hpp"

#include <math.h>
#include <memory>
#include <utility>
#include <iostream>

#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>

namespace sgd_hardware_drivers
{

struct LMFunctor
{
    std::unordered_map<int, std::pair<double, double>> tags;
    std::unordered_map<int, double> measuredRanges;
    
    // number of parameters in the equation system: x_T, y_T
    //int n = 2;
    // number of measured data points: every anchor generates one measurement
    //int m;

    // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const{
        // x has dimension 2 x 1, it contains the current estimates for the parameters
        int i = 0;
        for (auto entry : measuredRanges)
        {
            if (tags.find(entry.first) == tags.end())
            {
                std::cout << "Could not find tag with id " << entry.first << std::endl;
                continue;
            }
            auto xy = tags.at(entry.first);
            
            fvec(i) = sqrt( pow(xy.first-x(0),2) + pow(xy.second-x(1),2) ) - entry.second;
            i++;
        }
        return 0;
    }

    // Compute the jacobian of the errors
    int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
    {
        // fjac is a m-by-n matrix containing the jacobian of the errors
        int i = 0;
        for (auto entry : measuredRanges)
        {
            auto xy = tags.at(entry.first);
            
            fjac(i, 0) = (x(0) - xy.first) / sqrt(pow(xy.first - x(0),2) + pow(xy.second - x(1), 2));
            fjac(i, 1) = (x(1) - xy.second) / sqrt(pow(xy.first - x(0),2) + pow(xy.second - x(1), 2));
            i++;
        }
        return 0;
    }

    // Returns the number of parameters in the equation system: x_T, y_T
    int inputs() const
    {
        return 2;
    }

    // Returns 'm', the number of values
    int values() const
    {
        return measuredRanges.size();
    }
};

/**
 * @brief  Wrapper for Eigen/LevenbergMarquardt nonlinear optimization algorithm
 * 
 */
class LevMarq : public IMultilateration
{
private:
    std::shared_ptr<Eigen::MatrixXf> measuredValues;
    Eigen::Vector2f last_pose;

public:
    LevMarq();
    ~LevMarq() = default;

    std::pair<double, double> estimatePose() override;
};

}   // namespace sgd_hardware_drivers

#endif