/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/base/samplers/GMMValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"

// #include "/home/zizo/haptics-ctrl_ws/src/gmm_gmr/include/doRegression.h"
#include "/home/zizo/haptics-ctrl_ws/src/data_handle/include/data_handle/GMMHandler.h"

ompl::base::GMMValidStateSampler::GMMValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , stddev_(si->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
{
    name_ = "gmm_validstatesampler";
    params_.declareParam<double>("standard_deviation",
                                 [this](double stddev)
                                 {
                                     setStdDev(stddev);
                                 },
                                 [this]
                                 {
                                     return getStdDev();
                                 });
    getGMMfromBag();
    // nb_GMM_samples = 5000;        // uncomment to sample with GMM_sampler
    // drawSamples(nb_GMM_samples);  // uncomment to sample with GMM_sampler
}


bool ompl::base::GMMValidStateSampler::sample(State *state)
{
    std::cout << "GMMValidStateSampler: sample" << std::endl;

    // std::cout << "position.x: " << state->getX() << std::endl;
    
    // bool result = false;
    // unsigned int attempts = 0;
    // State *temp = si_->allocState();
    // do
    // {
    //     sampler_->sampleUniform(state);
    //     bool v1 = si_->isValid(state);
    //     sampler_->sampleGaussian(temp, state, stddev_);
    //     bool v2 = si_->isValid(temp);
    //     if (v1 != v2)
    //     {
    //         if (v2)
    //             si_->copyState(state, temp);
    //         result = true;
    //     }
    //     ++attempts;
    // } while (!result && attempts < attempts_);
    // si_->freeState(temp);
    // return result;

    ompl::RNG rnd_input;
    std::vector<float> prestate = doRegression(rnd_input.uniformInt(0, GMM_sample_indx-1));   // uncomment to sample with GMR_sampler // The -1 to avoid out of range error
    // std::vector<float> prestate= samplefromGMM(rnd_input.uniformInt(0, nb_GMM_samples-1)); // uncomment to sample with GMM_sampler


    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = prestate[0];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = prestate[1];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = prestate[2];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = prestate[3];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = prestate[4];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = prestate[5];
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = prestate[6];

    std::vector<double> state_vec = {state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5],
                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6]};
    saveSamples(state_vec);
    
    std::cout << state_vec[0] << std::endl;
    std::cout << state_vec[1] << std::endl;
    std::cout << state_vec[2] << std::endl;
    std::cout << state_vec[3] << std::endl;
    std::cout << state_vec[4] << std::endl;
    std::cout << state_vec[5] << std::endl;
    std::cout << state_vec[6] << std::endl;
    
    return true;
}

bool ompl::base::GMMValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    std::cout << "GMMValidStateSampler: sampleNear" << std::endl;

    // Debugging

    std::cout << "state.position.x(before): " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << std::endl;
    std::cout << "state.position.y(before): " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
    std::cout << "state.position.z(before): " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << std::endl;

    // sampler_->sampleUniform(state);
    // std::cout << "state.position.x (before): " << 
    //     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << std::endl;
    // state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.1;
    // std::cout << "state.position.x: " << 
    //     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << std::endl;

    // //// The following print code shows that the space doesn't have Orientation!!
    // // std::cout << "state.orientation.w (before): " << 
    // //     state->as<ompl::base::SO3StateSpace::StateType>()->w << std::endl;
    // // state->as<ompl::base::SO3StateSpace::StateType>()->w = 0.2;
    // // std::cout << "state.orientation.w: " << 
    // //     state->as<ompl::base::SO3StateSpace::StateType>()->w << std::endl;
    // ////\ The following print code shows that the space doesn't have Orientation!!
    //\ Debugging

    // RNG rnd_input;
    // std::vector<float> prestate = doRegression(rnd_input.uniformReal(0.0, 0.6), gmm_X, gmm_means, gmm_weights, gmm_covariances);
    
    // state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = prestate[0];
    // state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = prestate[1];
    // state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = prestate[2];

    std::cout << "state.position.x: " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << std::endl;
    std::cout << "state.position.y: " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << std::endl;
    std::cout << "state.position.z: " << 
        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << std::endl;

    return true;
}

// bool ompl::base::GMMValidStateSampler::sampleNear(State *state, const State *near, const double distance)
// {
//     std::cout << "GMMValidStateSampler: sampleNear" << std::endl;
//     bool result = false;
//     unsigned int attempts = 0;
//     State *temp = si_->allocState();
//     do
//     {
//         sampler_->sampleUniformNear(state, near, distance);
//         bool v1 = si_->isValid(state);
//         sampler_->sampleGaussian(temp, state, distance);
//         bool v2 = si_->isValid(temp);
//         if (v1 != v2)
//         {
//             if (v2)
//                 si_->copyState(state, temp);
//             result = true;
//         }
//         ++attempts;
//     } while (!result && attempts < attempts_);
//     si_->freeState(temp);
//     return result;
// }
