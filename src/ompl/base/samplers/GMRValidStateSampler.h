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

#ifndef OMPL_BASE_SAMPLERS_GMR_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_GMR_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <Eigen/Dense>

namespace ompl
{
    namespace base
    {
        // /// @cond IGNORE
        // /** \brief Forward declaration of ompl::base::ValidStateSampler */
        // OMPL_CLASS_FORWARD(GMRValidStateSampler);
        // /// @endcond

        /** \brief Generate valid samples using the Gaussian sampling strategy */
        class GMRValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            GMRValidStateSampler(const SpaceInformation *si, Eigen::MatrixXf&  GMM_X, 
                const std::vector<Eigen::VectorXf>&  GMM_means, 
                const std::vector<float>&  GMM_weights , const std::vector<Eigen::MatrixXf>&  GMM_covariances);

            ~GMRValidStateSampler() override = default;

            bool sample(State *state) override;
            bool sampleNear(State *state, const State *near, double distance) override;
            // bool sampleGMR(State *state) override;

            /** \brief Get the standard deviation used when sampling */
            double getStdDev() const
            {
                return stddev_;
            }

            /** \brief Set the standard deviation to use when sampling */
            void setStdDev(double stddev)
            {
                stddev_ = stddev;
            }

        protected:
            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The standard deviation to use in the sampling process */
            double stddev_;

            Eigen::MatrixXf gmm_X;
            std::vector<Eigen::VectorXf> gmm_means;
            std::vector<float> gmm_weights;
            std::vector<Eigen::MatrixXf> gmm_covariances;                        
        };

        // /** \brief Definition of a function that can allocate a valid state sampler */
        // using GMRValidStateSamplerAllocator = std::function<GMRValidStateSamplerPtr(const SpaceInformation *)>;
    }
}

#endif
