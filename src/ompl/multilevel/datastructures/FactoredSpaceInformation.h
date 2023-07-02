/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023,
 *  Technical University of Berlin
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
 *   * Neither the name of TU Berlin nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOREDSPACEINFORMATION_
#define OMPL_MULTILEVEL_PLANNERS_FACTOREDSPACEINFORMATION_

#include <ompl/base/SpaceInformation.h>
#include <ompl/util/ClassForward.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(StateSpace);
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::FactoredSpaceInformation */
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);
        OMPL_CLASS_FORWARD(Projection);
        /// @endcond
        class FactoredSpaceInformation : public base::SpaceInformation, public std::enable_shared_from_this<FactoredSpaceInformation>
        {
        public:
            FactoredSpaceInformation() = delete;
            FactoredSpaceInformation(const FactoredSpaceInformation &) = delete;
            FactoredSpaceInformation &operator=(const FactoredSpaceInformation &) = delete;

            FactoredSpaceInformation(const base::StateSpacePtr& space);

            virtual ~FactoredSpaceInformation() = default;

            void printSettings(std::ostream &out) const override;

            void setup() override;

            std::string getName() const;

            /** \brief addChild: Add a factor space as a child plus a projection
             * which defines how we map states from this factor space to the
             * added child.
             */
            bool addChild(FactoredSpaceInformationPtr factor_si, ProjectionPtr projection);

            void setProjectionToParent(ProjectionPtr projection);
            const ProjectionPtr& getProjection() const;

            void setParent(FactoredSpaceInformationPtr factor_si);
            const FactoredSpaceInformationPtr& getParent() const;

            bool hasParent() const;

            const std::vector<FactoredSpaceInformationPtr>& getChildren() const;
            const FactoredSpaceInformationPtr& getChild(const std::string& name) const;
            bool hasChildren() const;

            std::vector<FactoredSpaceInformationPtr> getLeafFactors();
            std::vector<FactoredSpaceInformationPtr> getAllFactors();

            /** \brief lift: Map states from children factors to this factor and
             * store the result in state
             */
            void lift(const std::unordered_map<std::string, base::State*>& childStates_, base::State* state) const;

            /** \brief project: Take state from this factor and map it to child
             * states
             */
            //void project(const base::State* state, std::unordered_map<std::string, base::State*>& childStates) const;
        private:
            ProjectionPtr projection_to_parent_{nullptr};

            FactoredSpaceInformationPtr parent_{nullptr};

            std::vector<FactoredSpaceInformationPtr> children_;
        };
    }
}
#endif

