/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/geometric/planners/rrt/GMRRRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include <ompl/base/spaces/SE3StateSpace.h>


ompl::geometric::GMRRRT::GMRRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "GMRRRTintermediate" : "GMRRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &GMRRRT::setRange, &GMRRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &GMRRRT::setGoalBias, &GMRRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &GMRRRT::setIntermediateStates, &GMRRRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::GMRRRT::~GMRRRT()
{
    freeMemory();
}

void ompl::geometric::GMRRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    // sampler_rrt.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::GMRRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::GMRRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

// ompl::base::ValidStateSamplerPtr ompl::geometric::RRT::allocGaussianValidStateSampler(const ompl::base::SpaceInformation *si)
// {
//     // si_->stateSpace_->setStateSamplerAllocator(
//     //     [this](ompl::base::StateSpace* ss){ return std::make_shared<ompl::base::GaussianValidStateSampler>(ss); });
//     // sampler_ = si_->stateSpace_->allocStateSampler();
//     return std::make_shared<ompl::base::GaussianValidStateSampler>(si);
// }

ompl::base::PlannerStatus ompl::geometric::GMRRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);

        // std::cout << "************************* I am RRT ************************* " << std::endl;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // std::cout << "************************* I am RRT ************************* " << std::endl;
    std::cout << "State Dimension: " << si_->getStateDimension() << std::endl;
    std::cout << "StateSpace Name: " << si_->getStateSpace()->getName() << std::endl;    
    std::cout << "StateSpace Type: " << si_->getStateSpace()->getType() << std::endl;
    //
    // si_->setValidStateSamplerAllocator(
    //     [this](const ompl::base::SpaceInformation* si){ return allocGaussianValidStateSampler(si); });
    // //\
    
    // if (!sampler_rrt)
    if (!sampler_){
        // std::cout << "NNOO SAMPPLER IS CHOOSEN" << std::endl;
        sampler_ = si_->allocStateSampler();
        // sampler_rrt = si_->allocValidStateSampler();
    }
        

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        bool samplegoal_flag = false;
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample()){
            goal_s->sampleGoal(rstate);
            std::cout << "####### SAMPLE GOAL " << std::endl;
            samplegoal_flag = true;
            }
        else{
            std::cout << "####### SAMPLE Gaussian" << std::endl;
            sampler_->sampleUniform(rstate);
            // sampler_->sampleGaussian(rstate, xstate, 0.1);
            // sampler_rrt->sampleNear(rstate, rstate, 0.1); // only the 1st arg is necessary
            }

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double D = si_->getStateSpace()->distance(nmotion->state, rstate);
        std::cout << "*********** StateSpace Distance = " << D << " *************" << std::endl;
        double D2 = si_->distance(nmotion->state, rstate);
        std::cout << "*********** SpaceInfo Distance = " << D2 << " *************" << std::endl;        
        // std::cout << "*********** MAXDistaaaance = " << maxDistance_ << " *************" << std::endl;
        // if (d > maxDistance_)
        // {
        //     si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
        //     dstate = xstate;
        //     std::cout << "############################"<< std::endl
        //               << "##### OUT OF DISTANCE #####" << std::endl
        //               << "############################"<< std::endl;
        // }

        /* Check if goal is far away from GMR samples and terminate if so*/
        double d = sqrt(pow(nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],2)
                      + pow(nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],2)
                      + pow(nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] - rstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[2],2));
        std::cout << "*********** Euclidean Distaaaance = " << d << " *************" << std::endl;
        if (D > 0.2 && samplegoal_flag)
        {
            std::cout << "Goal is Far away!" << std::endl;
            break;
        }
        
        if (si_->checkMotion(nmotion->state, dstate))
        {
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
            }
            else
            {          
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::GMRRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
