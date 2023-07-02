#include "ompl/multilevel/planners/factor/RestrictionSampler.h"

#include "ompl/base/StateSpace.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include "ompl/multilevel/planners/factor/FactoredPlanner.h"

#include <algorithm>

using namespace ompl::multilevel;

RestrictionSampler::RestrictionSampler(const FactoredSpaceInformationPtr& factor, const std::vector<FactoredPlannerPtr>& children_planner) 
  : factor_(factor), ompl::base::StateSampler(factor->getStateSpace().get()) 
{
    baseSampler_ = StateSampler::space_->allocDefaultStateSampler();

    if (!factor_->hasChildren())
    {
      OMPL_ERROR("Cannot create RestrictionSampler with a childless factor.");
      throw "NoChildrenError";
    }

    const auto& children = factor_->getChildren();

    if (children.size() != children_planner.size())
    {
      OMPL_ERROR("Number of children %d does not match children planner (%d)", children.size(), children_planner.size());
      throw "MismatchChildrenPlannerError";
    }

    for(const auto& child : children) 
    {
      const auto& name = child->getName();

      auto iterator = std::find_if(children_planner.begin(), children_planner.end(),
         [name](const auto& planner) {
          return planner->getFactoredSpaceInformation()->getName() == name;
         }
       );

      if(iterator == children_planner.end()) 
      {
        OMPL_ERROR("Could not find %s in children planner.", name.c_str());
        throw "NonExistingChildrenPlannerError";
      }

      childrenPlanner_[name] = *iterator;
      statesChild_[name] = child->allocState();
      childNames_.push_back(name);
    }

    OMPL_INFORM("Created restriction sampler for %d children.", childrenPlanner_.size());
}

void RestrictionSampler::sampleUniform(base::State *state) {
    ////////////////////////////////////////////////////////////////////////////////
    //Obtain samples from all children
    ////////////////////////////////////////////////////////////////////////////////
    for(const auto& name : childNames_) {
        OMPL_INFORM("Sampling from %s", name.c_str());
        childrenPlanner_[name]->sampleFromDatastructure(statesChild_[name]);
    }

    ////////////////////////////////////////////////////////////////////////////////
    //Lift samples to factor space
    ////////////////////////////////////////////////////////////////////////////////
    return factor_->lift(statesChild_, state);
}

void RestrictionSampler::sampleUniformNear(base::State *state, const base::State *near, double distance) {
    OMPL_WARN("sampleUniformNear is not using restriction sampling.");
    return baseSampler_->sampleUniformNear(state, near, distance);
}
void RestrictionSampler::sampleGaussian(base::State *state, const base::State *mean, double stdDev) {
    OMPL_WARN("sampleGaussian is not using restriction sampling.");
    return baseSampler_->sampleGaussian(state, mean, stdDev);
}
