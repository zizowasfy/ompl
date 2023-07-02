#include "ompl/multilevel/planners/factor/FactoredPlanner.h"
#include "ompl/multilevel/planners/factor/RestrictionSampler.h"
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"

using namespace ompl::multilevel;

FactoredPlanner::FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner) 
  : RRTConnect(si)
{
  setName("PlannerOn" + si->getName());
  if(!children_planner.empty()) {
    sampler_ = std::make_shared<RestrictionSampler>(si, children_planner);
  }
}

ompl::base::PlannerStatus FactoredPlanner::solve(const ompl::base::PlannerTerminationCondition &ptc) {
  return RRTConnect::solve(ptc);
}

const FactoredSpaceInformationPtr& FactoredPlanner::getFactoredSpaceInformation() const {
  return std::static_pointer_cast<FactoredSpaceInformation>(si_);
}

void FactoredPlanner::sampleFromDatastructure(ompl::base::State* state) {
    return sampler_->sampleUniform(state);
}
