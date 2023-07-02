#include "ompl/multilevel/planners/factor/FactorRRT.h"

#include <ompl/base/StateSpace.h>
#include "ompl/multilevel/datastructures/FactoredSpaceInformation.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <ompl/multilevel/planners/factor/FactoredPlanner.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

using namespace ompl::multilevel;

FactorRRT::FactorRRT(const FactoredSpaceInformationPtr &si) :
   ompl::base::Planner(si, "FactorRRT") 
{
  specs_.approximateSolutions = false;
  specs_.directed = true;
}

FactorRRT::~FactorRRT() {
}

const FactoredSpaceInformationPtr& FactorRRT::getFactoredSpaceInformation() const {
  return std::static_pointer_cast<FactoredSpaceInformation>(si_);
}

const std::unordered_map<std::string, ompl::base::ProblemDefinitionPtr>& FactorRRT::getProblemDefinitions() const {
  return problem_definitions_per_factor_;
}

const std::unordered_map<std::string, ompl::base::PlannerStatus>& FactorRRT::getPlannerStatus() const {
  return planner_status_per_factor_;
}

bool FactorRRT::allChildrenHaveSolutions_(const FactoredSpaceInformationPtr& factor) const {
  if(!factor->hasChildren()) {
    return true;
  }
  const auto& children = factor->getChildren();

  //Debug
  OMPL_INFORM("Factor %s has %d children.", factor->getName().c_str(), children.size());
  for(const auto& child : children) {
    OMPL_INFORM(" -- %s has %s.", child->getName().c_str(), (hasSolution_(child) ? "a solution" : "no solution"));
  }

  for(const auto& child : children) {
    if(!hasSolution_(child)) {
      return false;
    }
  }
  return true;
}

void FactorRRT::setSeed(size_t seed) {
  rng_.setSeed(seed);
}

const FactoredSpaceInformationPtr& FactorRRT::selectFactor_() {
  int index = rng_.uniformInt(0, active_factors_.size() - 1);
  return active_factors_.at(index);
}

void FactorRRT::clear() {
  Planner::clear();
}

void FactorRRT::setup() {
  Planner::setup();
}

bool FactorRRT::hasSolution_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = planner_status_per_factor_.find(factor->getName());
  if(iterator == planner_status_per_factor_.end()) {
    return false;
  }
  return planner_status_per_factor_.at(factor->getName()); //using bool operator
}

bool FactorRRT::isActive_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = is_active_.find(factor->getName());
  if(iterator == is_active_.end()) {
    return false;
  }
  return is_active_.at(factor->getName());
}

bool FactorRRT::isSolved_(const FactoredSpaceInformationPtr& factor) const {
  auto iterator = is_solved_.find(factor->getName());
  if(iterator == is_solved_.end()) {
    return false;
  }
  return is_solved_.at(factor->getName());
}

void FactorRRT::grow_(const FactoredSpaceInformationPtr& factor) {
  OMPL_INFORM("Growing factor %s (%d active factors)", factor->getName().c_str(), active_factors_.size());

  auto iterator = active_planners_.find(factor->getName());
  if(iterator == active_planners_.end()) {
    createPlannerForFactor_(factor);
  }
  auto& planner = active_planners_[factor->getName()];
  ompl::base::IterationTerminationCondition itc(1);
  planner_status_per_factor_.insert({factor->getName(), planner->solve(itc)});
}

std::vector<FactoredPlannerPtr> FactorRRT::getChildrenPlanner_(const FactoredSpaceInformationPtr& factor) const {
  std::vector<FactoredPlannerPtr> children_planner;
  if(!factor->hasChildren()) {
    return children_planner;
  }

  const auto& children = factor->getChildren();
  for(const auto& child : children) {
    child->getName();
    auto iterator = active_planners_.find(child->getName());
    if(iterator == active_planners_.end()) {
      OMPL_ERROR("Could not find a planner child with name %s for factor %s", child->getName().c_str(), factor->getName().c_str());
      throw "NotAPlanner";
    }
    children_planner.push_back(iterator->second);
  }
  return children_planner;
}

void FactorRRT::createPlannerForFactor_(const FactoredSpaceInformationPtr& factor) {
  const auto& name = factor->getName();

  if(!factor->hasChildren()) {
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor);
  } else {
    auto children_planner = getChildrenPlanner_(factor);
    active_planners_[name] = std::make_shared<FactoredPlanner>(factor, children_planner);
  }
  OMPL_INFORM("Created new planner %s for factor %s", active_planners_[name]->getName().c_str(), name.c_str());

  auto iterator = problem_definitions_per_factor_.find(factor->getName());
  if(iterator == problem_definitions_per_factor_.end()) {
    OMPL_ERROR("Could not get problem definition for factor %s", name.c_str());
    return;
  }
  active_planners_[name]->setProblemDefinition(iterator->second);
}

void FactorRRT::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
  Planner::setProblemDefinition(pdef);

  const auto& root = getFactoredSpaceInformation();

  problem_definitions_per_factor_[root->getName()] = pdef;
  if(!root->hasChildren()) {
    return;
  }

  if(pdef->getStartStateCount() != 1) {
    OMPL_ERROR("FactorRRT can handle only a single start state, but you have %d states.", pdef->getStartStateCount());
    return;
  }
  const auto& goal_region = pdef->getGoal();
  const auto& type = goal_region->getType();

  //NOTE: Any sampleable goal_region is possible, because we can always generate goals
  //on the highest level and then project them downwards in the factor tree.
  //However, this has not been implemented here
  if(type != base::GoalType::GOAL_STATE) {
    OMPL_ERROR("FactorRRT can handle only a goal_region with a single goal state.");
    return;
  }

  const base::State* goal = goal_region->as<base::GoalState>()->getState();
  const base::State* start = pdef->getStartState(0);

  std::stringstream ss_start, ss_goal;
  root->printState(start, ss_start);
  root->printState(goal, ss_goal);
  OMPL_INFORM("Project states \n Start %s Goal %s.", ss_start.str().c_str(), ss_goal.str().c_str());

  for(const auto& child : root->getChildren()) {
    createProblemDefinition_(child, start, goal);
  }
}

void FactorRRT::createProblemDefinition_(const FactoredSpaceInformationPtr& factor, const base::State* parent_start, const base::State* parent_goal) {
  base::ProblemDefinitionPtr pdef = std::make_shared<base::ProblemDefinition>(factor);

  base::State* start = factor->allocState();
  base::State* goal = factor->allocState();

  const auto& projection = factor->getProjection();

  projection->project(parent_start, start);
  projection->project(parent_goal, goal);

  std::stringstream ss_start, ss_goal;
  factor->printState(start, ss_start);
  factor->printState(goal, ss_goal);
  OMPL_INFORM("Project states onto factor %s \n Start %s Goal %s", 
      factor->getName().c_str(), ss_start.str().c_str(), ss_goal.str().c_str());

  pdef->addStartState(start);
  pdef->setGoalState(goal);

  problem_definitions_per_factor_[factor->getName()] = pdef;

  for(const auto& child : factor->getChildren()) {
    createProblemDefinition_(child, start, goal);
  }
}

ompl::base::PlannerStatus FactorRRT::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    ////////////////////////////////////////////////////////////////////////////////
    active_factors_ = getFactoredSpaceInformation()->getLeafFactors();
    for(const auto& factor: active_factors_) {
      is_active_.insert({factor->getName(), true});
      is_solved_.insert({factor->getName(), false});
    }
    OMPL_DEBUG("Solving FactoredSpaceInformation using %d active factors.", active_factors_.size());
    
    if(active_factors_.empty()) {
      OMPL_ERROR("Could not find any leaf nodes for factor %s", getFactoredSpaceInformation()->getName().c_str());
      return base::PlannerStatus(base::PlannerStatus::StatusType::ABORT);
    }

    planner_status_ = base::PlannerStatus(base::PlannerStatus::StatusType::TIMEOUT);
    size_t counter = 0;
    while (!ptc) {

        OMPL_DEBUG("Iteration %d", counter++);

        const auto& selectedFactor = selectFactor_();

        grow_(selectedFactor);

        if(hasSolution_(selectedFactor)) {

          if(!isSolved_(selectedFactor)) {
            is_solved_[selectedFactor->getName()] = true;
            OMPL_DEBUG(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
            OMPL_DEBUG(" >>> Solved factor %s.", selectedFactor->getName().c_str());
            OMPL_DEBUG(" >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ");
            if(!selectedFactor->hasParent()) {
              const auto& name = selectedFactor->getName();
              OMPL_DEBUG(" >>>>>> Found solution on root factor %s", name.c_str());
              planner_status_ = base::PlannerStatus::StatusType::EXACT_SOLUTION;
              continue;
            }
          }

          if(!selectedFactor->hasParent()) {
            continue;
          }

          const auto& parent = selectedFactor->getParent();
          if(!isActive_(parent) && allChildrenHaveSolutions_(parent)) {
            OMPL_DEBUG("Add factor %s to active spaces.", parent->getName().c_str());
            active_factors_.push_back(parent);
            is_active_.insert({parent->getName(), true});
            is_solved_.insert({parent->getName(), false});
          }
        }
    }
    return planner_status_;
}
