#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

using namespace ompl::multilevel;

ompl::multilevel::FactoredSpaceInformation::FactoredSpaceInformation(const ompl::base::StateSpacePtr& space) : 
  ompl::base::SpaceInformation(space) {
}

std::string FactoredSpaceInformation::getName() const {
  return getStateSpace()->getName();
}

const FactoredSpaceInformationPtr& FactoredSpaceInformation::getParent() const {
  return parent_;
}

bool FactoredSpaceInformation::hasParent() const {
  return parent_ != nullptr;
}

const std::vector<FactoredSpaceInformationPtr>& FactoredSpaceInformation::getChildren() const {
  return children_;
}

bool FactoredSpaceInformation::hasChildren() const {
  return !children_.empty();
}

const ProjectionPtr& FactoredSpaceInformation::getProjection() const {
  return projection_to_parent_;
}

void FactoredSpaceInformation::setParent(FactoredSpaceInformationPtr factor_si) {
  parent_ = factor_si;
}

std::vector<FactoredSpaceInformationPtr> FactoredSpaceInformation::getAllFactors() {
  std::vector<FactoredSpaceInformationPtr> factors_flatten;

  factors_flatten.push_back(shared_from_this());

  if(hasChildren()) {
    for(const auto& child : children_ ) {
      const auto next = child->getAllFactors();
      factors_flatten.insert(factors_flatten.end(), next.begin(), next.end());
    }
  }
  return factors_flatten;
}

std::vector<FactoredSpaceInformationPtr> FactoredSpaceInformation::getLeafFactors() {
  std::vector<FactoredSpaceInformationPtr> leaf_factors;
  for(const auto& factor : getAllFactors()) {
    if(!factor->hasChildren()) {
      leaf_factors.push_back(factor);
    }
  }
  return leaf_factors;
}

bool FactoredSpaceInformation::addChild(FactoredSpaceInformationPtr factor, ProjectionPtr projection) {
  if(getName() == factor->getName()) {
    OMPL_ERROR("Cannot add the same factor as child for factor %s.", getName().c_str());
    return false;
  }
  factor->setProjectionToParent(projection);
  factor->setParent(shared_from_this());
  for(const auto& child : children_) {
    if(child->getName() == factor->getName()) {
      OMPL_ERROR("Child with name %s already exists.", child->getName().c_str());
      return false;
    }
  }
  children_.push_back(factor);
  if(projection->isFibered()) {
    std::static_pointer_cast<FiberedProjection>(projection)->makeFiberSpace();
  }
  return true;
}

void FactoredSpaceInformation::setProjectionToParent(ProjectionPtr projection) {
  projection_to_parent_ = projection;
}

void FactoredSpaceInformation::setup() {
  SpaceInformation::setup();
}

const FactoredSpaceInformationPtr& FactoredSpaceInformation::getChild(const std::string& name) const {
  auto iterator = std::find_if(children_.begin(), children_.end(), 
        [name](const auto& child) {
          return child->getName() == name;
        });
  if(iterator == children_.end()) {
    OMPL_ERROR("No child with name %s", name.c_str());
    return nullptr;
  }
  return *iterator;
}

void FactoredSpaceInformation::lift(const std::unordered_map<std::string, base::State*>& childStates_, base::State* state) const {
  OMPL_ERROR("Need two lifts: Either to implicitly sample the fiber space, or to stitch multiple projections together");
  for(const auto& name_and_state: childStates_) {
    const auto& name = name_and_state.first;
    const auto& factorState = name_and_state.second;
    const auto& factor = getChild(name);
    const auto& projection = factor->getProjection();
    projection->lift(factorState, state);
  }
}

void FactoredSpaceInformation::printSettings(std::ostream &out) const
{
    SpaceInformation::printSettings(out);
    out << "Factorization of " << getName() << " has ";
    if(hasChildren()) {
      const auto& children = getChildren();
      const auto N = children.size();
      out << N << (N > 1 ? " children" : " child") << " (";
      for (auto iter = children.begin(); iter != children.end(); iter++) {
        if (iter != children.begin()) out << " | ";
        out << (*iter)->getName();
      }
      out << ")";
    } else {
      out << "no children";
    }

    out << " and ";

    if(hasParent()) {
      out << "1 parent (" << getParent()->getName() << ")";
    } else {
      out << "no parents";
    }
    out << "." << std::endl;
}
