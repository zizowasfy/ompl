#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

using namespace ompl::multilevel;

ompl::multilevel::FactoredSpaceInformation::FactoredSpaceInformation(const ompl::base::StateSpacePtr& space) : 
  ompl::base::SpaceInformation(space) 
{
  OMPL_INFORM("Create factor for space %s (dimensionality %d)", space->getName().c_str(), space->getDimension());
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

bool FactoredSpaceInformation::isEquivalentTo(const FactoredSpaceInformationPtr& rhs) const 
{
    return getName() == rhs->getName();
}

bool FactoredSpaceInformation::projectionHasValidIndices(const FactoredSpaceInformationPtr& factor, const ProjectionPtr& projection) const 
{
  if(!projection->isFibered()) {
    return true;
  }
  const auto& fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
  const std::vector<size_t> indices = fibered_projection->getInclusionIndices();
  const auto N = projection->getDimension();
  for(const auto& index : indices) 
  {
    if(index >= N) {
      OMPL_ERROR("Index %d / %d is out of bounds for factor space %s.", index, N, factor->getName().c_str());
      return false;
    }
  }
  return true;
}

bool FactoredSpaceInformation::projectionOverlapsWithExistingProjections(const FactoredSpaceInformationPtr& factor, const ProjectionPtr& projection) const 
{
  if(!projection->isFibered()) {
    return false;
  }
  const auto& fibered_projection = std::static_pointer_cast<FiberedProjection>(projection);
  std::vector<size_t> indices = fibered_projection->getInclusionIndices();
  std::sort(indices.begin(), indices.end());

  auto hasIndexIntersection = [factor, indices](const auto& child) {
        const auto& child_projection = child->getProjection();
        if(!child_projection->isFibered()) 
        {
          return false;
        }
        const auto& fibered_child_projection = std::static_pointer_cast<FiberedProjection>(child_projection);
        std::vector<size_t> child_indices = fibered_child_projection->getInclusionIndices();
        std::sort(child_indices.begin(), child_indices.end());

        std::vector<size_t> intersected_indices;
        std::set_intersection(indices.begin(), indices.end(),
                              child_indices.begin(), child_indices.end(),
                              back_inserter(intersected_indices));

        if(!intersected_indices.empty()) 
        {
          std::string error_msg = "Found overlap from factor " + factor->getName() 
            + " to factor " + child->getName() + " at: \n";
          for(const auto& index : intersected_indices) 
          {
            error_msg += " - Index " + std::to_string(index) + "\n";
          }
          error_msg += " Note: Indices from factor " + factor->getName() + " are ";
          for(const auto& index : indices) 
          {
            error_msg += std::to_string(index) + " ";
          }
          error_msg += "\n Note: Indices from child " + child->getName() + " are ";
          for(const auto& index : child_indices) 
          {
            error_msg += std::to_string(index) + " ";
          }

          OMPL_ERROR("%s", error_msg.c_str());
          return true;
        }
        return false;//!intersected_indices.empty();
      };

  return std::any_of(children_.begin(), children_.end(), hasIndexIntersection);
}

bool FactoredSpaceInformation::childExists(const FactoredSpaceInformationPtr& factor) const {
  return std::any_of(children_.begin(), children_.end(),
      [factor](const auto& child) {
        return child->isEquivalentTo(factor);
      });
}

bool FactoredSpaceInformation::projectionHasCorrectImage(const FactoredSpaceInformationPtr& child, const ProjectionPtr& projection) const
{
    if(projection->getBaseDimension() != child->getStateDimension())
    {
      return false;
    }

    return projection->getBase()->getName() == child->getName();
}

bool FactoredSpaceInformation::projectionHasCorrectPreimage(const ProjectionPtr& projection) const
{
    if(projection->getDimension() != this->getStateDimension())
    {
      return false;
    }

    return projection->getBundle()->getName() == this->getName();
}

bool FactoredSpaceInformation::addChild(FactoredSpaceInformationPtr child, ProjectionPtr projection) {

  if(this->isEquivalentTo(child)) 
  {
    OMPL_ERROR("Cannot add the same factor as child for factor %s.", getName().c_str());
    return false;
  }

  if(childExists(child)) 
  {
      OMPL_ERROR("Child with name %s already exists. Please choose unique names for each StateSpace.", child->getName().c_str());
      return false;
  }

  if(!projectionHasCorrectPreimage(projection))
  {
      OMPL_ERROR("Projection for child %s does not have correct preimage.", child->getName().c_str());
      return false;
  }

  if(!projectionHasCorrectImage(child, projection))
  {
      OMPL_ERROR("Projection for child %s does not have correct image.", child->getName().c_str());
      return false;
  }

  if(!projectionHasValidIndices(child, projection)) 
  {
      OMPL_ERROR("Projection for child %s has invalid indices.", child->getName().c_str());
      return false;
  }

  if(projectionOverlapsWithExistingProjections(child, projection)) 
  {
      OMPL_ERROR("Projection for child %s overlaps with existing projection.", child->getName().c_str());
      return false;
  }

  child->setProjectionToParent(projection);
  child->setParent(shared_from_this());
  children_.push_back(child);

  if(projection->isFibered()) {
    OMPL_INFORM("Create fiber space for projection from %s to %s", getName().c_str(), child->getName().c_str());
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
    throw "NoChildError";
  }
  return *iterator;
}

void FactoredSpaceInformation::lift(const std::unordered_map<std::string, base::State*>& childStates_, base::State* state) const {
  if(childStates_.size() <= 1) {
    const auto& name = childStates_.begin()->first;
    const auto& childState = childStates_.begin()->second;
    const auto& child = getChild(name);
    const auto& projection = child->getProjection();
    projection->lift(childState, state);
    return;
  }

  OMPL_INFORM("Lifting state");
  for(const auto& name_and_state: childStates_) {
    const auto& name = name_and_state.first;
    const auto& childState = name_and_state.second;
    const auto& child = getChild(name);
    const auto& projection = std::static_pointer_cast<FiberedProjection>(child->getProjection());
    child->printState(childState);
    projection->inclusionMap(childState, state);
  }
  printState(state);
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
