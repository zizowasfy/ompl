/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <boost/iterator/counting_iterator.hpp>

using namespace ompl::multilevel;

FiberedProjection::FiberedProjection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : Projection(bundleSpace, baseSpace)
{
}

void FiberedProjection::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    fiberSpaceSampler_->sampleUniform(xFiberTmp_);
    lift(xBase, xFiberTmp_, xBundle);
}

ompl::base::StateSpacePtr FiberedProjection::getFiberSpace() const
{
    return fiberSpace_;
}

bool FiberedProjection::isFibered() const
{
    return true;
}

std::vector<size_t> FiberedProjection::getInclusionIndices() const 
{
    return std::vector<size_t>(boost::counting_iterator<size_t>(0), boost::counting_iterator<size_t>(getBaseDimension()));
}

void FiberedProjection::inclusionMap(const ompl::base::State *xBase, ompl::base::State *xBundle) const 
{
  std::vector<size_t> indices = getInclusionIndices();
  for(const auto& index : indices) 
  {
    double* value = getBundle()->getValueAddressAtIndex(xBundle, index);
    const double* baseValue = getBase()->getValueAddressAtIndex(xBase, index);
    *value = *baseValue;
  }
}

unsigned int FiberedProjection::getFiberDimension() const
{
    if (fiberSpace_)
        return fiberSpace_->getDimension();
    else
        return 0;
}

std::string FiberedProjection::getFiberTypeAsString() const
{
    if (fiberSpace_)
        return stateTypeToString(fiberSpace_);
    else
        return "None";
}

void FiberedProjection::makeFiberSpace()
{
    fiberSpace_ = computeFiberSpace();

    if (fiberSpace_ != nullptr)
    {
        siFiberSpace_ = std::make_shared<ompl::base::SpaceInformation>(fiberSpace_);
        fiberSpaceSampler_ = siFiberSpace_->allocStateSampler();
        xFiberTmp_ = siFiberSpace_->allocState();
    }
}

ompl::base::StateSamplerPtr FiberedProjection::getFiberSamplerPtr() const
{
    return fiberSpaceSampler_;
}
