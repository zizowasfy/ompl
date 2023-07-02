#ifndef OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
#define OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"

#include <optional>

namespace ompl {
    namespace multilevel {

        OMPL_CLASS_FORWARD(FactoredPlanner);
        OMPL_CLASS_FORWARD(FactoredSpaceInformation);

        class FactoredPlanner : public ompl::geometric::RRTConnect {
          public:
            /** \brief Constructor */
            FactoredPlanner(const FactoredSpaceInformationPtr& si, const std::vector<FactoredPlannerPtr>& children_planner = {});

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            const FactoredSpaceInformationPtr& getFactoredSpaceInformation() const;

            void sampleFromDatastructure(ompl::base::State* state);
        };
    }
}

#endif // OMPL_MULTILEVEL_PLANNERS_FACTOR_FACTOREDPLANNER_
