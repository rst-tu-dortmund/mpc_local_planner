/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#ifndef FINITE_DIFFERENCES_GRID_H_
#define FINITE_DIFFERENCES_GRID_H_

#include <mpc_local_planner/optimal_control/full_discretization_grid_base_se2.h>

#include <memory>

namespace mpc_local_planner {

/**
 * @brief Finite differences grid for SE2
 *
 * This class implements a full discretization grid with finite difference collocation.
 * The temporal resolution is fixed.
 *
 * @see corbo::FullDiscretizationGridBase corbo::DiscretizationGridInterface
 *      FiniteDifferencesGridSE2
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FiniteDifferencesGridSE2 : public FullDiscretizationGridBaseSE2
{
 public:
    using Ptr = std::shared_ptr<FiniteDifferencesGridSE2>;

    using BaseEdge      = corbo::BaseEdge;
    using BaseMixedEdge = corbo::BaseMixedEdge;

    //! Default constructor
    FiniteDifferencesGridSE2() = default;
    //! Default destructor
    virtual ~FiniteDifferencesGridSE2() = default;

    //! Return a newly created shared instance of the implemented class
    corbo::DiscretizationGridInterface::Ptr getInstance() const override { return std::make_shared<FiniteDifferencesGridSE2>(); }

    //! Get access to the associated factory
    static corbo::Factory<corbo::DiscretizationGridInterface>& getFactory() { return corbo::Factory<corbo::DiscretizationGridInterface>::instance(); }

 protected:
    void createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics) override;

    bool isDtFixedIntended() const override { return true; }
};

}  // namespace mpc_local_planner

#endif  // FINITE_DIFFERENCES_GRID_H_
