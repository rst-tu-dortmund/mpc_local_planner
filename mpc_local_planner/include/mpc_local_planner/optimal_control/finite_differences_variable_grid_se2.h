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

#ifndef FINITE_DIFFERENCES_VARIABLE_GRID_H_
#define FINITE_DIFFERENCES_VARIABLE_GRID_H_

#include <mpc_local_planner/optimal_control/finite_differences_grid_se2.h>

#include <memory>

namespace mpc_local_planner {

/**
 * @brief Finite differences grid with variable resolution for SE2
 *
 * This class implements a full discretization grid with finite difference collocation.
 * The temporal resolution is free.
 * This grid corresponds to the global uniform grid in time-optimal MPC as defined in [1,2].
 *
 * [1] C. Rösmann, F. Hoffmann und T. Bertram: Timed-Elastic-Bands for Time-Optimal Point-to-Point Nonlinear Model Predictive Control, ECC 2015.
 * [2] C. Rösmann: Time-optimal nonlinear model predictive control - Direct transcription methods with variable discretization and structural sparsity
 *     exploitation. Dissertation, TU Dortmund University, Oct. 2019.
 *
 * @see FiniteDifferencesGridSE2
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FiniteDifferencesVariableGridSE2 : public FiniteDifferencesGridSE2
{
 public:
    using Ptr  = std::shared_ptr<FiniteDifferencesVariableGridSE2>;
    using UPtr = std::unique_ptr<FiniteDifferencesVariableGridSE2>;

    enum class GridAdaptStrategy { NoGridAdapt, TimeBasedSingleStep, TimeBasedAggressiveEstimate, SimpleShrinkingHorizon };

    FiniteDifferencesVariableGridSE2()          = default;
    virtual ~FiniteDifferencesVariableGridSE2() = default;

    //! Return a newly created shared instance of the implemented class
    corbo::DiscretizationGridInterface::Ptr getInstance() const override { return std::make_shared<FiniteDifferencesVariableGridSE2>(); }

    //! Get access to the associated factory
    static corbo::Factory<corbo::DiscretizationGridInterface>& getFactory() { return corbo::Factory<corbo::DiscretizationGridInterface>::instance(); }

    //! Set lower and upper bounds for the temporal resolution
    void setDtBounds(double dt_lb, double dt_ub);
    //! Set minium grid size for grid adaptation
    void setNmin(int n_min) { _n_min = n_min; }
    //! Disable grid adapation
    void disableGridAdaptation() { _grid_adapt = GridAdaptStrategy::NoGridAdapt; }
    //! Set grid adaptation strategy to 'single step'
    void setGridAdaptTimeBasedSingleStep(int n_max, double dt_hyst_ratio = 0.1, bool adapt_first_iter = false);
    //! Set grid adaptation strategy to 'aggressive'
    void setGridAdaptTimeBasedAggressiveEstimate(int n_max, double dt_hyst_ratio = 0.1, bool adapt_first_iter = false);
    //! Set grid adaptation strategy to 'shrinking horizon'
    void setGridAdaptSimpleShrinkingHorizon(bool adapt_first_iter = false);

 protected:
    bool isDtFixedIntended() const override { return false; }

    bool adaptGrid(bool new_run, NlpFunctions& nlp_fun) override;
    bool adaptGridTimeBasedSingleStep(NlpFunctions& nlp_fun);
    bool adaptGridTimeBasedAggressiveEstimate(NlpFunctions& nlp_fun);
    bool adaptGridSimpleShrinkingHorizon(NlpFunctions& nlp_fun);

    bool isMovingHorizonWarmStartActive() const override { return false; }
    bool isGridAdaptActive() const override { return true; }

    // auto resize stuff
    GridAdaptStrategy _grid_adapt = GridAdaptStrategy::NoGridAdapt;
    int _n_max                    = 1000;
    double _dt_hyst_ratio         = 0.1;
    int _n_min                    = 2;
    bool _adapt_first_iter        = false;
};

}  // namespace mpc_local_planner

#endif  // FINITE_DIFFERENCES_VARIABLE_GRID_H_
