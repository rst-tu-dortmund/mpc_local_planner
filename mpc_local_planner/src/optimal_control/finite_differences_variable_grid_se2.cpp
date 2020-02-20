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

#include <mpc_local_planner/optimal_control/finite_differences_variable_grid_se2.h>

#include <corbo-optimal-control/structured_ocp/edges/finite_differences_collocation_edges.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace mpc_local_planner {

void FiniteDifferencesVariableGridSE2::setDtBounds(double dt_lb, double dt_ub)
{
    _dt_lb = dt_lb;
    _dt_ub = dt_ub;
}

void FiniteDifferencesVariableGridSE2::setGridAdaptTimeBasedSingleStep(int n_max, double dt_hyst_ratio, bool adapt_first_iter)
{
    _grid_adapt       = GridAdaptStrategy::TimeBasedSingleStep;
    _n_max            = n_max;
    _dt_hyst_ratio    = dt_hyst_ratio;
    _adapt_first_iter = adapt_first_iter;
}

void FiniteDifferencesVariableGridSE2::setGridAdaptTimeBasedAggressiveEstimate(int n_max, double dt_hyst_ratio, bool adapt_first_iter)
{
    _grid_adapt       = GridAdaptStrategy::TimeBasedAggressiveEstimate;
    _n_max            = n_max;
    _dt_hyst_ratio    = dt_hyst_ratio;
    _adapt_first_iter = adapt_first_iter;
}

void FiniteDifferencesVariableGridSE2::setGridAdaptSimpleShrinkingHorizon(bool adapt_first_iter)
{
    _grid_adapt       = GridAdaptStrategy::SimpleShrinkingHorizon;
    _adapt_first_iter = adapt_first_iter;
}

bool FiniteDifferencesVariableGridSE2::adaptGrid(bool new_run, NlpFunctions& nlp_fun)
{
    // do not adapt grid in a new run
    if (new_run && !_adapt_first_iter) return false;

    bool changed = false;
    switch (_grid_adapt)
    {
        case GridAdaptStrategy::NoGridAdapt:
        {
            break;
        }
        case GridAdaptStrategy::TimeBasedSingleStep:
        {
            changed = adaptGridTimeBasedSingleStep(nlp_fun);
            break;
        }
        case GridAdaptStrategy::TimeBasedAggressiveEstimate:
        {
            changed = adaptGridTimeBasedAggressiveEstimate(nlp_fun);
            break;
        }
        case GridAdaptStrategy::SimpleShrinkingHorizon:
        {
            changed = adaptGridSimpleShrinkingHorizon(nlp_fun);
            break;
        }
        default:
        {
            PRINT_ERROR_NAMED("selected grid adaptation strategy not implemented.");
        }
    }
    return changed;
}

bool FiniteDifferencesVariableGridSE2::adaptGridTimeBasedSingleStep(NlpFunctions& nlp_fun)
{
    PRINT_WARNING_COND_NAMED(!isTimeVariableGrid(), "time based adaptation might only be used with a fixed dt.");

    _nlp_fun = &nlp_fun;

    int n = getN();

    double dt = getDt();
    if (dt > _dt_ref * (1.0 + _dt_hyst_ratio) && n < _n_max)
    {
        resampleTrajectory(n + 1);
        _n_adapt = n + 1;
        return true;
    }
    else if (dt < _dt_ref * (1.0 - _dt_hyst_ratio) && n > _n_min)
    {
        resampleTrajectory(n - 1);
        _n_adapt = n - 1;
        return true;
    }
    return false;
}

bool FiniteDifferencesVariableGridSE2::adaptGridTimeBasedAggressiveEstimate(NlpFunctions& nlp_fun)
{
    PRINT_WARNING_COND_NAMED(!isTimeVariableGrid(), "time based adaptation might only be used with a fixed dt.");

    _nlp_fun  = &nlp_fun;
    int n     = getN();
    double dt = getDt();

    // check if hysteresis is satisfied
    if (dt >= _dt_ref * (1.0 - _dt_hyst_ratio) && dt <= _dt_ref * (1.0 + _dt_hyst_ratio)) return false;

    // estimate number of samples based on the fraction dt/dt_ref.
    // dt is the time difference obtained in a previous solution (with a coarser resp. finer trajectory resolution)
    int new_n = std::round((double)n * (dt / _dt_ref));

    // bound value
    if (new_n > _n_max)
        new_n = _n_max;
    else if (new_n < _n_min)
        new_n = _n_min;

    if (new_n == n) return false;

    // and now resample
    resampleTrajectory(new_n);
    _n_adapt = new_n;
    return true;
}

bool FiniteDifferencesVariableGridSE2::adaptGridSimpleShrinkingHorizon(NlpFunctions& nlp_fun)
{
    int n = getN();
    if (n > _n_min)
    {
        resampleTrajectory(n - 1);
        _n_adapt = n - 1;
    }
    return false;
}

}  // namespace mpc_local_planner
