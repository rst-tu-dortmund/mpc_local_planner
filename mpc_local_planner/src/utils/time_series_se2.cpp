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

#include <mpc_local_planner/utils/time_series_se2.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <corbo-core/console.h>

#include <cmath>
#include <iomanip>

namespace mpc_local_planner {

bool TimeSeriesSE2::getValuesInterpolate(double time, Eigen::Ref<Eigen::VectorXd> values, Interpolation interpolation, Extrapolation extrapolation,
                                         double tolerance) const
{
    if (_time.empty()) return false;

    auto it = std::find_if(_time.begin(), _time.end(), [time](double val) { return val >= time; });  // find next time interval
    if (it == _time.end())
    {
        switch (extrapolation)
        {
            case Extrapolation::NoExtrapolation:
            {
                break;
            }
            case Extrapolation::ZeroOrderHold:
            {
                values = getValuesMap(getTimeDimension() - 1);
                return true;
            }
            default:
            {
                PRINT_ERROR("TimeSeries::valuesInterpolate(): desired extrapolation method not implemented.");
                return false;
            }
        }
    }
    // interpolate
    // int n     = (int)_time.size();
    int idx = (int)std::distance(_time.begin(), it);
    if (idx >= getTimeDimension()) idx = getTimeDimension() - 1;  // this might occure with floating point comparison
    if (std::abs(time - _time[idx]) < tolerance)
    {
        // we are close to the next value, in particular idx
        values = getValuesMap(idx);
        return true;
    }
    if (idx < 1)
    {
        PRINT_ERROR("accessing a time idx in the past which is not this time series");
    }
    // okay, so now we have the above case val > time -> so idx corresponds to the next sample.
    double dt = time - _time[idx - 1];
    PRINT_ERROR_COND_NAMED(dt < 0, "dt < 0 in interpolation. This cannot be correct.");

    switch (interpolation)
    {
        case Interpolation::ZeroOrderHold:
        {
            if (idx < 1) idx = 1;
            values = getValuesMap(idx - 1);  // since dt > 0 -> we use the previous value
            break;
        }
        case Interpolation::Linear:
        {
            if (idx < 1)
            {
                values = getValuesMap(0);
            }
            else
            {
                double dt_data   = _time[idx] - _time[idx - 1];
                double dt_frac   = dt / dt_data;
                values.noalias() = getValuesMap(idx - 1) + dt_frac * (getValuesMap(idx) - getValuesMap(idx - 1));
                // overwrite third component with is an angle in [-pi, pi)
                if (values.size() > 2)
                {
                    values[2] = interpolate_angle(getValuesMap(idx - 1)[2], getValuesMap(idx)[2], dt_frac);
                }
            }
            break;
        }
        default:
        {
            PRINT_ERROR("TimeSeries::valuesInterpolate(): desired interpolation method not implemented.");
            return false;
        }
    }
    return true;
}

double TimeSeriesSE2::computeMeanOverall()
{
    PRINT_ERROR_NAMED("SE2 version not yet implemented.");
    return getValuesMatrixView().mean();
}

void TimeSeriesSE2::computeMeanCwise(Eigen::Ref<Eigen::VectorXd> mean_values)
{
    PRINT_ERROR_NAMED("SE2 version Not yet implemented.");
    if (mean_values.size() != getValueDimension())
    {
        PRINT_ERROR("TimeSeries::computeMeanCwise(): provided mean_values vector does not match value dimension");
        return;
    }
    mean_values = getValuesMatrixView().rowwise().mean();
}

}  // namespace mpc_local_planner
