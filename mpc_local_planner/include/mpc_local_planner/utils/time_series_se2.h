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

#ifndef UTILS_TIME_SERIES_SE2_H_
#define UTILS_TIME_SERIES_SE2_H_

#include <corbo-core/time_series.h>

#include <Eigen/Core>

namespace mpc_local_planner {

/**
 * @brief Time Series with SE2 support
 *
 * A time series represents a sequence of vectors of floating point numbers
 * associated with time stamps. A time series might also be interpreted as a
 * discrete-time trajectory.
 * This class extends corbo::TimeSeries to support SE2 poses.
 * Hereby, the third state component is always treated as orientation in [-pi, pi).
 * In particular, whenever values are retrieved by interpolation
 * (getValuesInterpolate()), the third state is ensured to remain properly in [-pi, pi).
 *
 * Note, as the first three state components define the SE2, any further auxiliary
 * state component is treated as ordinary real number.
 *
 * @see corbo::TimeSeries
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class TimeSeriesSE2 : public corbo::TimeSeries
{
 public:
    using Ptr      = std::shared_ptr<TimeSeriesSE2>;
    using ConstPtr = std::shared_ptr<const TimeSeriesSE2>;

    //! Default constructor
    TimeSeriesSE2() = default;
    //! Construct empty time series with a dresired value vector dimension
    explicit TimeSeriesSE2(int value_dim) : TimeSeries(value_dim) {}

    /**
     * @brief Retrieve value vector at a desired time stamp (seconds) via interpolation
     *
     * This method interpolates the underlying time series object at a desired
     * time stamp (given in seconds). Interpolation and extrapolation settings
     * can be passed as optional arguments.
     *
     * @warning This method currently assumes that the values / time pairs are
     *          stored with monotonically increasing time stamps (in time()).
     *
     * @param[in]  time              desired time stamp
     * @param[out] values            interpolated value vector w.r.t. \c time
     * @param[in]  interpolation     interpolation method according to Interpolation enum
     * @param[in]  extrapolate       specify whether exrapliation should be applied with a similar
     *                               method like interpolation
     * @param[in]  tolerance         specify a tolerance for which two time stamps are treated equally
     * @returns true if successfull, false otherwise (e.g. if extrapolate is false but \c time > max(time)
     */
    bool getValuesInterpolate(double time, Eigen::Ref<Eigen::VectorXd> values, Interpolation interpolation = Interpolation::Linear,
                              Extrapolation extrapolate = Extrapolation::NoExtrapolation, double tolerance = 1e-6) const override;

    //! Compute and return the mean value of all values among all dimensions
    double computeMeanOverall() override;
    /**
     * @brief Compute and return the component-wise mean values
     * @param[out]  mean_values     Resulting cwise mean values [getValueDimension() x 1] (vector size must be preallocated)
     */
    void computeMeanCwise(Eigen::Ref<Eigen::VectorXd> mean_values) override;
};

}  // namespace mpc_local_planner

#endif  // UTILS_TIME_SERIES_SE2_H_
