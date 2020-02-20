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

#ifndef UTILS_CONVERSION_H_
#define UTILS_CONVERSION_H_

#include <corbo-core/time_series.h>
#include <corbo-core/types.h>
#include <mpc_local_planner/systems/robot_dynamics_interface.h>

#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <string>

namespace mpc_local_planner {

/**
 * @brief Convert TimeSeries to pose array
 *
 * Converts TimeSeries to std::vector<geometry_msgs::PoseStamped>.
 *
 * @remarks Note, the actual data of the TimeSeries object is copied without interpolation.
 *
 * @todo We could avoid the system dynamics dependency by specifying a generic getter function for the SE2 poses
 *
 * @param[in] time_series       corbo::TimeSeries object or any child class
 * @param[in] dynamics          Reference to the robot dynamics interface (to access state-to-SE2 conversion methods)
 * @param[out] poses_stamped    The resulting pose array (note, the incoming vector will be cleared)
 * @param[in] frame_id          The planning frame id that is added to the message header
 */
void convert(const corbo::TimeSeries& time_series, const RobotDynamicsInterface& dynamics, std::vector<geometry_msgs::PoseStamped>& poses_stamped,
             const std::string& frame_id);

}  // namespace mpc_local_planner

#endif  // UTILS_CONVERSION_H_
