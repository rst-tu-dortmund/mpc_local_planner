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

#include <mpc_local_planner/utils/conversion.h>

#include <tf2/transform_datatypes.h>

namespace mpc_local_planner {


auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void convert(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const corbo::TimeSeries& time_series,
             const RobotDynamicsInterface& dynamics, std::vector<geometry_msgs::msg::PoseStamped>& poses_stamped,
             const std::string& frame_id)
{
    poses_stamped.clear();

    if (time_series.isEmpty()) return;

    for (int i = 0; i < time_series.getTimeDimension(); ++i)
    {
        poses_stamped.emplace_back();

        double theta = 0;
        dynamics.getPoseSE2FromState(time_series.getValuesMap(i), poses_stamped.back().pose.position.x, poses_stamped.back().pose.position.y, theta);
        poses_stamped.back().pose.orientation = createQuaternionMsgFromYaw(theta);
        poses_stamped.back().header.frame_id  = frame_id;
        poses_stamped.back().header.stamp     = nh->now();
    }
}

}  // namespace mpc_local_planner
