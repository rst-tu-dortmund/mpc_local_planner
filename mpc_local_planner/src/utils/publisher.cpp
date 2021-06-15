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

#include <mpc_local_planner/utils/publisher.h>

#include <mpc_local_planner/utils/conversion.h>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>
#include <iomanip>
#include <memory>

namespace mpc_local_planner {

Publisher::Publisher(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame) { initialize(nh, system, map_frame); }

void Publisher::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame)
{
    if (initialized_) RCLCPP_WARN(logger_, "mpc_local_planner: Publisher already initialized. Reinitalizing...");

    nh_ = nh;
    system_    = system;
    map_frame_ = map_frame;

    initialized_ = true;
}

nav2_util::CallbackReturn Publisher::on_configure()
{
  // register topics
    global_plan_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
    local_plan_pub_  = nh_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
    mpc_marker_pub_  = nh_->create_publisher<visualization_msgs::msg::Marker>("mpc_markers", 1000);

  // result publisher:
    _ocp_result_pub = nh_->create_publisher<mpc_local_planner_msgs::msg::OptimalControlResult>("ocp_result", 100);

  initialized_ = true;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Publisher::on_activate()
{
  global_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  mpc_marker_pub_->on_activate();
  _ocp_result_pub->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Publisher::on_deactivate()
{
  global_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  mpc_marker_pub_->on_deactivate();
  _ocp_result_pub->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
Publisher::on_cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  mpc_marker_pub_.reset();
  _ocp_result_pub.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void Publisher::publishLocalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& local_plan) const
{
    if (!initialized_) return;

    nav_msgs::msg::Path mpc_path;
    mpc_path.header.frame_id = local_plan.begin()->header.frame_id;
    mpc_path.header.stamp = nh_->now();
    mpc_path.poses = local_plan;

    local_plan_pub_->publish(mpc_path);
}

void Publisher::publishLocalPlan(const corbo::TimeSeries& ts) const
{
    if (!initialized_) return;
    if (!system_)
    {
        RCLCPP_ERROR(logger_, "Publisher::publishLocalPlan(): cannot publish since the system class is not provided.");
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> local_plan;
    convert(nh_,ts, *system_, local_plan, map_frame_);

    nav_msgs::msg::Path mpc_path;
    mpc_path.header.frame_id = local_plan.begin()->header.frame_id;
    mpc_path.header.stamp = nh_->now();
    mpc_path.poses = local_plan;

    local_plan_pub_->publish(mpc_path);
}

void Publisher::publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan) const
{
    if (!initialized_) return;
    nav_msgs::msg::Path global_path;
    global_path.header.frame_id = global_plan.begin()->header.frame_id;
    global_path.header.stamp = nh_->now();
    global_path.poses = global_plan;

    global_plan_pub_->publish(global_path);
}

void Publisher::publishRobotFootprintModel(const teb_local_planner::PoseSE2& current_pose,
                                           const teb_local_planner::BaseRobotFootprintModel& robot_model, const std::string& ns,
                                           const std_msgs::msg::ColorRGBA& color)
{
    if (!initialized_) return;

    std::vector<visualization_msgs::msg::Marker> markers;
    robot_model.visualizeRobot(current_pose, markers, color);
    if (markers.empty()) return;

    int idx = 1000000;  // avoid overshadowing by obstacles
    for (visualization_msgs::msg::Marker& marker : markers)
    {
        marker.header.frame_id = map_frame_;
        marker.header.stamp    = nh_->now();
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.ns              = ns;
        marker.id              = idx++;
        marker.lifetime        = rclcpp::Duration::from_seconds(2.0);
        mpc_marker_pub_->publish(marker);
    }
}

void Publisher::publishObstacles(const teb_local_planner::ObstContainer& obstacles) const
{
    if (obstacles.empty() || !initialized_) return;

    // Visualize point obstacles
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp    = nh_->now();
        marker.ns              = "PointObstacles";
        marker.id              = 0;
        marker.type            = visualization_msgs::msg::Marker::POINTS;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.lifetime        = rclcpp::Duration::from_seconds(2.0);

        for (const ObstaclePtr& obst : obstacles)
        {
            // std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(obst); // TODO(roesmann): change teb_local_planner
            // types to std lib
            std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(obst);
            if (!pobst) continue;

            geometry_msgs::msg::Point point;
            point.x = pobst->x();
            point.y = pobst->y();
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        mpc_marker_pub_->publish(marker);
    }

    // Visualize line obstacles
    {
        int idx = 0;
        for (const ObstaclePtr& obst : obstacles)
        {
            // LineObstacle::Ptr pobst = std::dynamic_pointer_cast<LineObstacle>(obst);
            std::shared_ptr<LineObstacle> pobst = std::dynamic_pointer_cast<LineObstacle>(obst);
            if (!pobst) continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp    = nh_->now();
            marker.ns              = "LineObstacles";
            marker.id              = idx++;
            marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::msg::Marker::ADD;
            marker.lifetime        = rclcpp::Duration::from_seconds(2.0);
            geometry_msgs::msg::Point start;
            start.x = pobst->start().x();
            start.y = pobst->start().y();
            start.z = 0;
            marker.points.push_back(start);
            geometry_msgs::msg::Point end;
            end.x = pobst->end().x();
            end.y = pobst->end().y();
            end.z = 0;
            marker.points.push_back(end);

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            mpc_marker_pub_->publish(marker);
        }
    }

    // Visualize polygon obstacles
    {
        int idx = 0;
        for (const ObstaclePtr& obst : obstacles)
        {
            // PolygonObstacle::Ptr pobst = std::dynamic_pointer_cast<PolygonObstacle>(obst);
            std::shared_ptr<PolygonObstacle> pobst = std::dynamic_pointer_cast<PolygonObstacle>(obst);
            if (!pobst) continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp    = nh_->now();
            marker.ns              = "PolyObstacles";
            marker.id              = idx++;
            marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action          = visualization_msgs::msg::Marker::ADD;
            marker.lifetime        = rclcpp::Duration::from_seconds(2.0);

            for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
            {
                geometry_msgs::msg::Point point;
                point.x = vertex->x();
                point.y = vertex->y();
                point.z = 0;
                marker.points.push_back(point);
            }

            // Also add last point to close the polygon
            // but only if polygon has more than 2 points (it is not a line)
            if (pobst->vertices().size() > 2)
            {
                geometry_msgs::msg::Point point;
                point.x = pobst->vertices().front().x();
                point.y = pobst->vertices().front().y();
                point.z = 0;
                marker.points.push_back(point);
            }
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            mpc_marker_pub_->publish(marker);
        }
    }
}

void Publisher::publishOptimalControlResult(mpc_local_planner_msgs::msg::OptimalControlResult& msg)
{
    _ocp_result_pub->publish(msg);
}

void Publisher::publishViaPoints(const std::vector<teb_local_planner::PoseSE2>& via_points, const std::string& ns) const
{
    if (via_points.empty() || !initialized_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp    = nh_->now();
    marker.ns              = ns;
    marker.id              = 55555;
    marker.type            = visualization_msgs::msg::Marker::POINTS;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.lifetime        = rclcpp::Duration::from_seconds(2.0);

    for (const teb_local_planner::PoseSE2& via_point : via_points)
    {
        geometry_msgs::msg::Point point;
        point.x = via_point.x();
        point.y = via_point.y();
        point.z = 0;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    mpc_marker_pub_->publish(marker);
}

std_msgs::msg::ColorRGBA Publisher::toColorMsg(float a, float r, float g, float b)
{
    std_msgs::msg::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
}

}  // namespace mpc_local_planner
