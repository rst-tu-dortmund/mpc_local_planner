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

#ifndef UTILS_PUBLISHER_H_
#define UTILS_PUBLISHER_H_

#include <corbo-core/time_series.h>
#include <corbo-core/types.h>
#include <mpc_local_planner/systems/robot_dynamics_interface.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <memory>

namespace mpc_local_planner {

/**
 * @brief This class provides publishing methods for common messages
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class Publisher
{
    using ObstaclePtr      = teb_local_planner::ObstaclePtr;
    using ObstContainer    = teb_local_planner::ObstContainer;
    using PointObstacle    = teb_local_planner::PointObstacle;
    using CircularObstacle = teb_local_planner::CircularObstacle;
    using LineObstacle     = teb_local_planner::LineObstacle;
    using PolygonObstacle  = teb_local_planner::PolygonObstacle;
    using Point2dContainer = teb_local_planner::Point2dContainer;

 public:
    /**
     * @brief Default constructor
     * @remarks do not forget to call initialize()
     */
    Publisher() = default;

    /**
     * @brief Constructor that initializes the class and registers topics
     * @param[in] nh         local ros::NodeHandle
     * @param[in] map_frame  the planning frame name
     */
    Publisher(ros::NodeHandle& nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame);

    /**
     * @brief Initializes the class and registers topics.
     *
     * Call this function if only the default constructor has been called before.
     * @param[in] nh         local ros::NodeHandle
     * @param[in] map_frame  the planning frame name
     */
    void initialize(ros::NodeHandle& nh, RobotDynamicsInterface::Ptr system, const std::string& map_frame);

    /**
     * @brief Publish a given local plan to the ros topic \e ../../local_plan
     * @param[in] local_plan Pose array describing the local plan
     */
    void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const;

    /**
     * @brief Publish a given local plan to the ros topic \e ../../local_plan
     * @param[in] local_plan Pose array describing the local plan
     */
    void publishLocalPlan(const corbo::TimeSeries& ts) const;

    /**
     * @brief Publish a given global plan to the ros topic \e ../../global_plan
     * @param[in] global_plan Pose array describing the global plan
     */
    void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const;

    /**
     * @brief Publish the visualization of the robot model
     *
     * @param[in] current_pose   Current pose of the robot
     * @param[in] robot_model    Subclass of BaseRobotFootprintModel
     * @param[in] map_frame      frame name for the msg header
     * @param[in] ns             Namespace for the marker objects
     * @param[in] color          Color of the footprint
     */
    void publishRobotFootprintModel(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::BaseRobotFootprintModel& robot_model,
                                    const std::string& ns = "RobotFootprintModel", const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));

    /**
     * @brief Publish obstacle positions to the ros topic \e ../../mpc_markers
     * @todo Move filling of the marker message to polygon class in order to avoid checking types.
     * @param[in] obstacles Obstacle container
     */
    void publishObstacles(const teb_local_planner::ObstContainer& obstacles) const;

    /**
     * @brief Publish via-points to the ros topic \e ../../teb_markers
     *
     * @todo add option to switch between points and poses (including orientation) to be published
     *
     * @param[in] via_points    via-point container
     * @param[in] ns            marker namespace
     */
    void publishViaPoints(const std::vector<teb_local_planner::PoseSE2>& via_points, const std::string& ns = "ViaPoints") const;

    /**
     * @brief Helper function to generate a color message from single values
     * @param[in] a Alpha value
     * @param[in] r Red value
     * @param[in] g Green value
     * @param[in] b Blue value
     * @return Color message
     */
    static std_msgs::ColorRGBA toColorMsg(float a, float r, float g, float b);

 private:
    bool _initialized = false;

    std::string _map_frame = "map";

    RobotDynamicsInterface::Ptr _system;

    ros::Publisher _local_plan_pub;
    ros::Publisher _global_plan_pub;
    ros::Publisher _mpc_marker_pub;
};

}  // namespace mpc_local_planner

#endif  // UTILS_PUBLISHER_H_
