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

#include <nav_2d_utils/tf_help.hpp>
#include <mpc_local_planner/mpc_local_planner_ros.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <nav2_core/exceptions.hpp>
#include <tf2/exceptions.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.hpp>

namespace mpc_local_planner {

MpcLocalPlannerROS::MpcLocalPlannerROS()
    : _costmap_ros(nullptr),
      _tf(nullptr),
      _costmap_model(nullptr),
      _costmap_converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
      _no_infeasible_plans(0),
      /*last_preferred_rotdir_(RotType::none),*/
      _initialized(false)
{
}

void MpcLocalPlannerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node_ptr,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) {

  _nh = node_ptr;
  auto node = _nh.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();

  _costmap_ros = costmap_ros;
  _costmap     = _costmap_ros->getCostmap();
  _tf = tf;
  _name = name;

  initialize(node, name);
  // create visualization instance
  _publisher.initialize(node, _controller.getRobotDynamics(), _global_frame);
  _publisher.on_configure();

  return;
}

MpcLocalPlannerROS::~MpcLocalPlannerROS() {}

void MpcLocalPlannerROS::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string& name)
{
    // check if the plugin is already initialized
    if (!_initialized)
    {
        _params.declareParameters(node, name);
        _params.loadRosParamFromNodeHandle(node, name);
        // load plugin related main parameters

        // reserve some memory for obstacles
        _obstacles.reserve(700);

        _costmap = _costmap_ros->getCostmap(); // locking should be done in MoveBase.

        _costmap_model = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
        std::string costmap_model_name("costmap_model");
        _costmap_model->initialize(node, costmap_model_name, _name, _costmap_ros);

        _global_frame     = _costmap_ros->getGlobalFrameID();
        _robot_base_frame = _costmap_ros->getBaseFrameID();

        // create robot footprint/contour model for optimization
        _robot_model = getRobotFootprintFromParamServer(node, _costmap_ros);

        // create the planner instance
        if (!_controller.configure(node, _name, _obstacles, _robot_model, _via_points, &_params, &_publisher))
        {
            RCLCPP_ERROR(_logger, "Controller configuration failed.");
            return;
        }

        _intra_proc_node.reset(
            new rclcpp::Node("costmap_converter", node->get_namespace(),
            rclcpp::NodeOptions()));

        // Initialize a costmap to polygon converter
        if (!_params.costmap_converter_plugin.empty())
        {
            try
            {
                _costmap_converter         = _costmap_converter_loader.createSharedInstance(_params.costmap_converter_plugin);
                std::string converter_name = _costmap_converter_loader.getName(_params.costmap_converter_plugin);
                // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                boost::replace_all(converter_name, "::", "/");

                _costmap_converter->setOdomTopic(_params.odom_topic);
                _costmap_converter->initialize(_intra_proc_node);
                _costmap_converter->setCostmap2D(_costmap);
                const auto rate = std::make_shared<rclcpp::Rate>((double)_params.costmap_converter_rate);
                _costmap_converter->startWorker(rate, _costmap, _params.costmap_converter_spin_thread);
                RCLCPP_INFO_STREAM(_logger, "Costmap conversion plugin " << _params.costmap_converter_plugin << " loaded.");
            }
            catch (pluginlib::PluginlibException& ex)
            {
                RCLCPP_WARN(_logger,
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error "
                    "message: %s",
                    ex.what());
                _costmap_converter.reset();
            }
        }
        else
            RCLCPP_INFO(_logger, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

        // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        _footprint_spec = _costmap_ros->getRobotFootprint();
        nav2_costmap_2d::calculateMinAndMaxDistances(_footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius);

        // validate optimization footprint and costmap footprint
        validateFootprints(_robot_model->getInscribedRadius(), _robot_inscribed_radius, _controller.getInequalityConstraint()->getMinimumDistance());

        // setup callback for custom obstacles
        _custom_obst_sub = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>("obstacles", rclcpp::SystemDefaultsQoS(), std::bind(&MpcLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

        // setup callback for custom via-points
        _via_points_sub = node->create_subscription<nav_msgs::msg::Path>("via_points", rclcpp::SystemDefaultsQoS(), std::bind(&MpcLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

        // setup call back for custom global plan topic
        global_plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                    _params.global_plan_topic, 
                    rclcpp::SystemDefaultsQoS(),
                    std::bind(&MpcLocalPlannerROS::customGlobalPlanCB, this, std::placeholders::_1));   

        // set initialized flag
        _initialized = true;

        RCLCPP_DEBUG(_logger, "mpc_local_planner plugin initialized.");
    }
    else
    {
        RCLCPP_WARN(_logger, "mpc_local_planner has already been initialized, doing nothing.");
    }
}

void MpcLocalPlannerROS::setPlan(const nav_msgs::msg::Path& custom_global_plan_msg_)
{
    // check if plugin is initialized
    if (!_initialized)
    {
        RCLCPP_ERROR(_logger, "mpc_local_planner has not been initialized, please call initialize() before using this planner");
        return;
    }

    // store the global plan
  // store the global plan
    _global_plan.clear();
    _global_plan.reserve(custom_global_plan_msg_.poses.size());

    for(const auto &in_pose :custom_global_plan_msg_.poses)
    {
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.pose = in_pose.pose;
    out_pose.header = custom_global_plan_msg_.header;
    _global_plan.push_back(out_pose);
    }

    // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
    // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.
    return;
}

geometry_msgs::msg::TwistStamped MpcLocalPlannerROS::computeVelocityCommands(
		const geometry_msgs::msg::PoseStamped& pose,
		const geometry_msgs::msg::Twist& velocity)
{
    // check if plugin initialized
    if (!_initialized)
    {
      throw nav2_core::PlannerException(
        std::string("mpc_local_planner has not been initialized, please call initialize() before using this planner")
      );
    }

    RCLCPP_INFO(_logger,"COMPUTING VELOCITY COMMAND");

    geometry_msgs::msg::TwistStamped cmd_vel;

    cmd_vel.header.stamp    = _clock->now();
    cmd_vel.header.frame_id = _robot_base_frame;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    _costmap_ros->getRobotPose(robot_pose);
    _robot_pose = PoseSE2(robot_pose.pose);

    // Get robot velocity
    _robot_vel.linear.x  = pose.pose.position.x;
    _robot_vel.linear.y  = pose.pose.position.y;
    _robot_vel.angular.z = tf2::getYaw(pose.pose.orientation);

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*_tf, robot_pose, _global_plan, _params.controller.global_plan_prune_distance);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
    int goal_idx;
    geometry_msgs::msg::TransformStamped tf_plan_to_global;
    if (!transformGlobalPlan(*_tf, _global_plan, robot_pose, *_costmap, _global_frame, _params.controller.max_global_plan_lookahead_dist, transformed_plan,
                             &goal_idx, &tf_plan_to_global))
    {
      throw nav2_core::PlannerException(
        std::string("Could not transform the global plan to the frame of the controller")
      );
    }

    // update via-points container
    if (!_custom_via_points_active) updateViaPointsContainer(transformed_plan, _params.controller.global_plan_viapoint_sep);

    // check if global goal is reached
    geometry_msgs::msg::PoseStamped global_goal;
    tf2::doTransform(_global_plan.back(), global_goal, tf_plan_to_global);

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
      throw nav2_core::PlannerException(
        std::string("Transformed plan is empty. Cannot determine a local plan.")
      );
    }

    // Get current goal point (last point of the transformed plan)
    _robot_goal.x() = transformed_plan.back().pose.position.x;
    _robot_goal.y() = transformed_plan.back().pose.position.y;
    // Overwrite goal orientation if needed
    if (_params.controller.global_plan_overwrite_orientation)
    {
        _robot_goal.theta() = estimateLocalGoalOrientation(_global_plan, transformed_plan.back(), goal_idx, tf_plan_to_global);
        // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
        tf2::Quaternion q;
        q.setRPY(0, 0, _robot_goal.theta());
        tf2::convert(q, transformed_plan.back().pose.orientation);
    }
    else
    {
        _robot_goal.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    if (transformed_plan.size() == 1)  // plan only contains the goal
    {
        transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped());  // insert start (not yet initialized)
    }
    transformed_plan.front() = robot_pose;  // update start

    // clear currently existing obstacles
    _obstacles.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    if (_costmap_converter)
        updateObstacleContainerWithCostmapConverter();
    else
        updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    updateObstacleContainerWithCustomObstacles();

    // estimate current state vector and previous control
    // updateControlFromTwist()

    // Do not allow config changes during the following optimization step
    std::lock_guard<std::mutex> cfg_lock(_params.configMutex());

    // Now perform the actual planning
    // bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
    rclcpp::Time t = _clock->now();
    // controller_frequency might not be established in case planning takes longer:
    // value dt affects e.g. control deviation bounds (acceleration limits) and we want a goot start value
    // let's test how it works with the expected frequency instead of using the actual one
    double dt = 1.0 / _params.controller.controller_frequency;
    // double dt = time_last_cmd_.isZero() ? 0.0 : (t - time_last_cmd_).toSec();

    // set previous control value for control deviation bounds
    if (_u_seq && !_u_seq->isEmpty()) _controller.getOptimalControlProblem()->setPreviousControlInput(_u_seq->getValuesMap(0), dt);

    bool success = false;

    {
        std::lock_guard<std::mutex> vp_lock(_via_point_mutex);
        std::lock_guard<std::mutex> obst_lock(_custom_obst_mutex);
        success = _controller.step(transformed_plan, _robot_vel, dt, t, _u_seq, _x_seq);
    }

    if (!success)
    {
        _controller.reset();  // force reinitialization for next time
        RCLCPP_WARN(_logger,"mpc_local_planner was not able to obtain a local plan for the current setting.");

        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = _clock->now();
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        _last_cmd                  = cmd_vel.twist;
        return cmd_vel;
    }

    // Check feasibility (but within the first few states only)
    if (_params.footprint_model.is_footprint_dynamic)
    {
        // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        _footprint_spec = _costmap_ros->getRobotFootprint();
        nav2_costmap_2d::calculateMinAndMaxDistances(_footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius);
    }

    bool feasible = _controller.isPoseTrajectoryFeasible(_costmap_model, _footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius,
                                                         _params.collision_avoidance.collision_check_min_resolution_angular, _params.collision_avoidance.collision_check_no_poses);
    if (!feasible)
    {
        // now we reset everything to start again with the initialization of new trajectories.
        _controller.reset();  // force reinitialization for next time
        RCLCPP_WARN(_logger, "MpcLocalPlannerROS: trajectory is not feasible. Resetting planner...");
        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = _clock->now();
        _last_cmd                  = cmd_vel.twist;
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        return cmd_vel;
    }

    // Get the velocity command for this sampling interval
    // TODO(roesmann): we might also command more than just the imminent action, e.g. in a separate thread, until a new command is available
    if (!_u_seq || !_controller.getRobotDynamics()->getTwistFromControl(_u_seq->getValuesMap(0), cmd_vel.twist))
    {
        _controller.reset();
        RCLCPP_WARN(_logger, "MpcLocalPlannerROS: velocity command invalid. Resetting controller...");
        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = _clock->now();
        _last_cmd                  = cmd_vel.twist;
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        return cmd_vel;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to early termination or soft cosntraints).
    // saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
    //                 cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

    // a feasible solution should be found, reset counter
    _no_infeasible_plans = 0;

    // store last command (for recovery analysis etc.)
    _last_cmd      = cmd_vel.twist;
    _time_last_cmd = _clock->now();

    // Now visualize everything
    _publisher.publishLocalPlan(*_x_seq);
    _publisher.publishObstacles(_obstacles);
    _publisher.publishGlobalPlan(_global_plan);
    _publisher.publishViaPoints(_via_points);
    _publisher.publishRobotFootprintModel(_robot_pose, *_robot_model);

    return cmd_vel;
}

void MpcLocalPlannerROS::updateObstacleContainerWithCostmap()
{
    // Add costmap obstacles if desired
    if (_params.collision_avoidance.include_costmap_obstacles)
    {
        Eigen::Vector2d robot_orient = _robot_pose.orientationUnitVec();

        for (unsigned int i = 0; i < _costmap->getSizeInCellsX() - 1; ++i)
        {
            for (unsigned int j = 0; j < _costmap->getSizeInCellsY() - 1; ++j)
            {
                if (_costmap->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    Eigen::Vector2d obs;
                    _costmap->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    // check if obstacle is interesting (e.g. not far behind the robot)
                    Eigen::Vector2d obs_dir = obs - _robot_pose.position();
                    if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > _params.collision_avoidance.costmap_obstacles_behind_robot_dist) continue;

                    _obstacles.push_back(ObstaclePtr(new PointObstacle(obs)));
                }
            }
        }
    }
}

void MpcLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
    if (!_costmap_converter) return;

    // Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = _costmap_converter->getObstacles();
    if (!obstacles) return;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
        const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
        const geometry_msgs::msg::Polygon* polygon          = &obstacle->polygon;

        if (polygon->points.size() == 1 && obstacle->radius > 0)  // Circle
        {
            _obstacles.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
        }
        else if (polygon->points.size() == 1)  // Point
        {
            _obstacles.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
        }
        else if (polygon->points.size() == 2)  // Line
        {
            _obstacles.push_back(
                ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y, polygon->points[1].x, polygon->points[1].y)));
        }
        else if (polygon->points.size() > 2)  // Real polygon
        {
            PolygonObstacle* polyobst = new PolygonObstacle;
            for (std::size_t j = 0; j < polygon->points.size(); ++j)
            {
                polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
            }
            polyobst->finalizePolygon();
            _obstacles.push_back(ObstaclePtr(polyobst));
        }

        // Set velocity, if obstacle is moving
        if (!_obstacles.empty()) _obstacles.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
    }
}

void MpcLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
    // Add custom obstacles obtained via message
    std::lock_guard<std::mutex> l(_custom_obst_mutex);

    if (!_custom_obstacle_msg.obstacles.empty())
    {
        // We only use the global header to specify the obstacle coordinate system instead of individual ones
        Eigen::Affine3d obstacle_to_map_eig;
        try
        {
            geometry_msgs::msg::TransformStamped obstacle_to_map =
                _tf->lookupTransform(_global_frame, rclcpp::Time(0), _custom_obstacle_msg.header.frame_id, rclcpp::Time(0),
                                     _custom_obstacle_msg.header.frame_id, rclcpp::Duration::from_seconds(0.5));
            obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(_logger, "%s", ex.what());
            obstacle_to_map_eig.setIdentity();
        }

        for (size_t i = 0; i < _custom_obstacle_msg.obstacles.size(); ++i)
        {
            if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 1 && _custom_obstacle_msg.obstacles.at(i).radius > 0)  // circle
            {
                Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                _obstacles.push_back(
                    ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), _custom_obstacle_msg.obstacles.at(i).radius)));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 1)  // point
            {
                Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                _obstacles.push_back(ObstaclePtr(new PointObstacle((obstacle_to_map_eig * pos).head(2))));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 2)  // line
            {
                Eigen::Vector3d line_start(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                           _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                           _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                Eigen::Vector3d line_end(_custom_obstacle_msg.obstacles.at(i).polygon.points.back().x,
                                         _custom_obstacle_msg.obstacles.at(i).polygon.points.back().y,
                                         _custom_obstacle_msg.obstacles.at(i).polygon.points.back().z);
                _obstacles.push_back(
                    ObstaclePtr(new LineObstacle((obstacle_to_map_eig * line_start).head(2), (obstacle_to_map_eig * line_end).head(2))));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.empty())
            {
                RCLCPP_WARN(_logger, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
                continue;
            }
            else  // polygon
            {
                PolygonObstacle* polyobst = new PolygonObstacle;
                for (size_t j = 0; j < _custom_obstacle_msg.obstacles.at(i).polygon.points.size(); ++j)
                {
                    Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points[j].x,
                                        _custom_obstacle_msg.obstacles.at(i).polygon.points[j].y,
                                        _custom_obstacle_msg.obstacles.at(i).polygon.points[j].z);
                    polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
                }
                polyobst->finalizePolygon();
                _obstacles.push_back(ObstaclePtr(polyobst));
            }

            // Set velocity, if obstacle is moving
            if (!_obstacles.empty())
                _obstacles.back()->setCentroidVelocity(_custom_obstacle_msg.obstacles[i].velocities, _custom_obstacle_msg.obstacles[i].orientation);
        }
    }
}

void MpcLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
{
    _via_points.clear();

    if (min_separation <= 0) return;

    std::size_t prev_idx = 0;
    for (std::size_t i = 1; i < transformed_plan.size(); ++i)  // skip first one, since we do not need any point before the first min_separation [m]
    {
        // check separation to the previous via-point inserted
        if (distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation) continue;

        // add via-point
        _via_points.emplace_back(transformed_plan[i].pose);
        prev_idx = i;
    }
}

bool MpcLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::msg::PoseStamped& global_pose,
                                         std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot)
{
    if (global_plan.empty()) return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        geometry_msgs::msg::TransformStamped global_to_plan_transform =
            tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, rclcpp::Time(0));
        geometry_msgs::msg::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

        double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::msg::PoseStamped>::iterator it        = global_plan.begin();
        std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
        while (it != global_plan.end())
        {
            double dx      = robot.pose.position.x - it->pose.position.x;
            double dy      = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
        }
        if (erase_end == global_plan.end()) return false;

        if (erase_end != global_plan.begin()) global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_DEBUG(_logger, "Cannot prune path since no transform is available: %s\n", ex.what());
        return false;
    }
    return true;
}

bool MpcLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                                             const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap,
                                             const std::string& global_frame, double max_plan_length,
                                             std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx,
                                             geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
{
    // this method is a slightly modified version of base_local_planner/goal_functions.h

    const geometry_msgs::msg::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
        if (global_plan.empty())
        {
            RCLCPP_ERROR(_logger, "Received plan with zero length");
            *current_goal_idx = 0;
            return false;
        }

        // get plan_to_global_transform from plan frame to global_frame
        geometry_msgs::msg::TransformStamped plan_to_global_transform = tf.lookupTransform(
            global_frame, rclcpp::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, rclcpp::Duration::from_seconds(0.5));

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::msg::PoseStamped robot_pose;

        rclcpp::Duration transform_tolerance = rclcpp::Duration::from_seconds(0.5);

        nav_2d_utils::transformPose(_tf, plan_pose.header.frame_id, global_pose, robot_pose, transform_tolerance);

        // we'll discard points on the plan that are outside the local costmap
        double dist_threshold =
            std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
        dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                 // located on the border of the local costmap

        int i                    = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist           = 1e10;

        // we need to loop to a point on the plan that is within a certain distance of the robot
        for (int j = 0; j < (int)global_plan.size(); ++j)
        {
            double x_diff      = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff      = robot_pose.pose.position.y - global_plan[j].pose.position.y;
            double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (new_sq_dist > sq_dist_threshold) break;  // force stop if we have reached the costmap border

            if (new_sq_dist < sq_dist)  // find closest distance
            {
                sq_dist = new_sq_dist;
                i       = j;
            }
        }

        geometry_msgs::msg::PoseStamped newer_pose;

        double plan_length = 0;  // check cumulative Euclidean distance along the plan

        // now we'll transform until points are outside of our distance threshold
        while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
        {
            const geometry_msgs::msg::PoseStamped& pose = global_plan[i];
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
            sq_dist       = x_diff * x_diff + y_diff * y_diff;

            // caclulate distance to previous pose
            if (i > 0 && max_plan_length > 0)
                plan_length += teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

            ++i;
        }

        // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
        // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
        if (transformed_plan.empty())
        {
            tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
        }
        else
        {
            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
        }

        // Return the transformation from the global plan to the global planning frame if desired
        if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }
    catch (tf2::LookupException& ex)
    {
        RCLCPP_ERROR(_logger, "No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        RCLCPP_ERROR(_logger, "Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        RCLCPP_ERROR(_logger, "Extrapolation Error: %s\n", ex.what());
        if (global_plan.size() > 0)
            RCLCPP_ERROR(_logger, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                      global_plan[0].header.frame_id.c_str());

        return false;
    }

    return true;
}

double MpcLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                                                        const geometry_msgs::msg::PoseStamped& local_goal, int current_goal_idx,
                                                        const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
    int n = (int)global_plan.size();

    // check if we are near the global goal already
    if (current_goal_idx > n - moving_average_length - 2)
    {
        if (current_goal_idx >= n - 1)  // we've exactly reached the goal
        {
            return tf2::getYaw(local_goal.pose.orientation);
        }
        else
        {
            tf2::Quaternion global_orientation;
            tf2::convert(global_plan.back().pose.orientation, global_orientation);
            tf2::Quaternion rotation;
            tf2::convert(tf_plan_to_global.transform.rotation, rotation);
            // TODO(roesmann): avoid conversion to tf2::Quaternion
            return tf2::getYaw(rotation * global_orientation);
        }
    }

    // reduce number of poses taken into account if the desired number of poses is not available
    moving_average_length =
        std::min(moving_average_length, n - current_goal_idx - 1);  // maybe redundant, since we have checked the vicinity of the goal before

    std::vector<double> candidates;
    geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
    geometry_msgs::msg::PoseStamped tf_pose_kp1;

    int range_end = current_goal_idx + moving_average_length;
    for (int i = current_goal_idx; i < range_end; ++i)
    {
        // Transform pose of the global plan to the planning frame
        tf2::doTransform(global_plan.at(i + 1), tf_pose_kp1, tf_plan_to_global);

        // calculate yaw angle
        candidates.push_back(
            std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y, tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x));

        if (i < range_end - 1) tf_pose_k = tf_pose_kp1;
    }
    return teb_local_planner::average_angles(candidates);
}

void MpcLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    RCLCPP_WARN_EXPRESSION(_logger, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!",
                  opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}

void MpcLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
{
    std::lock_guard<std::mutex> l(_custom_obst_mutex);
    _custom_obstacle_msg = *obst_msg;
}

void MpcLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
{
    RCLCPP_INFO_ONCE(_logger, "Via-points received. This message is printed once.");
    if (_params.controller.global_plan_viapoint_sep > 0)
    {
        RCLCPP_WARN(_logger,
            "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
            "Ignoring custom via-points.");
        _custom_via_points_active = false;
        return;
    }

    std::lock_guard<std::mutex> lock(_via_point_mutex);
    _via_points.clear();
    for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
    {
        _via_points.emplace_back(pose.pose);
    }
    _custom_via_points_active = !_via_points.empty();
}
void MpcLocalPlannerROS::customGlobalPlanCB(const nav_msgs::msg::Path::SharedPtr global_plan_msg)
{
    std::lock_guard<std::mutex> l(custom_global_plan_mutex_);
    custom_global_plan_msg_ = *global_plan_msg;  
}

teb_local_planner::RobotFootprintModelPtr MpcLocalPlannerROS::getRobotFootprintFromParamServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
                                                                                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    std::string model_name;
    if (!nh->get_parameter(_name + "." + "footprint_model.type", model_name))
    {
        RCLCPP_INFO(_logger, "No robot footprint model specified for trajectory optimization. Using point-shaped model.");
        return std::make_shared<teb_local_planner::PointRobotFootprint>();
    }

    // from costmap_2d
    if (model_name.compare("costmap_2d") == 0)
    {
        if (!costmap_ros)
        {
            RCLCPP_WARN_STREAM(_logger, "Costmap 2d pointer is null. Using point model instead.");
            return std::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        RCLCPP_INFO(_logger, "Footprint model loaded from costmap_2d for trajectory optimization.");
        return getRobotFootprintFromCostmap2d(*costmap_ros);
    }

    // point
    if (model_name.compare("point") == 0)
    {
        RCLCPP_INFO(_logger, "Footprint model 'point' loaded for trajectory optimization.");
        return std::make_shared<teb_local_planner::PointRobotFootprint>();
    }

    // circular
    if (model_name.compare("circular") == 0)
    {
        // get radius
        double radius;
        if (!nh->get_parameter(_name + "." + "footprint_model.radius", radius))
        {
            RCLCPP_ERROR_STREAM(_logger, "Footprint model 'circular' cannot be loaded for trajectory optimization, since param '/footprint_model.radius' does not exist. Using point-model instead.");
            return std::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        RCLCPP_INFO_STREAM(_logger, "Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
        return std::make_shared<teb_local_planner::CircularRobotFootprint>(radius);
    }

    // line
    if (model_name.compare("line") == 0)
    {
        // check parameters
        if (!nh->has_parameter(_name + "." + "footprint_model.line_start") || !nh->has_parameter(_name + "." + "footprint_model.line_end"))
        {
            RCLCPP_ERROR_STREAM(_logger, "Footprint model 'line' cannot be loaded for trajectory optimization, since param '/footprint_model.line_start' and/or '/footprint_model.line_end' do not exist. Using point-model instead.");
            return std::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        // get line coordinates
        std::vector<double> line_start, line_end;
        nh->get_parameter(_name + "." + "footprint_model.line_start", line_start);
        nh->get_parameter(_name + "." + "footprint_model.line_end", line_end);
        if (line_start.size() != 2 || line_end.size() != 2)
        {
            RCLCPP_ERROR_STREAM(_logger,
                "Footprint model 'line' cannot be loaded for trajectory optimization, since param '/footprint_model.line_start' and/or '/footprint_model.line_end' do not contain x and y coordinates (2D). Using point-model instead.");
            return std::make_shared<teb_local_planner::PointRobotFootprint>();
        }

        RCLCPP_INFO_STREAM(_logger, "Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: [" << line_end[0] << ","
                                                                << line_end[1] << "]m) loaded for trajectory optimization.");
        return std::make_shared<teb_local_planner::LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()),
                                                                         Eigen::Map<const Eigen::Vector2d>(line_end.data()));
    }

    // two circles
    if (model_name.compare("two_circles") == 0)
    {
        // check parameters
        if (!nh->has_parameter(_name + "." + "footprint_model.front_offset") || !nh->has_parameter(_name + "." + "footprint_model.front_radius") ||
            !nh->has_parameter(_name + "." + "footprint_model.rear_offset") || !nh->has_parameter(_name + "." + "footprint_model.rear_radius"))
        {
            RCLCPP_ERROR_STREAM(_logger,"Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '/footprint_model.front_offset', '/footprint_model.front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using "
                                "point-model instead.");
            return std::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        double front_offset, front_radius, rear_offset, rear_radius;
        nh->get_parameter(_name + "." + "footprint_model.front_offset", front_offset);
        nh->get_parameter(_name + "." + "footprint_model.front_radius", front_radius);
        nh->get_parameter(_name + "." + "footprint_model.rear_offset", rear_offset);
        nh->get_parameter(_name + "." + "footprint_model.rear_radius", rear_radius);
        RCLCPP_INFO_STREAM(_logger, "Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius
                                                                        << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius
                                                                        << "m) loaded for trajectory optimization.");
        return std::make_shared<teb_local_planner::TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    std::string footprint_string;

    if (!nh->get_parameter(_name + "." + "footprint_model.vertices", footprint_string) )
    {
      RCLCPP_ERROR(_logger,
                   "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '%s.footprint_model.vertices' does not exist. Using point-model instead.",
                   nh->get_namespace());

      return std::make_shared<teb_local_planner::PointRobotFootprint>();
    }

    std::vector<geometry_msgs::msg::Point> footprint;
    // get vertices
    if (nav2_costmap_2d::makeFootprintFromString(footprint_string, footprint))
    {
      Point2dContainer polygon;
      for(const auto &pt : footprint) {
          polygon.push_back(Eigen::Vector2d(pt.x, pt.y));
      }
      RCLCPP_INFO(_logger, "Footprint model 'polygon' loaded for trajectory optimization.");
      return std::make_shared<teb_local_planner::PolygonRobotFootprint>(polygon);
    }
    else
    {
      RCLCPP_ERROR(_logger,
                "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '%s.footprint_model.vertices' does not define an array of coordinates. Using point-model instead.",
                nh->get_namespace());
      return std::make_shared<teb_local_planner::PointRobotFootprint>();
    }

  }
    // otherwise
    RCLCPP_WARN_STREAM(_logger, "Unknown robot footprint model specified with parameter '/footprint_model.type'. Using point model instead.");
    return std::make_shared<teb_local_planner::PointRobotFootprint>();
}

teb_local_planner::RobotFootprintModelPtr MpcLocalPlannerROS::getRobotFootprintFromCostmap2d(nav2_costmap_2d::Costmap2DROS& costmap_ros)
{
    Point2dContainer footprint;
    Eigen::Vector2d pt;
    geometry_msgs::msg::Polygon polygon = costmap_ros.getRobotFootprintPolygon();

    for (int i = 0; i < polygon.points.size(); ++i)
    {
        pt.x() = polygon.points[i].x;
        pt.y() = polygon.points[i].y;

        footprint.push_back(pt);
    }
    return std::make_shared<teb_local_planner::PolygonRobotFootprint>(footprint);
}

void MpcLocalPlannerROS::activate() {
  _publisher.on_activate();

  return;
}
void MpcLocalPlannerROS::deactivate() {
  _publisher.on_deactivate();
  return;
}
void MpcLocalPlannerROS::cleanup() {
  _publisher.on_cleanup();
  _costmap_converter->stopWorker();

  return;
}

void MpcLocalPlannerROS::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  auto node = _nh.lock();
  node->set_parameter(rclcpp::Parameter(_name + "." + "max_vel_x", rclcpp::ParameterValue(speed_limit)));
}

}  // end namespace mpc_local_planner

// register this planner both as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlannerROS, nav2_core::Controller)
