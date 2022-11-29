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

#ifndef MPC_LOCAL_PLANNER_ROS_H_
#define MPC_LOCAL_PLANNER_ROS_H_

#include <rclcpp/rclcpp.hpp>

#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// Navigation2 local planner base class and utilities
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>

// mpc_local_planner related classes
#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_config.h>
#include <mpc_local_planner/utils/publisher.h>

// teb_local_planner related classes
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <costmap_converter/costmap_converter_interface.h>

// message types
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// costmap
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <memory>
#include <mutex>

namespace mpc_local_planner {

/**
 * @class MpcLocalPlannerROS
 * @brief Implements both nav_core::BaseLocalPlanner and mbf_costmap_core::CostmapController abstract
 * interfaces, so the teb_local_planner plugin can be used both in move_base and move_base_flex (MBF).
 * @todo Escape behavior, more efficient obstacle handling
 */
class MpcLocalPlannerROS : public nav2_core::Controller
{
    using PoseSE2                = teb_local_planner::PoseSE2;
    using RobotFootprintModelPtr = teb_local_planner::RobotFootprintModelPtr;
    using Point2dContainer       = teb_local_planner::Point2dContainer;
    using ObstContainer          = teb_local_planner::ObstContainer;
    using ViaPointContainer      = std::vector<PoseSE2>;

    using ObstaclePtr      = teb_local_planner::ObstaclePtr;
    using PointObstacle    = teb_local_planner::PointObstacle;
    using CircularObstacle = teb_local_planner::CircularObstacle;
    using LineObstacle     = teb_local_planner::LineObstacle;
    using PolygonObstacle  = teb_local_planner::PolygonObstacle;

 public:
    /**
     * @brief Default constructor of the plugin
     */
    MpcLocalPlannerROS();
    /**
     * @brief  Destructor of the plugin
     */
    ~MpcLocalPlannerROS();

    /**
     * @brief Initializes the teb plugin
     * @param name The name of the instance
     * @param tf Pointer to a tf buffer
     * @param costmap_ros Cost map representing occupied and free space
     */
    void configure(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
      std::string name,
      const std::shared_ptr<tf2_ros::Buffer> & tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void initialize(nav2_util::LifecycleNode::SharedPtr node, std::string& name);

    /**
     * @brief Set the plan that the teb local planner is following
     * @param orig_global_plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;

    /**
     * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid trajectory was found, false otherwise
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity) override;

    /** @name Public utility functions/methods */
    //@{

    /**
     * @brief Get the current robot footprint/contour model
     * @param nh const reference to the local ros::NodeHandle
     * @param costmap_ros pointer to an intialized instance of costmap_2d::Costmap2dROS
     * @return Robot footprint model used for optimization
     */
    RobotFootprintModelPtr getRobotFootprintFromParamServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros = nullptr);

    /**
     * @brief Get the current robot footprint/contour model
     * @param costmap_ros reference to an intialized instance of costmap_2d::Costmap2dROS
     * @return Robot footprint model used for optimization
     */
    static RobotFootprintModelPtr getRobotFootprintFromCostmap2d(nav2_costmap_2d::Costmap2DROS& costmap_ros);

    //@}

 protected:
    /**
     * @brief Update internal obstacle vector based on occupied costmap cells
     * @remarks All occupied cells will be added as point obstacles.
     * @remarks All previous obstacles are cleared.
     * @sa updateObstacleContainerWithCostmapConverter
     * @todo Include temporal coherence among obstacle msgs (id vector)
     * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
     */
    void updateObstacleContainerWithCostmap();

    /**
     * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
     * @remarks Requires a loaded costmap_converter plugin.
     * @remarks All previous obstacles are cleared.
     * @sa updateObstacleContainerWithCostmap
     */
    void updateObstacleContainerWithCostmapConverter();

    /**
     * @brief Update internal obstacle vector based on custom messages received via subscriber
     * @remarks All previous obstacles are NOT cleared. Call this method after other update methods.
     * @sa updateObstacleContainerWithCostmap, updateObstacleContainerWithCostmapConverter
     */
    void updateObstacleContainerWithCustomObstacles();

    /**
     * @brief Update internal via-point container based on the current reference plan
     * @remarks All previous via-points will be cleared.
     * @param transformed_plan (local) portion of the global plan (which is already transformed to the planning frame)
     * @param min_separation minimum separation between two consecutive via-points
     */
    void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation);


    /**
     * @brief Callback for custom obstacles that are not obtained from the costmap
     * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
     */
    void customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg);


    /**
     * @brief Callback for custom via-points
     * @param via_points_msg pointer to the message containing a list of via-points
     */
    void customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg);

    /**
     * @brief Prune global plan such that already passed poses are cut off
     *
     * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
     * If no valid transformation can be found, the method returns \c false.
     * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
     * If no pose within the specified treshold \c dist_behind_robot can be found,
     * nothing will be pruned and the method returns \c false.
     * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
     * @param tf A reference to a tf buffer
     * @param global_pose The global pose of the robot
     * @param[in,out] global_plan The plan to be transformed
     * @param dist_behind_robot Distance behind the robot that should be kept [meters]
     * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
     */
    void customGlobalPlanCB(const nav_msgs::msg::Path::SharedPtr global_plan_msg);
    
    /**
        * @brief Callback for custom global plan topic
        * @param global_plan_msg pointer to the message containing the global plan
        */
    bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::msg::PoseStamped& global_pose,
                         std::vector<geometry_msgs::msg::PoseStamped>& _global_plan, double dist_behind_robot = 1);

    /**
     * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
     *
     * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
     * such that the index of the current goal pose is returned as well as
     * the transformation between the global plan and the planning frame.
     * @param tf A reference to a tf buffer
     * @param global_plan The plan to be transformed
     * @param global_pose The global pose of the robot
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also
     * bounded by the local costmap size!]
     * @param[out] transformed_plan Populated with the transformed plan
     * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @return \c true if the global plan is transformed, \c false otherwise
     */
    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::msg::PoseStamped>& _global_plan,
                             const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                             double max_plan_length, std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx = NULL,
                             geometry_msgs::msg::TransformStamped* tf_plan_to_global = NULL) const;

    /**
     * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
     *
     * If the current (local) goal point is not the final one (global)
     * substitute the goal orientation by the angle of the direction vector between
     * the local goal and the subsequent pose of the global plan.
     * This is often helpful, if the global planner does not consider orientations. \n
     * A moving average filter is utilized to smooth the orientation.
     * @param global_plan The global plan
     * @param local_goal Current local goal
     * @param current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @param moving_average_length number of future poses of the global plan to be taken into account
     * @return orientation (yaw-angle) estimate
     */
    double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& _global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                                        int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global,
                                        int moving_average_length = 3) const;

    /**
     * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
     *
     * This method prints warnings if validation fails.
     * @remarks Currently, we only validate the inscribed radius of the footprints
     * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
     * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
     * @param min_obst_dist desired distance to obstacles
     */
    void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);

    void setSpeedLimit(const double& speed_limit, const bool& percentage);

 private:
    // Definition of member variables
    nav2_util::LifecycleNode::WeakPtr _nh;
    std::string _name;

    rclcpp::Logger _logger{rclcpp::get_logger("mpc_local_planner")};
    rclcpp::Clock::SharedPtr _clock;

    rclcpp::Node::SharedPtr _intra_proc_node;

    // external objects (store weak pointers)
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;  //!< Pointer to the costmap ros wrapper, received from the navigation stack
    nav2_costmap_2d::Costmap2D* _costmap;         //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
    std::shared_ptr<tf2_ros::Buffer> _tf;                    //!< pointer to tf buffer

    // internal objects
    MpcController _controller;
    ObstContainer _obstacles;  //!< Obstacle vector that should be considered during local trajectory optimization
    Publisher _publisher;
    std::shared_ptr<dwb_critics::ObstacleFootprintCritic> _costmap_model;

    corbo::TimeSeries::Ptr _x_seq = std::make_shared<corbo::TimeSeries>();
    corbo::TimeSeries::Ptr _u_seq = std::make_shared<corbo::TimeSeries>();

    std::vector<geometry_msgs::msg::PoseStamped> _global_plan;  //!< Store the current global plan

    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> _costmap_converter_loader;  //!< Load costmap converter plugins at runtime
    std::shared_ptr<costmap_converter::BaseCostmapToPolygons> _costmap_converter;              //!< Store the current costmap_converter

    rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr _custom_obst_sub;                          //!< Subscriber for custom obstacles received via a ObstacleMsg.
    std::mutex _custom_obst_mutex;                             //!< Mutex that locks the obstacle array (multi-threaded)
    std::mutex custom_global_plan_mutex_; //!< Mutex that locks the path array (multi-threaded)
    costmap_converter_msgs::msg::ObstacleArrayMsg _custom_obstacle_msg;  //!< Copy of the most recent obstacle message

    ViaPointContainer _via_points;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _via_points_sub;         //!< Subscriber for custom via-points received via a Path msg.
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_; //!< Subscriber for custom global plan topic received via a Path msg.    
    bool _custom_via_points_active = false;  //!< Keep track whether valid via-points have been received from via_points_sub_
    std::mutex _via_point_mutex;             //!< Mutex that locks the via_points container (multi-threaded)

    PoseSE2 _robot_pose;                   //!< Store current robot pose
    PoseSE2 _robot_goal;                   //!< Store current robot goal
    geometry_msgs::msg::Twist _robot_vel;       //!< Store current robot translational and angular velocity (vx, vy, omega)
    rclcpp::Time _time_last_infeasible_plan;  //!< Store at which time stamp the last infeasible plan was detected
    int _no_infeasible_plans = 0;          //!< Store how many times in a row the planner failed to find a feasible plan.
    geometry_msgs::msg::Twist _last_cmd;        //!< Store the last control command generated in computeVelocityCommands()
    rclcpp::Time _time_last_cmd;

    // store path message for custom global plan topic
    nav_msgs::msg::Path custom_global_plan_msg_;

    RobotFootprintModelPtr _robot_model;

    std::vector<geometry_msgs::msg::Point> _footprint_spec;  //!< Store the footprint of the robot
    double _robot_inscribed_radius;                     //!< The radius of the inscribed circle of the robot (collision possible)
    double _robot_circumscribed_radius;                 //!< The radius of the circumscribed circle of the robot

    std::string _global_frame;      //!< The frame in which the controller will run
    std::string _robot_base_frame;  //!< Used as the base frame id of the robot

    // flags
    bool _initialized;  //!< Keeps track about the correct initialization of this class

    MpcConfig _params;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

};  // end namespace mpc_local_planner

#endif  // MPC_LOCAL_PLANNER_ROS_H_
