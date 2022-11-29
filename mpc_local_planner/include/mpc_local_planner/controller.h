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

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <corbo-controllers/predictive_controller.h>

#include <dwb_critics/obstacle_footprint.hpp>

#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <mpc_local_planner/mpc_config.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>
#include <mpc_local_planner/systems/robot_dynamics_interface.h>
#include <mpc_local_planner/utils/publisher.h>

#include <mpc_local_planner_msgs/msg/state_feedback.hpp>
#include <mpc_local_planner_msgs/msg/optimal_control_result.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <memory>
#include <mutex>

namespace mpc_local_planner {

/**
 * @brief MPC controller for mobile robots
 *
 * @ingroup controllers
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MpcController : public corbo::PredictiveController
{
 public:
    using Ptr     = std::shared_ptr<MpcController>;
    using PoseSE2 = teb_local_planner::PoseSE2;

    MpcController() = default;

    bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, std::string name, const teb_local_planner::ObstContainer& obstacles, teb_local_planner::RobotFootprintModelPtr robot_model,
                   const std::vector<teb_local_planner::PoseSE2>& via_points, MpcConfig* params, Publisher* publisher);
    bool step(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist& vel, double dt, rclcpp::Time t, corbo::TimeSeries::Ptr u_seq,
              corbo::TimeSeries::Ptr x_seq);

    bool step(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist& vel, double dt, rclcpp::Time t,
              corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq);

    // implements interface method
    corbo::ControllerInterface::Ptr getInstance() const override { return std::make_shared<MpcController>(); }
    static corbo::ControllerInterface::Ptr getInstanceStatic() { return std::make_shared<MpcController>(); }

    void setOptimalControlProblem(corbo::OptimalControlProblemInterface::Ptr ocp) = delete;

    RobotDynamicsInterface::Ptr getRobotDynamics() { return _dynamics; }
    StageInequalitySE2::Ptr getInequalityConstraint() { return _inequality_constraint; }

    void stateFeedbackCallback(const mpc_local_planner_msgs::msg::StateFeedback::ConstSharedPtr msg);

    void publishOptimalControlResult();

    /**
     * @brief Check whether the planned trajectory is feasible or not.
     *
     * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
     * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
     * @param costmap_model Pointer to the costmap model
     * @param footprint_spec The specification of the footprint of the robot in world coordinates
     * @param inscribed_radius The radius of the inscribed circle of the robot
     * @param circumscribed_radius The radius of the circumscribed circle of the robot
     * @param min_resolution_collision_check_angular Min angular resolution during the costmap collision check:
     *        if not respected intermediate samples are added. [rad]
     * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
     * @return \c true, if the robot footprint along the first part of the trajectory intersects with
     *         any obstacle in the costmap, \c false otherwise.
     */
    virtual bool isPoseTrajectoryFeasible(std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                          double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
                                          double min_resolution_collision_check_angular = M_PI, int look_ahead_idx = -1);

    // implements interface method
    void reset() override;

 protected:
    corbo::DiscretizationGridInterface::Ptr configureGrid(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh);
    RobotDynamicsInterface::Ptr configureRobotDynamics(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh);
    corbo::NlpSolverInterface::Ptr configureSolver(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh);
    corbo::StructuredOptimalControlProblem::Ptr configureOcp(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const teb_local_planner::ObstContainer& obstacles,
                                                             teb_local_planner::RobotFootprintModelPtr robot_model,
                                                             const std::vector<teb_local_planner::PoseSE2>& via_points);

    bool generateInitialStateTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf,
                                        const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, bool backward);

    std::string name_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;
    MpcConfig* params_;
    Publisher* publisher_;
    rclcpp::Logger logger_{rclcpp::get_logger("mpc_local_planner")};
    corbo::DiscretizationGridInterface::Ptr _grid;
    RobotDynamicsInterface::Ptr _dynamics;
    corbo::NlpSolverInterface::Ptr _solver;
    StageInequalitySE2::Ptr _inequality_constraint;
    corbo::StructuredOptimalControlProblem::Ptr _structured_ocp;

    bool _ocp_successful      = false;
    std::size_t _ocp_seq      = 0;

    rclcpp::Subscription<mpc_local_planner_msgs::msg::StateFeedback>::SharedPtr _x_feedback_sub;
    std::mutex _x_feedback_mutex;
    rclcpp::Time _recent_x_time;
    Eigen::VectorXd _recent_x_feedback;

    teb_local_planner::PoseSE2 _last_goal;
    corbo::DiscreteTimeReferenceTrajectory _x_seq_init;
};

}  // namespace mpc_local_planner

#endif  // CONTROLLER_H_
