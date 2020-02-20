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

#ifndef SYSTEMS_ROBOT_DYNAMICS_INTERFACE_H_
#define SYSTEMS_ROBOT_DYNAMICS_INTERFACE_H_

#include <corbo-core/types.h>
#include <corbo-systems/system_dynamics_interface.h>
#include <geometry_msgs/Twist.h>
#include <teb_local_planner/pose_se2.h>

#include <memory>

namespace mpc_local_planner {

/**
 * @brief Specialization of SystemDynamicsInterface for mobile robots
 *
 * This abstract class extends cobro::SystemDynamicsInterface
 * by (abstract) conversion methods between states, poses embedded in SE2,
 * controls and twist.
 *
 * @see BaseRobotSE2 corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class RobotDynamicsInterface : public corbo::SystemDynamicsInterface
{
 public:
    using Ptr = std::shared_ptr<RobotDynamicsInterface>;

    /**
     * @brief Convert state vector to position (x,y)
     *
     * @param[in]  x           Reference to the state vector [getStateDimension() x 1]
     * @param[out] pos_x       X-component of the position
     * @param[out] pos_y       Y-component of the position
     */
    virtual void getPositionFromState(const Eigen::Ref<const Eigen::VectorXd>& x, double& pos_x, double& pos_y) const = 0;

    /**
     * @brief Convert state vector to pose (x,y,theta)
     *
     * @param[in]  x           Reference to the state vector [getStateDimension() x 1]
     * @param[out] pos_x       X-component of the position part
     * @param[out] pos_y       Y-component of the position part
     * @param[out] theta       Orientation (yaw angle) in [-pi, pi)
     */
    virtual void getPoseSE2FromState(const Eigen::Ref<const Eigen::VectorXd>& x, double& pos_x, double& pos_y, double& theta) const = 0;

    /**
     * @brief Convert state vector to PoseSE2 object
     *
     * @param[in]  x           Reference to the state vector [getStateDimension() x 1]
     * @param[out] pose        PoseSE2 object
     */
    virtual void getPoseSE2FromState(const Eigen::Ref<const Eigen::VectorXd>& x, teb_local_planner::PoseSE2& pose) const
    {
        getPoseSE2FromState(x, pose.x(), pose.y(), pose.theta());
    }

    /**
     * @brief Convert pose (x,y,theta) to steady state
     *
     * Converts a pose to state vector with dimensions [getStateDimension() x 1].
     * In case getStateDimension() > 3, the state information is incomplete
     * and hence it is assumed that a steady state for this particular pose is avaiable,
     * which means that there exist a control u such that f(x,u) = 0 (continuous time) or
     * f(x,u) = x (discrete time).
     *
     * For example, in case the remaining state components correspond to just integrators,
     * they can be set to zero.
     *
     * @param[in]  pos_x       X-component of the position part
     * @param[in]  pos_y       Y-component of the position part
     * @param[in]  theta       Orientation (yaw angle) in [-pi, pi)
     * @param[out] x           Reference to the state vector [getStateDimension() x 1]
     */
    virtual void getSteadyStateFromPoseSE2(double pos_x, double pos_y, double theta, Eigen::Ref<Eigen::VectorXd> x) const = 0;

    /**
     * @brief Convert PoseSE2 to steady state
     *
     * See description of getSteadyStateFromPoseSE2(double pos_x, double pos_y, double theta, Eigen::Ref<Eigen::VectorXd> x)
     *
     * @param[in]  pose        PoseSE2 object
     * @param[out] x           Reference to the state vector [getStateDimension() x 1]
     */
    virtual void getSteadyStateFromPoseSE2(const teb_local_planner::PoseSE2& pose, Eigen::Ref<Eigen::VectorXd> x) const
    {
        getSteadyStateFromPoseSE2(pose.x(), pose.y(), pose.theta(), x);
    }

    /**
     * @brief Convert control vector to twist message
     *
     * Convert the control vector to a twist message (containing velocity information)
     * if possible.
     *
     * @todo Maybe add current state x as optional input to allow for computing a twist out of the first state and control
     *
     * @param[in]  u           Reference to the control vector [getInputDimension() x 1]
     * @param[out] twist       Reference to the twist message
     * @return true, if conversion was successful, false otherwise
     */
    virtual bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist) const = 0;

    /**
     * @brief Merge custom state feedback with pose and twist feedback
     *
     * Many robots in ROS publish their state information (pose, linear and angular velocity)e.g. via an odometry message
     * (potentially corrected by localization).
     * However, for more complex robot models, it might not be possible to obtain the full state information without any observer
     * or further measurements.
     * For this purpose, custom state feedback can be provided. But as the complete navigation stack relies on the information obtained by
     * odometry, this method allows for merging of both sources.
     * In particular, it overrides only state components related to the pose and velocity.
     *
     * @param[in]     odom_pose   PoseSE2 object obtained from odometry
     * @param[in]     odom_twist  geometry::Twist message obtained from odometry
     * @param[in,out] x           Custom state feedback in which related will be overriden [getStateDimension() x 1]
     * @return true, if merging was successful, false otherwise
     */
    virtual bool mergeStateFeedbackAndOdomFeedback(const teb_local_planner::PoseSE2& odom_pose, const geometry_msgs::Twist& odom_twist,
                                                   Eigen::Ref<Eigen::VectorXd> x) const = 0;

    virtual void reset() {}
};

}  // namespace mpc_local_planner

#endif  // SYSTEMS_ROBOT_DYNAMICS_INTERFACE_H_
