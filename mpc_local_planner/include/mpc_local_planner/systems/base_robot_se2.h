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

#ifndef SYSTEMS_BASE_ROBOT_SE2_H_
#define SYSTEMS_BASE_ROBOT_SE2_H_

#include <mpc_local_planner/systems/robot_dynamics_interface.h>

namespace mpc_local_planner {

/**
 * @brief Specialization of RobotDynamicsInterface for mobile robots operating in SE2
 *
 * This abstract class defines a base class for robots in which the full state vector x
 * is embedded in SE2, i.e. x = [pos_x, pos_y, theta] with pos_x, pos_y as real numbers
 * and theta in [-pi, pi).
 *
 * Note, these models allow for simple conversion between state vectors and poses.
 *
 * @see RobotDynamicsInterface corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class BaseRobotSE2 : public RobotDynamicsInterface
{
 public:
    using Ptr = std::shared_ptr<BaseRobotSE2>;

    //! Default constructor
    BaseRobotSE2() = default;

    // implements interface method
    corbo::SystemDynamicsInterface::Ptr getInstance() const override = 0;

    // implements interface method
    int getInputDimension() const override = 0;
    // implements interface method
    int getStateDimension() const override { return 3; }

    // implements interface method
    bool isContinuousTime() const override { return true; }

    // implements interface method
    bool isLinear() const override { return false; }

    // implements interface method
    void getPositionFromState(const Eigen::Ref<const Eigen::VectorXd>& x, double& pos_x, double& pos_y) const override
    {
        assert(x.size() == getStateDimension());
        pos_x = x[0];
        pos_y = x[1];
    }

    // implements interface method
    void getPoseSE2FromState(const Eigen::Ref<const Eigen::VectorXd>& x, double& pos_x, double& pos_y, double& theta) const override
    {
        assert(x.size() == getStateDimension());
        pos_x = x[0];
        pos_y = x[1];
        theta = x[2];
    }

    // implements interface method
    void getSteadyStateFromPoseSE2(double pos_x, double pos_y, double theta, Eigen::Ref<Eigen::VectorXd> x) const override
    {
        assert(x.size() == getStateDimension());
        x[0] = pos_x;
        x[1] = pos_y;
        x[2] = theta;
        if (x.size() > 3) x.tail(x.size() - 3).setZero();
    }

    // implements interface method
    virtual bool mergeStateFeedbackAndOdomFeedback(const teb_local_planner::PoseSE2& odom_pose, const geometry_msgs::Twist& odom_twist,
                                                   Eigen::Ref<Eigen::VectorXd> x) const override
    {
        assert(x.size() == getStateDimension());
        x[0] = odom_pose.x();
        x[1] = odom_pose.y();
        x[2] = odom_pose.theta();
        return true;
    }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override = 0;
};

}  // namespace mpc_local_planner

#endif  // SYSTEMS_BASE_ROBOT_SE2_H_
