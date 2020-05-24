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

#ifndef SYSTEMS_KINEMATIC_BICYCLE_MODEL_H_
#define SYSTEMS_KINEMATIC_BICYCLE_MODEL_H_

#include <mpc_local_planner/systems/base_robot_se2.h>

#include <cmath>

namespace mpc_local_planner {

/**
 * @brief Kinematic Bicycle Model with Velocity Input
 *
 * This class implements the dynamics for a kinematic bicycle model.
 * Note, in this model the robot coordinate system is located at the center of gravity
 * which is between the rear and the front axle.
 * In case you want to define the coordinate system at the rear axle,
 * please refer to the simplified equations simple_car.h (SimpleCarModel).
 *
 * [1] R. Rajamani, Vehicle Dynamics and Control, Springer, 2012.
 * [2] J. Kong et al., Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design, IV, 2015.
 *
 * @see SimpleCarModel SimpleCarFrontWheelDrivingModel BaseRobotSE2 RobotDynamicsInterface
 *      corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class KinematicBicycleModelVelocityInput : public BaseRobotSE2
{
 public:
    //! Default constructor
    KinematicBicycleModelVelocityInput() = default;

    //! Constructs model with given wheelbase
    KinematicBicycleModelVelocityInput(double lr, double lf) : _lr(lr), _lf(lf) {}

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<KinematicBicycleModelVelocityInput>(); }

    // implements interface method
    int getInputDimension() const override { return 2; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() &&
               "KinematicBicycleModelVelocityInput::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        double beta = std::atan(_lr / (_lf + _lr) * std::tan(u[1]));

        f[0] = u[0] * std::cos(x[2] + beta);
        f[1] = u[0] * std::sin(x[2] + beta);
        f[2] = u[0] * std::sin(beta) / _lr;
    }

    // implements interface method
    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist) const override
    {
        assert(u.size() == getInputDimension());
        twist.linear.x = u[0];
        twist.linear.y = twist.linear.z = 0;

        twist.angular.z = u[1];  // warning, this is the angle and not the angular vel
        twist.angular.x = twist.angular.y = 0;

        return true;
    }

    //! Set parameters
    void setParameters(double lr, double lf)
    {
        _lr = lr;
        _lf = lf;
    }
    //! Get length between COG and front axle
    double getLengthFront() const { return _lf; }
    //! Get length between COG and rear axle
    double getLengthRear() const { return _lr; }

 protected:
    double _lr = 1.0;
    double _lf = 1.0;
};

}  // namespace mpc_local_planner

#endif  // SYSTEMS_KINEMATIC_BICYCLE_MODEL_H_
