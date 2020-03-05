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

#ifndef SYSTEMS_UNICYCLE_ROBOT_H_
#define SYSTEMS_UNICYCLE_ROBOT_H_

#include <mpc_local_planner/systems/base_robot_se2.h>

#include <cmath>

namespace mpc_local_planner {

/**
 * @brief Unicycle model
 *
 * This class implements the dynamics for a unicycle model
 * which can be used e.g., for differential-drive robots.
 * See [1] for a mathematical description and a figure.
 *
 * [1] S. M. LaValle, Planning Algorithms, Cambridge University Press, 2006.
 *     (Chapter 13, http://planning.cs.uiuc.edu/)
 *
 * @see BaseRobotSE2 RobotDynamicsInterface corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class UnicycleModel : public BaseRobotSE2
{
 public:
    //! Default constructor
    UnicycleModel() = default;

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<UnicycleModel>(); }

    // implements interface method
    int getInputDimension() const override { return 2; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() && "UnicycleModel::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        f[0] = u[0] * std::cos(x[2]);
        f[1] = u[0] * std::sin(x[2]);
        f[2] = u[1];
    }

    // implements interface method
    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist) const override
    {
        assert(u.size() == getInputDimension());
        twist.linear.x = u[0];
        twist.linear.y = twist.linear.z = 0;

        twist.angular.z = u[1];
        twist.angular.x = twist.angular.y = 0;
        return true;
    }
};

}  // namespace mpc_local_planner

#endif  // SYSTEMS_UNICYCLE_ROBOT_H_
