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

#ifndef FINITE_DIFFERENCES_COLLOCATION_H_
#define FINITE_DIFFERENCES_COLLOCATION_H_

#include <corbo-numerics/finite_differences_collocation.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <functional>
#include <memory>

namespace mpc_local_planner {

/**
 * @brief Collocation via forward differences (specialized for SE2)
 *
 * Forward differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = f(x_k, u_k)
 * \f]
 *
 * @see ForwardDiffCollocation FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class ForwardDiffCollocationSE2 : public corbo::FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    corbo::FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<ForwardDiffCollocationSE2>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const corbo::SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(x1.size() >= 3);
        // assert(dt > 0 && "dt must be greater then zero!");

        system.dynamics(x1, u1, error);
        error.head(2) -= (x2.head(2) - x1.head(2)) / dt;
        error.coeffRef(2) -= normalize_theta(x2.coeffRef(2) - x1.coeffRef(2)) / dt;
        if (x1.size() > 3)
        {
            int n = x1.size() - 3;
            error.tail(n) -= (x2.tail(n) - x1.tail(n)) / dt;
        }
    }
};

/**
 * @brief Collocation via midpoint differences  (specialized for SE2)
 *
 * Midpoint differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = f(0.5*(x_k + x_{k+1}), u_k)
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MidpointDiffCollocationSE2 : public corbo::FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    corbo::FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<MidpointDiffCollocationSE2>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const corbo::SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        Eigen::VectorXd midpoint = 0.5 * (x1 + x2);
        // fix angular component
        midpoint.coeffRef(2) = interpolate_angle(x1.coeffRef(2), x2.coeffRef(2), 0.5);
        system.dynamics(midpoint, u1, error);
        error.head(2) -= (x2.head(2) - x1.head(2)) / dt;
        error.coeffRef(2) -= normalize_theta(x2.coeffRef(2) - x1.coeffRef(2)) / dt;
        if (x1.size() > 3)
        {
            int n = x1.size() - 3;
            error.tail(n) -= (x2.tail(n) - x1.tail(n)) / dt;
        }
    }
};

/**
 * @brief Collocation via Crank-Nicolson differences (specialized for SE2)
 *
 * Crank-Nicolson differences approximate \f$ \dot{x} = f(x, u) \f$ in the following manner:
 * \f[
 *    \frac{x_{k+1} - x_k}{\delta T} = 0.5 * ( f(x_k, u_k) + f(x_{k+1}, u_k))
 * \f]
 *
 * @see FiniteDifferencesCollocationInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class CrankNicolsonDiffCollocationSE2 : public corbo::FiniteDifferencesCollocationInterface
{
 public:
    // Implements interface method
    corbo::FiniteDifferencesCollocationInterface::Ptr getInstance() const override { return std::make_shared<CrankNicolsonDiffCollocationSE2>(); }

    // Implements interface method
    void computeEqualityConstraint(const StateVector& x1, const InputVector& u1, const StateVector& x2, double dt,
                                   const corbo::SystemDynamicsInterface& system, Eigen::Ref<Eigen::VectorXd> error) override
    {
        assert(error.size() == x1.size());
        assert(dt > 0 && "dt must be greater then zero!");

        Eigen::VectorXd f1(x1.size());
        system.dynamics(x1, u1, f1);
        system.dynamics(x2, u1, error);
        // error = (x2 - x1) / dt - 0.5 * (f1 + error);
        error.head(2) -= (x2.head(2) - x1.head(2)) / dt - 0.5 * (f1.head(2) + error.head(2));
        error.coeffRef(2) -= normalize_theta(x2.coeffRef(2) - x1.coeffRef(2)) / dt - 0.5 * (f1.coeffRef(2) + error.coeffRef(2));
        if (x1.size() > 3)
        {
            int n = x1.size() - 3;
            error.tail(n) -= (x2.tail(n) - x1.tail(n)) / dt - 0.5 * (f1.tail(n) + error.tail(n));
        }
    }
};

}  // namespace mpc_local_planner

#endif  // FINITE_DIFFERENCES_COLLOCATION_H_
