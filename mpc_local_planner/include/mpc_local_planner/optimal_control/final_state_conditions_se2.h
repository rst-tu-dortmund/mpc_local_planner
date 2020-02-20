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

#ifndef FINAL_STATE_CONDITIONS_SE2_H_
#define FINAL_STATE_CONDITIONS_SE2_H_

#include <corbo-optimal-control/functions/final_state_constraints.h>
#include <corbo-optimal-control/functions/final_state_cost.h>

#include <cmath>
#include <memory>

namespace mpc_local_planner {

/**
 * @brief Quadratic final state cost (specialized for SE2)
 *
 * This class implements quadratic final costs
 * \f[
 *    J_f(x) = (x_{ref} - x)^T Q_f (x_{xref} - x)
 * \f]
 * However, this class ensures that the distance (x_{ref} - x) is
 * computed properly in SO(2) for the third state component.
 *
 * @see corbo::QuadraticFinalStateCost
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class QuadraticFinalStateCostSE2 : public corbo::QuadraticFinalStateCost
{
 public:
    using Ptr = std::shared_ptr<QuadraticFinalStateCostSE2>;

    QuadraticFinalStateCostSE2() : corbo::QuadraticFinalStateCost() {}

    QuadraticFinalStateCostSE2(const Eigen::Ref<const Eigen::MatrixXd>& Qf, bool lsq_form = false) : corbo::QuadraticFinalStateCost(Qf, lsq_form) {}

    corbo::FinalStageCost::Ptr getInstance() const override { return std::make_shared<QuadraticFinalStateCostSE2>(); }

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;
};

/**
 * @brief Terminal ball constraint (specialized for SE2)
 *
 * This class ensures that the distance (x_{ref} - x) is
 * computed properly in SO(2) for the third state component.
 *
 * @see corbo::TerminalBall
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class TerminalBallSE2 : public corbo::TerminalBall
{
 public:
    using Ptr = std::shared_ptr<TerminalBallSE2>;

    TerminalBallSE2() : corbo::TerminalBall() {}

    TerminalBallSE2(const Eigen::Ref<const Eigen::MatrixXd>& S, double gamma) : corbo::TerminalBall(S, gamma) {}

    corbo::FinalStageConstraint::Ptr getInstance() const override { return std::make_shared<TerminalBallSE2>(); }

    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;
};

}  // namespace mpc_local_planner

#endif  // FINAL_STATE_CONDITIONS_SE2_H_
