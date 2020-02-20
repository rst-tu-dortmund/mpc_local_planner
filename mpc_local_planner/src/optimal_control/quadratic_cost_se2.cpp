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

#include <mpc_local_planner/optimal_control/quadratic_cost_se2.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <cmath>

namespace mpc_local_planner {

void QuadraticFormCostSE2::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(!_integral_form);
    assert(cost.size() == getNonIntegralStateTermDimension(k));

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_lsq_form)
    {
        if (_Q_diagonal_mode)
            cost.noalias() = _Q_diag_sqrt * xd;
        else
            cost.noalias() = _Q_sqrt * xd;
    }
    else
    {
        if (_Q_diagonal_mode)
            cost.noalias() = xd.transpose() * _Q_diag * xd;
        else
            cost.noalias() = xd.transpose() * _Q * xd;
    }
}

void QuadraticFormCostSE2::computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                           const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(_integral_form);
    assert(cost.size() == 1);

    cost[0] = 0;

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_Q_diagonal_mode)
        cost[0] += xd.transpose() * _Q_diag * xd;
    else
        cost[0] += xd.transpose() * _Q * xd;

    if (_zero_u_ref)
    {
        if (_R_diagonal_mode)
            cost[0] += u_k.transpose() * _R_diag * u_k;
        else
            cost[0] += u_k.transpose() * _R * u_k;
    }
    else
    {
        Eigen::VectorXd ud = u_k - _u_ref->getReferenceCached(k);
        if (_R_diagonal_mode)
            cost[0] += ud.transpose() * _R_diag * ud;
        else
            cost[0] += ud.transpose() * _R * ud;
    }
}

void QuadraticStateCostSE2::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(!_integral_form);
    assert(cost.size() == getNonIntegralStateTermDimension(k));

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_lsq_form)
    {
        if (_diagonal_mode)
            cost.noalias() = _Q_diag_sqrt * xd;
        else
            cost.noalias() = _Q_sqrt * xd;
    }
    else
    {
        if (_diagonal_mode)
            cost.noalias() = xd.transpose() * _Q_diag * xd;
        else
            cost.noalias() = xd.transpose() * _Q * xd;
    }
}

void QuadraticStateCostSE2::computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                            const Eigen::Ref<const Eigen::VectorXd>& u_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(_integral_form);
    assert(cost.size() == 1);

    cost[0] = 0;

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_diagonal_mode)
        cost[0] += xd.transpose() * _Q_diag * xd;
    else
        cost[0] += xd.transpose() * _Q * xd;
}

}  // namespace mpc_local_planner
