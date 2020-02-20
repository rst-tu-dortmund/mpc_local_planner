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

#include <mpc_local_planner/optimal_control/min_time_via_points_cost.h>

#include <mpc_local_planner/optimal_control/full_discretization_grid_base_se2.h>
#include <mpc_local_planner/utils/math_utils.h>

#include <cmath>
#include <memory>

namespace mpc_local_planner {

int MinTimeViaPointsCost::getNonIntegralStateTermDimension(int k) const
{
    assert(k < _vp_association.size());
    assert(_vp_association[k].first.size() == _vp_association[k].second);
    return (int)_vp_association[k].second;
}

bool MinTimeViaPointsCost::update(int n, double /*t*/, corbo::ReferenceTrajectoryInterface& /*xref*/, corbo::ReferenceTrajectoryInterface& /*uref*/,
                                  corbo::ReferenceTrajectoryInterface* /*sref*/, bool single_dt, const Eigen::VectorXd& x0,
                                  corbo::StagePreprocessor::Ptr /*stage_preprocessor*/, const std::vector<double>& /*dts*/,
                                  const corbo::DiscretizationGridInterface* grid)
{
    if (!_via_points)
    {
        PRINT_WARNING("MinTimeViaPointsCost::update(): no via_point container specified");
        return !_vp_association.empty();
    }

    // setup minimum time cost
    _single_dt = single_dt;
    if (single_dt)
        _time_weight = n - 1;
    else
        _time_weight = 1.0;

    // Setup via points

    // we currently require a full discretization grid as we want to have fast access to
    // individual states without requiring any new simulation.
    // Alternatively, other grids could be used in combination with method getStateAndControlTimeSeries()
    const FullDiscretizationGridBaseSE2* fd_grid = static_cast<const FullDiscretizationGridBaseSE2*>(grid);

    assert(n == fd_grid->getN());

    bool new_structure = (n != _vp_association.size());
    if (new_structure)
    {
        _vp_association.resize(n, std::make_pair<std::vector<const teb_local_planner::PoseSE2*>, std::size_t>({}, 0));
    }

    // clear previous association
    for (auto& item : _vp_association)
    {
        item.first.clear();
    }

    int start_pose_idx = 0;

    for (const teb_local_planner::PoseSE2& pose : *_via_points)
    {
        int idx = fd_grid->findClosestPose(pose.x(), pose.y(), start_pose_idx);

        if (_via_points_ordered) start_pose_idx = idx + 2;  // skip a point to have a DOF inbetween for further via-points

        // check if point coincides with final state or is located behind it
        if (idx > n - 2) idx = n - 2;  // set to a pose before the goal, since it is never fixed

        // check if point coincides with start or is located before it
        if (idx < 1)
        {
            if (_via_points_ordered)
                idx = 1;  // try to connect the via point with the second pose. Grid adaptation might add new poses inbetween later if enabled.
            else
            {
                PRINT_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
                continue;  // skip via points really close or behind the current robot pose
            }
        }

        _vp_association[idx].first.push_back(&pose);
    }

    // check for structure changes
    for (auto& item : _vp_association)
    {
        if (item.first.size() != item.second)
        {
            item.second   = item.first.size();
            new_structure = true;
            // but continue until end for next time
        }
    }

    //  return true if structure requires a changed
    return new_structure;
}

void MinTimeViaPointsCost::computeNonIntegralDtTerm(int k, double dt, Eigen::Ref<Eigen::VectorXd> cost) const
{
    if (!_single_dt || k == 0)
        cost[0] = _time_weight * dt;
    else
    {
        PRINT_DEBUG_NAMED("this method should not be called in single_dt mode and k>0");
    }
}

void MinTimeViaPointsCost::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(_via_points);
    assert(k < _vp_association.size());
    assert(cost.size() == _vp_association[k].first.size());
    assert(_vp_association[k].second == _vp_association[k].first.size());

    for (int i = 0; i < (int)_vp_association[k].second; ++i)
    {
        cost[i] = _vp_position_weight * (_vp_association[k].first[i]->position() - x_k.head(2)).squaredNorm();
        if (_vp_orientation_weight > 0)
        {
            cost[i] += _vp_orientation_weight * normalize_theta(_vp_association[k].first[i]->theta() - x_k[2]);
        }
    }
}

}  // namespace mpc_local_planner
