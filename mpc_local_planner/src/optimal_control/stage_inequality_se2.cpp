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

#include <mpc_local_planner/optimal_control/full_discretization_grid_base_se2.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/pose_se2.h>

#include <cmath>
#include <memory>

namespace mpc_local_planner {

int StageInequalitySE2::getNonIntegralControlDeviationTermDimension(int k) const { return _num_du_lb_finite + _num_du_ub_finite; }

int StageInequalitySE2::getNonIntegralStateTermDimension(int k) const
{
    if (!_obstacles) return 0;
    assert(k < _relevant_obstacles.size());
    return (int)_relevant_obstacles[k].size();
}

int StageInequalitySE2::getNonIntegralStateDtTermDimension(int k) const
{
    if (!_obstacles) return 0;
    assert(k < _relevant_dyn_obstacles.size());
    return (int)_relevant_dyn_obstacles[k].size();
}

bool StageInequalitySE2::update(int n, double /*t*/, corbo::ReferenceTrajectoryInterface& /*xref*/, corbo::ReferenceTrajectoryInterface& /*uref*/,
                                corbo::ReferenceTrajectoryInterface* /*sref*/, bool /* single_dt*/, const Eigen::VectorXd& x0,
                                corbo::StagePreprocessor::Ptr /*stage_preprocessor*/, const std::vector<double>& /*dts*/,
                                const corbo::DiscretizationGridInterface* grid)
{
    PRINT_WARNING_COND_ONCE(!_obstacles, "StageInequalitySE2 requires a valid obstacle ptr which is not null (ignoring obstacle avoidance).");
    PRINT_WARNING_COND_ONCE(!_robot_model, "StageInequalitySE2 requires a valid robot model ptr which is not null (ignoring obstacle avoidance).");

    // Setup obstacle avoidance

    // we currently require a full discretization grid as we want to have fast access to
    // individual states without requiring any new simulation.
    // Alternatively, other grids could be used in combination with method getStateAndControlTimeSeries()
    const FullDiscretizationGridBaseSE2* fd_grid = static_cast<const FullDiscretizationGridBaseSE2*>(grid);

    bool new_dimensions = (n != _relevant_obstacles.size()) || (n != _relevant_dyn_obstacles.size());

    _relevant_obstacles.resize(n);
    _relevant_dyn_obstacles.resize(n);

    teb_local_planner::PoseSE2 pose;

    // iterate states but skip start state as it is not subject to optimization
    for (int k = 1; k < n; ++k)
    {
        double left_min_dist  = std::numeric_limits<double>::max();
        double right_min_dist = std::numeric_limits<double>::max();
        ObstaclePtr left_obstacle;
        ObstaclePtr right_obstacle;

        const Eigen::VectorXd& xk = fd_grid->getState(k);
        assert(xk.size() >= 3);
        // TODO(roesmann): Overload robot fooprint model functions in teb_local_planner to avoid this copy:
        pose.position()             = xk.head(2);
        pose.theta()                = xk.coeffRef(2);
        Eigen::Vector2d pose_orient = pose.orientationUnitVec();
        // Eigen::Vector2d pose_orient(std::cos(xk[2]), std::sin(xk[2]));

        int num_prev_obst = (int)_relevant_obstacles[k].size();
        _relevant_obstacles[k].clear();
        int num_prev_dyn_obst = (int)_relevant_dyn_obstacles[k].size();
        _relevant_dyn_obstacles[k].clear();

        // iterate obstacles
        if (_obstacles && _robot_model)
        {
            for (const ObstaclePtr& obst : *_obstacles)
            {
                // check for dynamic obstacle
                if (_enable_dynamic_obstacles && obst->isDynamic())
                {
                    // we consider all dynamic obstacles by now
                    // TODO(roesmann): we might remove obstacles that "go away" from the trajectory
                    // or more generally that any intersection in the future is unlikely
                    _relevant_dyn_obstacles[k].push_back(obst);
                    continue;
                }

                // calculate distance to robot model
                double dist = _robot_model->calculateDistance(pose, obst.get());

                // force considering obstacle if really close to the current pose
                if (dist < _obstacle_filter_force_inclusion_dist)
                {
                    _relevant_obstacles[k].push_back(obst);
                    continue;
                }
                // cut-off distance
                if (dist > _obstacle_filter_cutoff_dist) continue;

                // determine side (left or right) and assign obstacle if closer than the previous one
                if (cross2d(pose_orient, obst->getCentroid()) > 0)  // left
                {
                    if (dist < left_min_dist)
                    {
                        left_min_dist = dist;
                        left_obstacle = obst;
                    }
                }
                else
                {
                    if (dist < right_min_dist)
                    {
                        right_min_dist = dist;
                        right_obstacle = obst;
                    }
                }
            }
        }

        // add left and right obstacle
        if (left_obstacle) _relevant_obstacles[k].push_back(left_obstacle);
        if (right_obstacle) _relevant_obstacles[k].push_back(right_obstacle);

        // check if dimensions changed
        new_dimensions = new_dimensions || (_relevant_obstacles[k].size() != num_prev_obst);
        new_dimensions = new_dimensions || (_relevant_dyn_obstacles[k].size() != num_prev_dyn_obst);
    }

    // update current dt
    _current_dt = grid->getFirstDt();  // TODO(roesmann): this does currently not work with non-uniform grids

    // setup control deviation constraint
    int num_du_lb_finite = _du_lb.size() > 0 ? (_du_lb.array() > -corbo::CORBO_INF_DBL).count() : 0;
    int num_du_ub_finite = _du_ub.size() > 0 ? (_du_ub.array() < corbo::CORBO_INF_DBL).count() : 0;
    if (num_du_lb_finite != _num_du_lb_finite) new_dimensions = true;
    if (num_du_ub_finite != _num_du_ub_finite) new_dimensions = true;
    _num_du_lb_finite = num_du_lb_finite;
    _num_du_ub_finite = num_du_ub_finite;

    //  return true if structure requires a changed
    return new_dimensions;
}

void StageInequalitySE2::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(k < _relevant_obstacles.size());
    assert(cost.size() == _relevant_obstacles[k].size());

    // TODO(roesmann): Overload robot fooprint model functions in teb_local_planner to avoid this copy:
    teb_local_planner::PoseSE2 pose(x_k[0], x_k[1], x_k[2]);
    for (int i = 0; i < (int)_relevant_obstacles[k].size(); ++i)
    {
        cost[i] = _min_obstacle_dist - _robot_model->calculateDistance(pose, _relevant_obstacles[k][i].get());
    }
}

void StageInequalitySE2::computeNonIntegralStateDtTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, double dt_k,
                                                       Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(k < _relevant_dyn_obstacles.size());
    assert(cost.size() == _relevant_dyn_obstacles[k].size());

    // TODO(roesmann): Overload robot fooprint model functions in teb_local_planner to avoid this copy:
    teb_local_planner::PoseSE2 pose(x_k[0], x_k[1], x_k[2]);
    for (int i = 0; i < (int)_relevant_dyn_obstacles[k].size(); ++i)
    {
        cost[i] = _min_obstacle_dist - _robot_model->estimateSpatioTemporalDistance(pose, _relevant_dyn_obstacles[k][i].get(), (double)k * dt_k);
    }
}

void StageInequalitySE2::computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                                                const Eigen::Ref<const Eigen::VectorXd>& u_prev, double dt_prev,
                                                                Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.size() == _num_du_lb_finite + _num_du_ub_finite);
    if (cost.size() == 0) return;  // TODO(roesmann): necessary?
    if (k == 0 && dt_prev == 0)
    {
        cost.setZero();  // this is fine as dt_prev for k==0 is not subject to optimization
        return;
    }

    assert(_num_du_lb_finite == 0 || _du_lb.size() == u_k.size());
    assert(_num_du_ub_finite == 0 || _du_ub.size() == u_k.size());
    assert(dt_prev != 0);

    int idx_lb = 0;
    int idx_ub = 0;
    for (int i = 0; i < u_k.size(); ++i)
    {
        if (_du_lb[i] > -corbo::CORBO_INF_DBL)
        {
            cost[idx_lb] = _du_lb[i] - (u_k[i] - u_prev[i]) / dt_prev;
            ++idx_lb;
        }
        if (_du_ub[i] < corbo::CORBO_INF_DBL)
        {
            cost[_num_du_lb_finite + idx_ub] = (u_k[i] - u_prev[i]) / dt_prev - _du_ub[i];
            ++idx_ub;
        }
    }
}

void StageInequalitySE2::setControlDeviationBounds(const Eigen::VectorXd& du_lb, const Eigen::VectorXd& du_ub)
{
    _du_lb = du_lb;
    _du_ub = du_ub;
}

}  // namespace mpc_local_planner
