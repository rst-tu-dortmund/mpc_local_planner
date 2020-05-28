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

#ifndef STAGE_INEQUALITY_SE2_H_
#define STAGE_INEQUALITY_SE2_H_

#include <corbo-core/reference_trajectory.h>
#include <corbo-optimal-control/functions/stage_functions.h>

#include <teb_local_planner/robot_footprint_model.h>

#include <teb_local_planner/obstacles.h>

#include <cmath>
#include <memory>

namespace mpc_local_planner {

/**
 * @brief Stage inequality constraint for obstacle avoidance and control deviation limits
 *
 * This class defines the inequality constraint for obstacle avoidance and control
 * deviation limits (in case limits are active).
 *
 * The current obstacle association strategy is borrowed from the teb_local_planner.
 * It takes the robot footprint model and the geometric obstacle shapes into account.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class StageInequalitySE2 : public corbo::StageInequalityConstraint
{
 public:
    using Ptr = std::shared_ptr<StageInequalitySE2>;

    using ObstaclePtr = teb_local_planner::ObstaclePtr;

    // implements interface method
    corbo::StageInequalityConstraint::Ptr getInstance() const override { return std::make_shared<StageInequalitySE2>(); }

    // implements interface method
    bool hasNonIntegralTerms(int k) const override { return true; }
    // implements interface method
    bool hasIntegralTerms(int k) const override { return false; }

    // implements interface method
    int getNonIntegralStateTermDimension(int k) const override;
    // implements interface method
    int getNonIntegralStateDtTermDimension(int k) const override;
    // implements interface method
    int getNonIntegralControlDeviationTermDimension(int k) const override;

    // implements interface method
    bool update(int n, double /*t*/, corbo::ReferenceTrajectoryInterface& /*xref*/, corbo::ReferenceTrajectoryInterface& /*uref*/,
                corbo::ReferenceTrajectoryInterface* /*sref*/, bool /* single_dt*/, const Eigen::VectorXd& x0,
                corbo::StagePreprocessor::Ptr /*stage_preprocessor*/, const std::vector<double>& /*dts*/,
                const corbo::DiscretizationGridInterface* grid) override;

    // implements interface method
    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;
    // implements interface method
    void computeNonIntegralStateDtTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, double dt_k,
                                       Eigen::Ref<Eigen::VectorXd> cost) const override;
    // implements interface method
    void computeNonIntegralControlDeviationTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& u_k, const Eigen::Ref<const Eigen::VectorXd>& u_prev,
                                                double dt, Eigen::Ref<Eigen::VectorXd> cost) const override;

    //! Set reference to obstacle vector (must remain allocated)
    void setObstacleVector(const teb_local_planner::ObstContainer& obstacles) { _obstacles = &obstacles; }
    //! Set robot footprint model
    void setRobotFootprintModel(teb_local_planner::RobotFootprintModelPtr robot_model) { _robot_model = robot_model; }

    //! Set minimum distance allowed
    void setMinimumDistance(double min_dist) { _min_obstacle_dist = min_dist; }
    //! Set parameters for prior filtering of obstacles
    void setObstacleFilterParameters(double force_inclusion_dist, double cutoff_dist)
    {
        _obstacle_filter_force_inclusion_dist = force_inclusion_dist;
        _obstacle_filter_cutoff_dist          = cutoff_dist;
    }
    //! Set to true to enable dynamic obstacle (constant-velocity prediction)
    void setEnableDynamicObstacles(bool enable_dyn_obst) { _enable_dynamic_obstacles = enable_dyn_obst; }
    //! Specify lower and upper bounds for limiting the control deviation
    void setControlDeviationBounds(const Eigen::VectorXd& du_lb, const Eigen::VectorXd& du_ub);

    //! Get current minimum distance parameter value
    double getMinimumDistance() const { return _min_obstacle_dist; }

 protected:
    const teb_local_planner::ObstContainer* _obstacles = nullptr;
    std::vector<teb_local_planner::ObstContainer> _relevant_obstacles;  // TODO: we can also store raw pointers as _via_points is locked by mutex
    std::vector<teb_local_planner::ObstContainer> _relevant_dyn_obstacles;

    teb_local_planner::RobotFootprintModelPtr _robot_model;

    double _current_dt    = 0.1;
    int _num_du_lb_finite = 0;
    int _num_du_ub_finite = 0;

    double _min_obstacle_dist                    = 0.1;
    double _obstacle_filter_force_inclusion_dist = 1.5;
    double _obstacle_filter_cutoff_dist          = 5;
    bool _enable_dynamic_obstacles               = false;

    Eigen::VectorXd _du_lb;
    Eigen::VectorXd _du_ub;
};

}  // namespace mpc_local_planner

#endif  // STAGE_INEQUALITY_SE2_H_
