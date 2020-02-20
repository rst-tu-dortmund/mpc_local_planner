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

#ifndef MIN_TIME_VIA_POINTS_COSTS_H_
#define MIN_TIME_VIA_POINTS_COSTS_H_

#include <corbo-core/reference_trajectory.h>
#include <corbo-optimal-control/functions/stage_functions.h>

#include <teb_local_planner/pose_se2.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace mpc_local_planner {

/**
 * @brief Hybrid cost function with minimum time and via-point objectives
 *
 * This class defines a joint objective of minimum time and via-point attraction.
 * A user defined weight controls the trade-off between both individual objectives.
 *
 * The current via-point strategy is borrowed from the teb_local_planner.
 *
 * @todo we can implement this class as LSQ version as well
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class MinTimeViaPointsCost : public corbo::StageCost
{
 public:
    using Ptr = std::shared_ptr<MinTimeViaPointsCost>;

    using ViaPointContainer = std::vector<teb_local_planner::PoseSE2>;

    //! Default constructor
    MinTimeViaPointsCost() = default;

    /**
     * @brief Construct with via-point container and weights
     *
     * @param[in] via_points             Reference to the via-point container (must remain allocated)
     * @param[in] position_weight        Via-point attraction weight for the position part
     * @param[in] orientation_weight     Via-point attraction weight for the angular part
     * @param[in] via_points_orderedx    True, if via-points should be associated w.r.t. the ordering in \c via_points
     */
    MinTimeViaPointsCost(const ViaPointContainer& via_points, double position_weight, double orientation_weight, bool via_points_ordered)
        : _via_points_ordered(via_points_ordered)
    {
        setViaPointContainer(via_points);
        setViaPointWeights(position_weight, orientation_weight);
    }

    // implements interface method
    corbo::StageCost::Ptr getInstance() const override { return std::make_shared<MinTimeViaPointsCost>(); }

    // implements interface method
    bool hasNonIntegralTerms(int k) const override { return true; }
    // implements interface method
    bool hasIntegralTerms(int k) const override { return false; }

    // implements interface method
    int getNonIntegralDtTermDimension(int k) const override { return (k == 0 || !_single_dt) ? 1 : 0; }
    // implements interface method
    bool isLsqFormNonIntegralDtTerm(int k) const override { return false; }
    // implements interface method
    int getNonIntegralStateTermDimension(int k) const override;

    // implements interface method
    bool update(int n, double /*t*/, corbo::ReferenceTrajectoryInterface& /*xref*/, corbo::ReferenceTrajectoryInterface& /*uref*/,
                corbo::ReferenceTrajectoryInterface* /*sref*/, bool single_dt, const Eigen::VectorXd& x0,
                corbo::StagePreprocessor::Ptr /*stage_preprocessor*/, const std::vector<double>& /*dts*/,
                const corbo::DiscretizationGridInterface* grid) override;

    // implements interface method
    void computeNonIntegralDtTerm(int k, double dt, Eigen::Ref<Eigen::VectorXd> cost) const override;
    // implements interface method
    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    //! Set reference to via-point container (warning, object must remain allocated)
    void setViaPointContainer(const ViaPointContainer& via_points) { _via_points = &via_points; }
    //! Set if the optimzier should try to match the via-point ordering
    void setViaPointOrderedMode(bool ordered) { _via_points_ordered = ordered; }
    //! Set weights for via-point attraction
    void setViaPointWeights(double position_weight, double orientation_weight)
    {
        _vp_position_weight    = position_weight;
        _vp_orientation_weight = orientation_weight;
    }

 protected:
    bool _single_dt     = false;
    double _time_weight = 1;

    bool _via_points_ordered      = false;
    double _vp_position_weight    = 1e-3;
    double _vp_orientation_weight = 0;

    const ViaPointContainer* _via_points = nullptr;
    std::vector<std::pair<std::vector<const teb_local_planner::PoseSE2*>, std::size_t>> _vp_association;
    // also store previous number of associated points to detect structure changes
};

}  // namespace mpc_local_planner

#endif  // MIN_TIME_VIA_POINTS_COSTS_H_
