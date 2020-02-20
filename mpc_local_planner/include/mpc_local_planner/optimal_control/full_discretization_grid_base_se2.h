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

#ifndef FULL_DISCRETIZATION_GRID_BASE_H_
#define FULL_DISCRETIZATION_GRID_BASE_H_

#include <corbo-optimal-control/structured_ocp/discretization_grids/discretization_grid_interface.h>

#include <corbo-optimization/hyper_graph/scalar_vertex.h>
#include <corbo-optimization/hyper_graph/vector_vertex.h>
#include <mpc_local_planner/optimal_control/vector_vertex_se2.h>

#include <teb_local_planner/pose_se2.h>

#include <corbo-numerics/finite_differences_collocation.h>

#include <memory>

namespace mpc_local_planner {

/**
 * @brief Full discretization grid specialization for SE2
 *
 * This class defines the basis for full discretization grids
 * similar to corbo::FullDiscretizationGridBase.
 * However, the main difference is, that this grid
 * requires the first three state components to be embedded in SE2.
 * Whereas the first two states are ordinary real numbers on (-inf, inf),
 * the third component must remain in [-pi, pi).
 *
 * Therefore, the minimum dimension state dimension is 3.
 * However, larger dimensions are allowed, but the related state components
 * are considered as standard real numbers on (-inf, inf).
 *
 * Note the full discretization grid currently supports only finite difference collocation
 * to discretize the continuous time dynamics.
 * Full discretization is also possible with multiple shooting and (expicit) numerical integration,
 * but for this purpose we need to specialize corbo::ShootingGridBase for SE2.
 *
 * @see corbo::FullDiscretizationGridBase corbo::DiscretizationGridInterface
 *      FiniteDifferencesGridSE2
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class FullDiscretizationGridBaseSE2 : public corbo::DiscretizationGridInterface
{
 public:
    using Ptr                          = std::shared_ptr<FullDiscretizationGridBaseSE2>;
    using UPtr                         = std::unique_ptr<FullDiscretizationGridBaseSE2>;
    using ReferenceTrajectoryInterface = corbo::ReferenceTrajectoryInterface;
    using SystemDynamicsInterface      = corbo::SystemDynamicsInterface;
    using NlpFunctions                 = corbo::NlpFunctions;
    using OptimizationEdgeSet          = corbo::OptimizationEdgeSet;
    using VertexInterface              = corbo::VertexInterface;
    using ScalarVertex                 = corbo::ScalarVertex;
    using VectorVertex                 = corbo::VectorVertex;
    using PartiallyFixedVectorVertex   = corbo::PartiallyFixedVectorVertex;
    using TimeSeries                   = corbo::TimeSeries;

    enum class CostIntegrationRule { LeftSum, TrapezoidalRule };

    FullDiscretizationGridBaseSE2()          = default;
    virtual ~FullDiscretizationGridBaseSE2() = default;

    //! Return a newly created shared instance of the implemented class
    corbo::DiscretizationGridInterface::Ptr getInstance() const override = 0;

    // implements interface method
    corbo::GridUpdateResult update(const Eigen::VectorXd& x0, ReferenceTrajectoryInterface& xref, ReferenceTrajectoryInterface& uref,
                                   NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics, bool new_run,
                                   const corbo::Time& t, ReferenceTrajectoryInterface* sref = nullptr, const Eigen::VectorXd* prev_u = nullptr,
                                   double prev_u_dt = 0, ReferenceTrajectoryInterface* xinit = nullptr,
                                   ReferenceTrajectoryInterface* uinit = nullptr) override;

    // implements interface method
    double getFirstDt() const override { return getDt(); }
    // implements interface method
    double getFinalTime() const override { return double(getN() - 1) * getDt(); }

    // implements interface method
    bool hasConstantControls() const override { return true; }
    // implements interface method
    bool hasSingleDt() const override { return true; }
    // implements interface method
    bool isTimeVariableGrid() const override { return !isDtFixedIntended(); }
    // implements interface method
    bool isUniformGrid() const override { return true; }
    // implements interface method
    bool providesStateTrajectory() const override { return true; }
    // implements interface method
    bool getFirstControlInput(Eigen::VectorXd& u0) override;

    //! Return state at time stamp k
    const Eigen::VectorXd& getState(int k) const
    {
        assert(k <= getN());
        if (k == _x_seq.size()) return _xf.values();
        return _x_seq[k].values();
    }

    // implements interface method
    void getStateAndControlTimeSeries(TimeSeries::Ptr x_sequence, TimeSeries::Ptr u_sequence, double t_max = corbo::CORBO_INF_DBL) const override;

    // implements interface method
    void clear() override;

    // implements interface method
    bool isEmpty() const override { return _x_seq.empty() || _u_seq.empty(); }
    // implements interface method
    virtual bool isValid() const { return (_x_seq.size() == _u_seq.size()); }

    // implements interface method
    void setN(int n, bool try_resample = true) override;
    // implements interface method
    void setInitialDt(double dt) override { setDtRef(dt); }
    // implements interface method
    double getInitialDt() const override { return getDtRef(); }
    // implements interface method
    int getInitialN() const override { return getNRef(); }

    //! get reference horizon length
    int getNRef() const { return _n_ref; }
    //! get current horizon length
    int getN() const override { return _x_seq.size() + 1; }
    //! set reference horizon length
    void setNRef(int n);
    //! set reference temporal resolution
    void setDtRef(double dt) { _dt_ref = dt; }
    //! get current reference temporal resolution
    double getDtRef() const { return _dt_ref; }
    //! get current temporal resolution
    double getDt() const { return _dt.value(); }
    //! activate or deactive warmstart
    void setWarmStart(bool active) { _warm_start = active; }
    //! Set individual components of the final state to fixed or unfixed
    void setXfFixed(const Eigen::Matrix<bool, -1, 1>& xf_fixed)
    {
        _xf_fixed = xf_fixed;
        setModified(true);
    }

    //! Set finite differences collocation method
    void setFiniteDifferencesCollocationMethod(corbo::FiniteDifferencesCollocationInterface::Ptr fd_eval) { _fd_eval = fd_eval; }
    //! Set cost integration rule
    void setCostIntegrationRule(CostIntegrationRule integration) { _cost_integration = integration; }

    /**
     * @brief Find the closest pose (first part of the state vector) on the grid w.r.t. to a provided reference point.
     *
     * @param[in] x_ref       X-position of the reference point
     * @param[in] y_ref       Y-position of the reference point
     * @param[in] begin_idx   Start search at this time stamp
     * @param[out] distance   [optional] the resulting minimum distance
     * @return Index to the closest state/pose on the grid
     */
    int findClosestPose(double x_ref, double y_ref, int start_idx = 0, double* distance = nullptr) const;

    // implements interface method
    std::vector<VertexInterface*>& getActiveVertices() override { return _active_vertices; }
    // implements interface method
    void getVertices(std::vector<VertexInterface*>& vertices) override;

 protected:
    virtual void initializeSequences(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun);
    virtual void initializeSequences(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf, ReferenceTrajectoryInterface& xref,
                                     ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun);
    virtual void warmStartShifting(const Eigen::VectorXd& x0);

    virtual bool adaptGrid(bool new_run, NlpFunctions& nlp_fun) { return false; }  // A subclass might add a grid adaptation, returns true if adapted

    int findNearestState(const Eigen::VectorXd& x0);

    void updateBounds(const NlpFunctions& nlp_fun);

    virtual void resampleTrajectory(int n_new);

    bool checkAndInitializeXfFixedFlags(int dim_x);

    virtual void createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics) = 0;

    virtual bool isDtFixedIntended() const { return true; }
    virtual bool isMovingHorizonWarmStartActive() const { return _warm_start; }
    virtual bool isGridAdaptActive() const { return false; }

    void computeActiveVertices() override;

    corbo::FiniteDifferencesCollocationInterface::Ptr _fd_eval = std::make_shared<corbo::CrankNicolsonDiffCollocation>();

    std::vector<VectorVertexSE2> _x_seq;
    std::vector<VectorVertex> _u_seq;
    PartiallyFixedVectorVertexSE2 _xf;
    std::vector<VertexInterface*> _active_vertices;

    const NlpFunctions* _nlp_fun = nullptr;  // cache -> for bounds

    int _n_ref     = 11;
    int _n_adapt   = 0;  // if adaption is on and warmstart off, we might use this n instead of n_ref (only if n_adapt > 0)
    double _dt_ref = 0.1;
    ScalarVertex _dt;  // we need a ScalarVertex to use the helper methods in stage_functions.cpp
    bool _warm_start = false;
    bool _first_run  = true;

    // might be required if the last dt should be fixed or if dt is not fixed
    Eigen::Matrix<bool, -1, 1> _xf_fixed;
    double _dt_lb = 0;
    double _dt_ub = corbo::CORBO_INF_DBL;

    CostIntegrationRule _cost_integration;
};

}  // namespace mpc_local_planner

#endif  // FULL_DISCRETIZATION_GRID_BASE_H_
