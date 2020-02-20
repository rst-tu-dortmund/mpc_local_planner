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

#include <mpc_local_planner/optimal_control/finite_differences_grid_se2.h>

#include <corbo-optimal-control/structured_ocp/edges/finite_differences_collocation_edges.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace mpc_local_planner {

void FiniteDifferencesGridSE2::createEdges(NlpFunctions& nlp_fun, OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics)
{
    assert(isValid());

    // clear edges first
    // TODO(roesmann): we could implement a more efficient strategy without recreating the whole edgeset everytime
    edges.clear();

    int n = getN();

    std::vector<BaseEdge::Ptr> cost_terms, eq_terms, ineq_terms;
    for (int k = 0; k < n - 1; ++k)
    {
        VectorVertexSE2& x_next = (k < n - 2) ? _x_seq[k + 1] : _xf;
        VectorVertex& u_prev    = (k > 0) ? _u_seq[k - 1] : _u_prev;
        ScalarVertex& dt_prev   = (k > 0) ? _dt : _u_prev_dt;

        cost_terms.clear();
        eq_terms.clear();
        ineq_terms.clear();
        nlp_fun.getNonIntegralStageFunctionEdges(k, _x_seq[k], _u_seq[k], _dt, u_prev, dt_prev, cost_terms, eq_terms, ineq_terms);
        for (BaseEdge::Ptr& edge : cost_terms) edges.addObjectiveEdge(edge);
        for (BaseEdge::Ptr& edge : eq_terms) edges.addEqualityEdge(edge);
        for (BaseEdge::Ptr& edge : ineq_terms) edges.addInequalityEdge(edge);

        if (nlp_fun.stage_cost && nlp_fun.stage_cost->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                corbo::TrapezoidalIntegralCostEdge::Ptr edge =
                    std::make_shared<corbo::TrapezoidalIntegralCostEdge>(_x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_cost, k);
                edges.addObjectiveEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                corbo::LeftSumCostEdge::Ptr edge = std::make_shared<corbo::LeftSumCostEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_cost, k);
                edges.addObjectiveEdge(edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }

        if (nlp_fun.stage_equalities && nlp_fun.stage_equalities->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                corbo::TrapezoidalIntegralEqualityDynamicsEdge::Ptr edge = std::make_shared<corbo::TrapezoidalIntegralEqualityDynamicsEdge>(
                    dynamics, _x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_equalities, k);
                edge->setFiniteDifferencesCollocationMethod(_fd_eval);
                edges.addEqualityEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                corbo::LeftSumEqualityEdge::Ptr edge =
                    std::make_shared<corbo::LeftSumEqualityEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_equalities, k);
                edges.addEqualityEdge(edge);

                // system dynamics edge
                corbo::FDCollocationEdge::Ptr sys_edge = std::make_shared<corbo::FDCollocationEdge>(dynamics, _x_seq[k], _u_seq[k], x_next, _dt);
                sys_edge->setFiniteDifferencesCollocationMethod(_fd_eval);
                edges.addEqualityEdge(sys_edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }
        else
        {
            // just the system dynamics edge
            corbo::FDCollocationEdge::Ptr edge = std::make_shared<corbo::FDCollocationEdge>(dynamics, _x_seq[k], _u_seq[k], x_next, _dt);
            edge->setFiniteDifferencesCollocationMethod(_fd_eval);
            edges.addEqualityEdge(edge);
        }

        if (nlp_fun.stage_inequalities && nlp_fun.stage_inequalities->hasIntegralTerms(k))
        {
            if (_cost_integration == CostIntegrationRule::TrapezoidalRule)
            {
                corbo::TrapezoidalIntegralInequalityEdge::Ptr edge =
                    std::make_shared<corbo::TrapezoidalIntegralInequalityEdge>(_x_seq[k], _u_seq[k], x_next, _dt, nlp_fun.stage_inequalities, k);
                edges.addInequalityEdge(edge);
            }
            else if (_cost_integration == CostIntegrationRule::LeftSum)
            {
                corbo::LeftSumInequalityEdge::Ptr edge =
                    std::make_shared<corbo::LeftSumInequalityEdge>(_x_seq[k], _u_seq[k], _dt, nlp_fun.stage_inequalities, k);
                edges.addInequalityEdge(edge);
            }
            else
                PRINT_ERROR_NAMED("Cost integration rule not implemented");
        }
    }

    // check if we have a separate unfixed final state
    if (!_xf.isFixed())
    {
        // set final state cost
        BaseEdge::Ptr cost_edge = nlp_fun.getFinalStateCostEdge(n - 1, _xf);
        if (cost_edge) edges.addObjectiveEdge(cost_edge);

        // set final state constraint
        BaseEdge::Ptr constr_edge = nlp_fun.getFinalStateConstraintEdge(n - 1, _xf);
        if (constr_edge)
        {
            if (nlp_fun.final_stage_constraints->isEqualityConstraint())
                edges.addEqualityEdge(constr_edge);
            else
                edges.addInequalityEdge(constr_edge);
        }
    }

    // add control deviation edges for last control
    cost_terms.clear();
    eq_terms.clear();
    ineq_terms.clear();
    nlp_fun.getFinalControlDeviationEdges(n, _u_ref, _u_seq.back(), _dt, cost_terms, eq_terms, ineq_terms);
    for (BaseEdge::Ptr& edge : cost_terms) edges.addObjectiveEdge(edge);
    for (BaseEdge::Ptr& edge : eq_terms) edges.addEqualityEdge(edge);
    for (BaseEdge::Ptr& edge : ineq_terms) edges.addInequalityEdge(edge);
}

}  // namespace mpc_local_planner
