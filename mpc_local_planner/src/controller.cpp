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

#include <mpc_local_planner/controller.h>

#include <corbo-optimal-control/functions/hybrid_cost.h>
#include <corbo-optimal-control/functions/minimum_time.h>
#include <corbo-optimal-control/functions/quadratic_control_cost.h>
#include <corbo-optimal-control/structured_ocp/discretization_grids/finite_differences_variable_grid.h>
#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <corbo-optimization/hyper_graph/hyper_graph_optimization_problem_edge_based.h>
#include <corbo-optimization/solver/levenberg_marquardt_sparse.h>
#include <corbo-optimization/solver/nlp_solver_ipopt.h>
#include <corbo-optimization/solver/qp_solver_osqp.h>
#include <mpc_local_planner/optimal_control/fd_collocation_se2.h>
#include <mpc_local_planner/optimal_control/final_state_conditions_se2.h>
#include <mpc_local_planner/optimal_control/finite_differences_variable_grid_se2.h>
#include <mpc_local_planner/optimal_control/min_time_via_points_cost.h>
#include <mpc_local_planner/optimal_control/quadratic_cost_se2.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>
#include <mpc_local_planner/systems/kinematic_bicycle_model.h>
#include <mpc_local_planner/systems/simple_car.h>
#include <mpc_local_planner/systems/unicycle_robot.h>
#include <mpc_local_planner/utils/time_series_se2.h>

#include <mpc_local_planner_msgs/msg/optimal_control_result.hpp>
#include <mpc_local_planner_msgs/msg/state_feedback.hpp>

#include <teb_local_planner/misc.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <tf2/utils.h>

#include <memory>
#include <mutex>

namespace mpc_local_planner {

bool MpcController::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, std::string name, const teb_local_planner::ObstContainer& obstacles,
                           teb_local_planner::RobotFootprintModelPtr robot_model, const std::vector<teb_local_planner::PoseSE2>& via_points, MpcConfig* params, Publisher* publisher)
{
    nh_ = nh;
    name_ = name;
    params_ = params;
    publisher_ = publisher;

    _dynamics = configureRobotDynamics(nh_);
    if (!_dynamics) return false;  // we may need state and control dimensions to check other parameters

    _grid   = configureGrid(nh_);
    _solver = configureSolver(nh_);

    _structured_ocp = configureOcp(nh_, obstacles, robot_model, via_points);
    _ocp            = _structured_ocp;  // copy pointer also to parent member

    int outer_ocp_iterations = 1;
    setNumOcpIterations(outer_ocp_iterations);

    // custom feedback:
    _x_feedback_sub = nh_->create_subscription<mpc_local_planner_msgs::msg::StateFeedback>(
      "state_feedback",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MpcController::stateFeedbackCallback, this, std::placeholders::_1));

    setAutoUpdatePreviousControl(false);  // we want to update the previous control value manually

    if (_ocp->initialize())
        RCLCPP_INFO(logger_,"OCP initialized.");
    else
    {
        RCLCPP_ERROR(logger_,"OCP initialization failed");
        return false;
    }
    return _grid && _dynamics && _solver && _structured_ocp;
}

bool MpcController::step(const MpcController::PoseSE2& start, const MpcController::PoseSE2& goal, const geometry_msgs::msg::Twist& vel, double dt, rclcpp::Time t,
                      corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
{
    std::vector<geometry_msgs::msg::PoseStamped> initial_plan(2);
    start.toPoseMsg(initial_plan.front().pose);
    goal.toPoseMsg(initial_plan.back().pose);
    return step(initial_plan, vel, dt, t, u_seq, x_seq);
}

bool MpcController::step(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist& vel, double dt, rclcpp::Time t,
                      corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
{
    if (!_dynamics || !_grid || !_structured_ocp)
    {
        RCLCPP_ERROR(logger_, "Controller must be configured before invoking step().");
        return false;
    }
    if (initial_plan.size() < 2)
    {
        RCLCPP_ERROR(logger_, "Controller::step(): initial plan must contain at least two poses.");
        return false;
    }

    PoseSE2 start(initial_plan.front().pose);
    PoseSE2 goal(initial_plan.back().pose);

    Eigen::VectorXd xf(_dynamics->getStateDimension());
    _dynamics->getSteadyStateFromPoseSE2(goal, xf);

    // retrieve or estimate current state
    Eigen::VectorXd x(_dynamics->getStateDimension());
    // check for new measurements
    bool new_x = false;
    {
        std::lock_guard<std::mutex> lock(_x_feedback_mutex);
        new_x = _recent_x_feedback.size() > 0 && (t - _recent_x_time).seconds() < 2.0 * dt;
        if (new_x) x = _recent_x_feedback;
    }
    if (!new_x && (!_x_ts || _x_ts->isEmpty() || !_x_ts->getValuesInterpolate(dt, x)))  // predict with previous state sequence
    {
        _dynamics->getSteadyStateFromPoseSE2(start, x);  // otherwise initialize steady state
    }
    if (!new_x || !params_->controller.prefer_x_feedback)
    {
        // Merge state feedback with odometry feedback if desired.
        // Note, some models like unicycle overwrite the full state by odom feedback unless _prefer_x_measurement is set to true.
        _dynamics->mergeStateFeedbackAndOdomFeedback(start, vel, x);
    }

    // now check goal
    if (params_->controller.force_reinit_num_steps > 0 && _ocp_seq % params_->controller.force_reinit_num_steps == 0) _grid->clear();
    if (!_grid->isEmpty() && ((goal.position() - _last_goal.position()).norm() > params_->controller.force_reinit_new_goal_dist ||
                              std::abs(normalize_theta(goal.theta() - _last_goal.theta())) > params_->controller.force_reinit_new_goal_angular))
    {
        // goal pose diverges a lot from the previous one, so force reinit
        _grid->clear();
    }
    if (_grid->isEmpty())
    {
        // generate custom initialization based on initial_plan
        // check if the goal is behind the start pose (w.r.t. start orientation)
        bool backward = params_->controller.allow_init_with_backwards_motion && (goal.position() - start.position()).dot(start.orientationUnitVec()) < 0;
        generateInitialStateTrajectory(x, xf, initial_plan, backward);
    }
    corbo::Time time(t.seconds());
    _x_seq_init.setTimeFromStart(time);

    corbo::StaticReference xref(xf);  // currently, we only support point-to-point transitions in ros
    corbo::ZeroReference uref(_dynamics->getInputDimension());

    _ocp_successful = PredictiveController::step(x, xref, uref, corbo::Duration(dt), time, u_seq, x_seq, nullptr, nullptr, &_x_seq_init);
    // publish results if desired
    if (params_->controller.publish_ocp_results) publishOptimalControlResult();  // TODO(roesmann): we could also pass time t from above
    RCLCPP_INFO_EXPRESSION(logger_, params_->controller.print_cpu_time, "Cpu time: %f ms.", _statistics.step_time.toSec() * 1000.0);
    ++_ocp_seq;
    _last_goal = goal;
    return _ocp_successful;
}

void MpcController::stateFeedbackCallback(const mpc_local_planner_msgs::msg::StateFeedback::ConstSharedPtr msg)
{
    if (!_dynamics) return;

    if ((int)msg->state.size() != _dynamics->getStateDimension())
    {
        RCLCPP_ERROR_STREAM(logger_, "stateFeedbackCallback(): state feedback dimension does not match robot state dimension: "
                         << msg->state.size() << " != " << _dynamics->getStateDimension());
        return;
    }

    std::lock_guard<std::mutex> lock(_x_feedback_mutex);
    _recent_x_time     = msg->header.stamp;
    _recent_x_feedback = Eigen::Map<const Eigen::VectorXd>(msg->state.data(), (int)msg->state.size());
}

void MpcController::publishOptimalControlResult()
{
    if (!_dynamics) return;
    mpc_local_planner_msgs::msg::OptimalControlResult msg;
    msg.header.stamp           = nh_->now();
//    msg.header.seq             = static_cast<unsigned int>(_ocp_seq); TODO: do we need this?
    msg.dim_states             = _dynamics->getStateDimension();
    msg.dim_controls           = _dynamics->getInputDimension();
    msg.optimal_solution_found = _ocp_successful;
    msg.cpu_time               = _statistics.step_time.toSec();

    if (_x_ts && !_x_ts->isEmpty())
    {
        msg.time_states = _x_ts->getTime();
        msg.states      = _x_ts->getValues();
    }

    if (_u_ts && !_u_ts->isEmpty())
    {
        msg.time_controls = _u_ts->getTime();
        msg.controls      = _u_ts->getValues();
    }

    publisher_->publishOptimalControlResult(msg);
}

void MpcController::reset() { PredictiveController::reset(); }

corbo::DiscretizationGridInterface::Ptr MpcController::configureGrid(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh)
{
    if (!_dynamics) return {};

    if (params_->grid.type == "fd_grid")
    {
        FiniteDifferencesGridSE2::Ptr grid;

        if (params_->grid.variable_grid.enable)
        {
            FiniteDifferencesVariableGridSE2::Ptr var_grid = std::make_shared<FiniteDifferencesVariableGridSE2>();

            var_grid->setDtBounds(params_->grid.variable_grid.min_dt, params_->grid.variable_grid.max_dt);

            if (params_->grid.variable_grid.grid_adaptation.enable)
            {
                var_grid->setGridAdaptTimeBasedSingleStep(params_->grid.variable_grid.grid_adaptation.max_grid_size, params_->grid.variable_grid.grid_adaptation.dt_hyst_ratio, true);

                int min_grid_size = 2;

                var_grid->setNmin(min_grid_size);
            }
            else
            {
                var_grid->disableGridAdaptation();
            }
            grid = var_grid;
        }
        else
        {
            grid = std::make_shared<FiniteDifferencesGridSE2>();
        }
        // common grid parameters

        grid->setNRef(params_->grid.grid_size_ref);

        grid->setDtRef(params_->grid.dt_ref);

        if ((int)params_->grid.xf_fixed.size() != _dynamics->getStateDimension())
        {
            RCLCPP_ERROR_STREAM(logger_, "Array size of `xf_fixed` does not match robot state dimension(): " << params_->grid.xf_fixed.size()
                                                                                                 << " != " << _dynamics->getStateDimension());
            return {};
        }
        Eigen::Matrix<bool, -1, 1> xf_fixed_eigen(params_->grid.xf_fixed.size());  // we cannot use Eigen::Map as vector<bool> does not provide raw access
        for (int i = 0; i < (int)params_->grid.xf_fixed.size(); ++i) xf_fixed_eigen[i] = params_->grid.xf_fixed[i];
        grid->setXfFixed(xf_fixed_eigen);

        grid->setWarmStart(params_->grid.warm_start);

        if (params_->grid.collocation_method == "forward_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<ForwardDiffCollocationSE2>());
        }
        else if (params_->grid.collocation_method == "midpoint_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<MidpointDiffCollocationSE2>());
        }
        else if (params_->grid.collocation_method == "crank_nicolson_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<CrankNicolsonDiffCollocationSE2>());
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Unknown collocation method '" << params_->grid.collocation_method << "' specified. Falling back to default...");
        }



        if (params_->grid.cost_integration_method == "left_sum")
        {
            grid->setCostIntegrationRule(FullDiscretizationGridBaseSE2::CostIntegrationRule::LeftSum);
        }
        else if (params_->grid.cost_integration_method == "trapezoidal_rule")
        {
            grid->setCostIntegrationRule(FullDiscretizationGridBaseSE2::CostIntegrationRule::TrapezoidalRule);
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Unknown cost integration method '" << params_->grid.cost_integration_method << "' specified. Falling back to default...");
        }

        return std::move(grid);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown grid type '" << params_->grid.type << "' specified.");
    }

    return {};
}

RobotDynamicsInterface::Ptr MpcController::configureRobotDynamics(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh)
{
    if (params_->robot.type == "unicycle")
    {
        return std::make_shared<UnicycleModel>();
    }
    else if (params_->robot.type == "simple_car")
    {
        if (params_->robot.simple_car.front_wheel_driving)
            return std::make_shared<SimpleCarFrontWheelDrivingModel>(params_->robot.simple_car.wheelbase);
        else
            return std::make_shared<SimpleCarModel>(params_->robot.simple_car.wheelbase);
    }
    else if (params_->robot.type == "kinematic_bicycle_vel_input")
    {
        return std::make_shared<KinematicBicycleModelVelocityInput>(params_->robot.kinematic_bicycle_vel_input.length_rear, params_->robot.kinematic_bicycle_vel_input.length_front);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown robot type '" << params_->robot.type << "' specified.");
    }

    return {};
}

corbo::NlpSolverInterface::Ptr MpcController::configureSolver(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh)
{


    if (params_->solver.type == "ipopt")
    {
        corbo::SolverIpopt::Ptr solver = std::make_shared<corbo::SolverIpopt>();
        solver->initialize();  // requried for setting parameters afterward

        solver->setIterations(params_->solver.ipopt.iterations);

        solver->setMaxCpuTime(params_->solver.ipopt.max_cpu_time);

        // now check for additional ipopt options
        for (const auto& item : params_->solver.ipopt.numeric_options)
        {
            if (!solver->setIpoptOptionNumeric(item.first, item.second)) RCLCPP_WARN_STREAM(logger_, "Ipopt option " << item.first << " could not be set.");
        }

        for (const auto& item : params_->solver.ipopt.string_options)
        {
            if (!solver->setIpoptOptionString(item.first, item.second)) RCLCPP_WARN_STREAM(logger_, "Ipopt option " << item.first << " could not be set.");
        }

        for (const auto& item : params_->solver.ipopt.integer_options)
        {
            if (!solver->setIpoptOptionInt(item.first, item.second)) RCLCPP_WARN_STREAM(logger_, "Ipopt option " << item.first << " could not be set.");
        }

        return std::move(solver);
    }
    //    else if (params_->solver.type == "sqp")
    //    {
    //        corbo::SolverSQP::Ptr solver = std::make_shared<corbo::SolverSQP>();
    //        solver->setUseObjectiveHessian(true);
    //        // solver->setQpZeroWarmstart(false);
    //        solver->setVerbose(true);
    //        corbo::SolverOsqp::Ptr qp_solver      = std::make_shared<corbo::SolverOsqp>();
    //        qp_solver->getOsqpSettings()->verbose = 1;
    //        solver->setQpSolver(qp_solver);
    //        corbo::LineSearchL1::Ptr ls = std::make_shared<corbo::LineSearchL1>();
    //        ls->setVerbose(true);
    //        ls->setEta(1e-6);
    //        solver->setLineSearchAlgorithm(ls);

    //        return std::move(solver);
    //    }
    else if (params_->solver.type == "lsq_lm")
    {
        corbo::LevenbergMarquardtSparse::Ptr solver = std::make_shared<corbo::LevenbergMarquardtSparse>();

        solver->setIterations(params_->solver.lsq_lm.iterations);

        solver->setPenaltyWeights( params_->solver.lsq_lm.weight_init_eq,  params_->solver.lsq_lm.weight_init_ineq,  params_->solver.lsq_lm.weight_init_bounds);


        solver->setWeightAdapation( params_->solver.lsq_lm.weight_adapt_factor_eq,  params_->solver.lsq_lm.weight_adapt_factor_ineq,  params_->solver.lsq_lm.weight_adapt_factor_bounds,  params_->solver.lsq_lm.weight_adapt_max_eq,
                                    params_->solver.lsq_lm.weight_adapt_max_ineq,  params_->solver.lsq_lm.weight_adapt_max_bounds);

        return std::move(solver);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown solver type '" << params_->solver.type << "' specified.");
    }

    return {};
}

corbo::StructuredOptimalControlProblem::Ptr MpcController::configureOcp(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const teb_local_planner::ObstContainer& obstacles,
                                                                     teb_local_planner::RobotFootprintModelPtr robot_model,
                                                                     const std::vector<teb_local_planner::PoseSE2>& via_points)
{
    corbo::BaseHyperGraphOptimizationProblem::Ptr hg = std::make_shared<corbo::HyperGraphOptimizationProblemEdgeBased>();

    corbo::StructuredOptimalControlProblem::Ptr ocp = std::make_shared<corbo::StructuredOptimalControlProblem>(_grid, _dynamics, hg, _solver);

    const int x_dim = _dynamics->getStateDimension();
    const int u_dim = _dynamics->getInputDimension();

    if (params_->robot.type == "unicycle")
    {

        if (params_->robot.unicycle.max_vel_x_backwards < 0)
        {
            RCLCPP_WARN(logger_,"max_vel_x_backwards must be >= 0");
        }
        // ocp->setBounds(Eigen::Vector3d(-corbo::CORBO_INF_DBL, -corbo::CORBO_INF_DBL, -corbo::CORBO_INF_DBL),
        //               Eigen::Vector3d(corbo::CORBO_INF_DBL, corbo::CORBO_INF_DBL, corbo::CORBO_INF_DBL),
        //               Eigen::Vector2d(-max_vel_x_backwards, -max_vel_theta), Eigen::Vector2d(max_vel_x, max_vel_theta));
        ocp->setControlBounds(Eigen::Vector2d(-params_->robot.unicycle.max_vel_x_backwards, -params_->robot.unicycle.max_vel_theta), Eigen::Vector2d(params_->robot.unicycle.max_vel_x, params_->robot.unicycle.max_vel_theta));
    }
    else if (params_->robot.type == "simple_car")
    {

        if ( params_->robot.simple_car.max_vel_x_backwards < 0)
        {
            RCLCPP_WARN(logger_, "robot.simple_car.max_vel_x_backwards must be >= 0");
        }

        ocp->setControlBounds(Eigen::Vector2d(- params_->robot.simple_car.max_vel_x_backwards, - params_->robot.simple_car.max_steering_angle), Eigen::Vector2d( params_->robot.simple_car.max_vel_x,  params_->robot.simple_car.max_steering_angle));
    }
    else if (params_->robot.type == "kinematic_bicycle_vel_input")
    {

        if ( params_->robot.kinematic_bicycle_vel_input.max_vel_x_backwards < 0)
        {
            RCLCPP_WARN(logger_, "robot.kinematic_bicycle_vel_input..max_vel_x_backwards must be >= 0");
        }

        ocp->setControlBounds(Eigen::Vector2d(- params_->robot.kinematic_bicycle_vel_input.max_vel_x_backwards, - params_->robot.kinematic_bicycle_vel_input.max_steering_angle), Eigen::Vector2d( params_->robot.kinematic_bicycle_vel_input.max_vel_x,  params_->robot.kinematic_bicycle_vel_input.max_steering_angle));
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Cannot configure OCP for unknown robot type " << params_->robot.type << ".");
        return {};
    }
    bool lsq_solver = _solver->isLsqSolver();

    if (params_->planning.objective.type == "minimum_time")
    {
        ocp->setStageCost(std::make_shared<corbo::MinimumTime>(lsq_solver));
    }
    else if (params_->planning.objective.type == "quadratic_form")
    {
        Eigen::MatrixXd Q;
        if (params_->planning.objective.quadratic_form.state_weights.size() == x_dim)
        {
            Q = Eigen::Map<Eigen::VectorXd>(params_->planning.objective.quadratic_form.state_weights.data(), x_dim).asDiagonal();
        }
        else if (params_->planning.objective.quadratic_form.state_weights.size() == x_dim * x_dim)
        {
            Q = Eigen::Map<Eigen::MatrixXd>(params_->planning.objective.quadratic_form.state_weights.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "State weights dimension invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }

        Eigen::MatrixXd R;
        if (params_->planning.objective.quadratic_form.control_weights.size() == u_dim)
        {
            R = Eigen::Map<Eigen::VectorXd>(params_->planning.objective.quadratic_form.control_weights.data(), u_dim).asDiagonal();
        }
        else if (params_->planning.objective.quadratic_form.control_weights.size() == u_dim * u_dim)
        {
            R = Eigen::Map<Eigen::MatrixXd>(params_->planning.objective.quadratic_form.control_weights.data(), u_dim, u_dim);  // Eigens default is column major
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Control weights dimension invalid. Must be either " << u_dim << " x 1 or " << u_dim << " x " << u_dim << ".");
            return {};
        }

        bool q_zero = Q.isZero();
        bool r_zero = R.isZero();
        if (!q_zero && !r_zero)
        {
            RCLCPP_ERROR_EXPRESSION(logger_, params_->planning.objective.quadratic_form.hybrid_cost_minimum_time,
                             "Hybrid minimum time and quadratic form cost is currently only supported for non-zero control weights only. Falling "
                             "back to quadratic form.");
            ocp->setStageCost(std::make_shared<QuadraticFormCostSE2>(Q, R, params_->planning.objective.quadratic_form.integral_form, lsq_solver));
        }
        else if (!q_zero && r_zero)
        {
            RCLCPP_ERROR_EXPRESSION(logger_, params_->planning.objective.quadratic_form.hybrid_cost_minimum_time,
                             "Hybrid minimum time and quadratic form cost is currently only supported for non-zero control weights only. Falling "
                             "back to only quadratic state cost.");
            ocp->setStageCost(std::make_shared<QuadraticStateCostSE2>(Q, params_->planning.objective.quadratic_form.integral_form, lsq_solver));
        }
        else if (q_zero && !r_zero)
        {
            if (params_->planning.objective.quadratic_form.hybrid_cost_minimum_time)
            {
                ocp->setStageCost(std::make_shared<corbo::MinTimeQuadraticControls>(R, params_->planning.objective.quadratic_form.integral_form, lsq_solver));
            }
            else
            {
                ocp->setStageCost(std::make_shared<corbo::QuadraticControlCost>(R, params_->planning.objective.quadratic_form.integral_form, lsq_solver));
            }
        }
    }
    else if (params_->planning.objective.type == "minimum_time_via_points")
    {

        ocp->setStageCost(std::make_shared<MinTimeViaPointsCost>(via_points, params_->planning.objective.minimum_time_via_points.position_weight, params_->planning.objective.minimum_time_via_points.orientation_weight, params_->planning.objective.minimum_time_via_points.via_points_ordered));
        // TODO(roesmann): lsq version
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown objective type '" << params_->planning.objective.type << "' specified ('planning.objective.type').");
        return {};
    }



    if (params_->planning.terminal_cost.type == "none")
    {
        ocp->setFinalStageCost({});
    }
    else if (params_->planning.terminal_cost.type == "quadratic")
    {
        Eigen::MatrixXd Qf;
        if (params_->planning.terminal_cost.quadratic.state_weights.size() == x_dim)
        {
            Qf = Eigen::Map<Eigen::VectorXd>(params_->planning.terminal_cost.quadratic.state_weights.data(), x_dim).asDiagonal();
        }
        else if (params_->planning.terminal_cost.quadratic.state_weights.size() == x_dim * x_dim)
        {
            Qf = Eigen::Map<Eigen::MatrixXd>(params_->planning.terminal_cost.quadratic.state_weights.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Final state weights dimension invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }
        ocp->setFinalStageCost(std::make_shared<QuadraticFinalStateCostSE2>(Qf, lsq_solver));
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown terminal_cost type '" << params_->planning.terminal_cost.type << "' specified ('planning.terminal_cost.type').");
        return {};
    }



    if (params_->planning.terminal_constraint.type == "none")
    {
        ocp->setFinalStageConstraint({});
    }
    else if (params_->planning.terminal_constraint.type == "l2_ball")
    {

        Eigen::MatrixXd S;
        if (params_->planning.terminal_constraint.l2ball.weight_matrix.size() == x_dim)
        {
            S = Eigen::Map<Eigen::VectorXd>(params_->planning.terminal_constraint.l2ball.weight_matrix.data(), x_dim).asDiagonal();
        }
        else if (params_->planning.terminal_constraint.l2ball.weight_matrix.size() == x_dim * x_dim)
        {
            S = Eigen::Map<Eigen::MatrixXd>(params_->planning.terminal_constraint.l2ball.weight_matrix.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "l2-ball weight_matrix dimensions invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }

        ocp->setFinalStageConstraint(std::make_shared<TerminalBallSE2>(S, params_->planning.terminal_constraint.l2ball.radius));
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Unknown terminal_constraint type '" << params_->planning.terminal_constraint.type << "' specified ('planning.terminal_constraint.type').");
        return {};
    }
    _inequality_constraint = std::make_shared<StageInequalitySE2>();
    _inequality_constraint->setObstacleVector(obstacles);
    _inequality_constraint->setRobotFootprintModel(robot_model);

    // configure collision avoidance
    _inequality_constraint->setMinimumDistance(params_->collision_avoidance.min_obstacle_dist);
    _inequality_constraint->setEnableDynamicObstacles(params_->collision_avoidance.enable_dynamic_obstacles);
    _inequality_constraint->setObstacleFilterParameters(params_->collision_avoidance.force_inclusion_dist, params_->collision_avoidance.cutoff_dist);

    // configure control deviation bounds

    if (params_->robot.type == "unicycle")
    {
        if (params_->robot.unicycle.dec_lim_x < 0)
        {
            RCLCPP_WARN(logger_,"dec_lim_x must be >= 0");
            params_->robot.unicycle.dec_lim_x *= -1;
        }


        if (params_->robot.unicycle.acc_lim_x <= 0) params_->robot.unicycle.acc_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.unicycle.dec_lim_x <= 0) params_->robot.unicycle.dec_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.unicycle.acc_lim_theta <= 0) params_->robot.unicycle.acc_lim_theta = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-params_->robot.unicycle.dec_lim_x, -params_->robot.unicycle.acc_lim_theta);
        Eigen::Vector2d ud_ub(params_->robot.unicycle.acc_lim_x, params_->robot.unicycle.acc_lim_theta);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else if (params_->robot.type == "simple_car")
    {
        if (params_->robot.simple_car.dec_lim_x < 0)
        {
            RCLCPP_WARN(logger_, "dec_lim_x must be >= 0");
            params_->robot.simple_car.dec_lim_x *= -1;
        }

        if (params_->robot.simple_car.acc_lim_x <= 0) params_->robot.simple_car.acc_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.simple_car.dec_lim_x <= 0) params_->robot.simple_car.dec_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.simple_car.max_steering_rate <= 0) params_->robot.simple_car.max_steering_rate = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-params_->robot.simple_car.dec_lim_x, -params_->robot.simple_car.max_steering_rate);
        Eigen::Vector2d ud_ub(params_->robot.simple_car.acc_lim_x, params_->robot.simple_car.max_steering_rate);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else if (params_->robot.type == "kinematic_bicycle_vel_input")
    {
        if (params_->robot.kinematic_bicycle_vel_input.dec_lim_x < 0)
        {
            RCLCPP_WARN(logger_, "dec_lim_x must be >= 0");
            params_->robot.kinematic_bicycle_vel_input.dec_lim_x *= -1;
        }

        if (params_->robot.kinematic_bicycle_vel_input.acc_lim_x <= 0) params_->robot.kinematic_bicycle_vel_input.acc_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.kinematic_bicycle_vel_input.dec_lim_x <= 0) params_->robot.kinematic_bicycle_vel_input.dec_lim_x = corbo::CORBO_INF_DBL;
        if (params_->robot.kinematic_bicycle_vel_input.max_steering_rate <= 0) params_->robot.kinematic_bicycle_vel_input.max_steering_rate = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-params_->robot.kinematic_bicycle_vel_input.dec_lim_x, -params_->robot.kinematic_bicycle_vel_input.max_steering_rate);
        Eigen::Vector2d ud_ub(params_->robot.kinematic_bicycle_vel_input.acc_lim_x, params_->robot.kinematic_bicycle_vel_input.max_steering_rate);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else
    {
        RCLCPP_ERROR_STREAM(logger_, "Cannot configure control deviation bounds for unknown robot type " << params_->robot.type << ".");
        return {};
    }

    ocp->setStageInequalityConstraint(_inequality_constraint);

    return ocp;
}

bool MpcController::generateInitialStateTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf,
                                                const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, bool backward)
{
    if (initial_plan.size() < 2 || !_dynamics) return false;

    TimeSeriesSE2::Ptr ts = std::make_shared<TimeSeriesSE2>();

    int n_init = (int)initial_plan.size();
    int n_ref  = _grid->getInitialN();
    if (n_ref < 2)
    {
        RCLCPP_ERROR(logger_, "Controller::generateInitialStateTrajectory(): grid not properly initialized");
        return false;
    }
    ts->add(0.0, x0);

    double dt_ref = _grid->getInitialDt();
    double tf_ref = (double)(n_ref - 1) * dt_ref;

    Eigen::VectorXd x(_dynamics->getStateDimension());

    // we initialize by assuming equally distributed poses
    double dt_init = tf_ref / double(n_init - 1);

    double t = dt_init;
    for (int i = 1; i < n_init - 1; ++i)
    {
        // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
        double yaw;
        if (params_->controller.global_plan_overwrite_orientation)
        {
            double dx = initial_plan[i + 1].pose.position.x - initial_plan[i].pose.position.x;
            double dy = initial_plan[i + 1].pose.position.y - initial_plan[i].pose.position.y;
            yaw       = std::atan2(dy, dx);
            if (backward) normalize_theta(yaw + M_PI);
        }
        else
        {
            yaw = tf2::getYaw(initial_plan[i].pose.orientation);
        }
        PoseSE2 intermediate_pose(initial_plan[i].pose.position.x, initial_plan[i].pose.position.y, yaw);
        _dynamics->getSteadyStateFromPoseSE2(intermediate_pose, x);
        ts->add(t, x);
        t += dt_init;
    }

    ts->add(tf_ref, xf);

    _x_seq_init.setTrajectory(ts, corbo::TimeSeries::Interpolation::Linear);
    return true;
}

bool MpcController::isPoseTrajectoryFeasible(std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                          double inscribed_radius, double circumscribed_radius, double min_resolution_collision_check_angular,
                                          int look_ahead_idx)
{
    if (!_grid)
    {
        RCLCPP_ERROR(logger_, "Controller must be configured before invoking step().");
        return false;
    }
    if (_grid->getN() < 2) return false;

    // we currently require a full discretization grid as we want to have fast access to
    // individual states without requiring any new simulation.
    // Alternatively, other grids could be used in combination with method getStateAndControlTimeSeries()
    const FullDiscretizationGridBaseSE2* fd_grid = dynamic_cast<const FullDiscretizationGridBaseSE2*>(_grid.get());
    if (!fd_grid)
    {
        RCLCPP_ERROR(logger_,"isPoseTrajectoriyFeasible is currently only implemented for fd grids");
        return true;
    }

    if (look_ahead_idx < 0 || look_ahead_idx >= _grid->getN()) look_ahead_idx = _grid->getN() - 1;

    for (int i = 0; i <= look_ahead_idx; ++i)
    {
        PoseSE2 pose(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2]);
        geometry_msgs::msg::Pose2D pose2d;
        pose.toPoseMsg(pose2d);

        if (costmap_model->scorePose(pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec)) < 0)
        {
            return false;
        }
        // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
        // and interpolates in that case.
        // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
        if (i < look_ahead_idx)
        {
            double delta_rot           = normalize_theta(fd_grid->getState(i + 1)[2] - fd_grid->getState(i)[2]);
            Eigen::Vector2d delta_dist = fd_grid->getState(i + 1).head(2) - fd_grid->getState(i).head(2);
            if (std::abs(delta_rot) > min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
            {
                int n_additional_samples = std::max(std::ceil(std::abs(delta_rot) / min_resolution_collision_check_angular),
                                                    std::ceil(delta_dist.norm() / inscribed_radius)) -
                                           1;
                for (int step = 0; step < n_additional_samples; ++step)
                {
                    PoseSE2 intermediate_pose(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2]);
                    geometry_msgs::msg::Pose2D intermediate_pose2d;
                    intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                    intermediate_pose.theta()    = g2o::normalize_theta(intermediate_pose.theta() + delta_rot / (n_additional_samples + 1.0));
                    intermediate_pose.toPoseMsg(intermediate_pose2d);
                    if (costmap_model->scorePose(intermediate_pose2d, dwb_critics::getOrientedFootprint(intermediate_pose2d, footprint_spec)) < 0)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

}  // namespace mpc_local_planner
