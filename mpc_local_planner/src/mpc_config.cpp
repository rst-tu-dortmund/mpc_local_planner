#include "mpc_local_planner/mpc_config.h"

namespace mpc_local_planner {

void MpcConfig::declareParameters(const nav2_util::LifecycleNode::SharedPtr node, const std::string name)
{
  node->declare_parameter(name + "." + "odom_topic", odom_topic);
  node->declare_parameter(name + "." + "map_frame", map_frame);

  node->declare_parameter(name + "." + "costmap_converter_plugin", costmap_converter_plugin);
  node->declare_parameter(name + "." + "costmap_converter_rate", costmap_converter_rate);
  node->declare_parameter(name + "." + "costmap_converter_spin_thread", costmap_converter_spin_thread);

  // Collision Avoidance
  node->declare_parameter(name + "." + "collision_avoidance.collision_check_min_resolution_angular", collision_avoidance.collision_check_min_resolution_angular);
  node->declare_parameter(name + "." + "collision_avoidance.collision_check_no_poses", collision_avoidance.collision_check_no_poses);
  node->declare_parameter(name + "." + "collision_avoidance.costmap_obstacles_behind_robot_dist", collision_avoidance.costmap_obstacles_behind_robot_dist);
  node->declare_parameter(name + "." + "collision_avoidance.cutoff_dist", collision_avoidance.cutoff_dist);
  node->declare_parameter(name + "." + "collision_avoidance.enable_dynamic_obstacles", collision_avoidance.enable_dynamic_obstacles);
  node->declare_parameter(name + "." + "collision_avoidance.force_inclusion_dist", collision_avoidance.force_inclusion_dist);
  node->declare_parameter(name + "." + "collision_avoidance.include_costmap_obstacles", collision_avoidance.include_costmap_obstacles);
  node->declare_parameter(name + "." + "collision_avoidance.min_obstacle_dist", collision_avoidance.min_obstacle_dist);

  // Controller
  node->declare_parameter(name + "." + "controller.allow_init_with_backward_motion", controller.allow_init_with_backwards_motion);
  node->declare_parameter(name + "." + "controller.force_reinit_new_goal_angular", controller.force_reinit_new_goal_angular);
  node->declare_parameter(name + "." + "controller.force_reinit_new_goal_dist", controller.force_reinit_new_goal_dist);
  node->declare_parameter(name + "." + "controller.force_reinit_num_steps", controller.force_reinit_num_steps);
  node->declare_parameter(name + "." + "controller.global_plan_overwrite_orientation", controller.global_plan_overwrite_orientation);
  node->declare_parameter(name + "." + "controller.global_plan_prune_distance", controller.global_plan_prune_distance);
  node->declare_parameter(name + "." + "controller.global_plan_viapoint_sep", controller.global_plan_viapoint_sep);
  node->declare_parameter(name + "." + "controller.max_global_plan_lookahead_dist", controller.max_global_plan_lookahead_dist);
  node->declare_parameter(name + "." + "controller.outer_ocp_iterations", controller.outer_ocp_iterations);
  node->declare_parameter(name + "." + "controller.prefer_x_feedback", controller.prefer_x_feedback);
  node->declare_parameter(name + "." + "controller.print_cpu_time", controller.print_cpu_time);
  node->declare_parameter(name + "." + "controller.publish_ocp_results", controller.publish_ocp_results);
  node->declare_parameter(name + "." + "controller.xy_goal_tolerance", controller.xy_goal_tolerance);
  node->declare_parameter(name + "." + "controller.yaw_goal_tolerance", controller.yaw_goal_tolerance);

  // Footprint
  node->declare_parameter(name + "." + "footprint_model.is_footprint_dynamic", footprint_model.is_footprint_dynamic);
  node->declare_parameter(name + "." + "footprint_model.type", footprint_model.type);
  node->declare_parameter(name + "." + "footprint_model.radius", footprint_model.radius);
  node->declare_parameter(name + "." + "footprint_model.line_start", footprint_model.line_start);
  node->declare_parameter(name + "." + "footprint_model.line_end", footprint_model.line_end);
  node->declare_parameter(name + "." + "footprint_model.front_offset", footprint_model.front_offset);
  node->declare_parameter(name + "." + "footprint_model.front_radius", footprint_model.front_radius);
  node->declare_parameter(name + "." + "footprint_model.rear_offset", footprint_model.rear_offset);
  node->declare_parameter(name + "." + "footprint_model.rear_radius", footprint_model.rear_radius);
  node->declare_parameter(name + "." + "footprint_model.vertices");

  // Grid
  node->declare_parameter(name + "." + "grid.collocation_method", grid.collocation_method);
  node->declare_parameter(name + "." + "grid.cost_integration_method", grid.cost_integration_method);
  node->declare_parameter(name + "." + "grid.dt_ref", grid.dt_ref);
  node->declare_parameter(name + "." + "grid.grid_size_ref", grid.grid_size_ref);
  node->declare_parameter(name + "." + "grid.type", grid.type);
  node->declare_parameter(name + "." + "grid.warm_start", grid.warm_start);
  node->declare_parameter(name + "." + "grid.xf_fixed", grid.xf_fixed);

  node->declare_parameter(name + "." + "grid.variable_grid.enable", grid.variable_grid.enable);
  node->declare_parameter(name + "." + "grid.variable_grid.max_dt", grid.variable_grid.max_dt);
  node->declare_parameter(name + "." + "grid.variable_grid.min_dt", grid.variable_grid.min_dt);

  node->declare_parameter(name + "." + "grid.variable_grid.grid_adaptation.enable", grid.variable_grid.grid_adaptation.enable);
  node->declare_parameter(name + "." + "grid.variable_grid.grid_adaptation.dt_hyst_ratio", grid.variable_grid.grid_adaptation.dt_hyst_ratio);
  node->declare_parameter(name + "." + "grid.variable_grid.grid_adaptation.max_grid_size", grid.variable_grid.grid_adaptation.max_grid_size);
  node->declare_parameter(name + "." + "grid.variable_grid.grid_adaptation.min_grid_size", grid.variable_grid.grid_adaptation.min_grid_size);

  // Planning
  node->declare_parameter(name + "." + "planning.objective.type", planning.objective.type);

  node->declare_parameter(name + "." + "planning.objective.minimum_time_via_points.via_points_ordered", planning.objective.minimum_time_via_points.via_points_ordered);
  node->declare_parameter(name + "." + "planning.objective.minimum_time_via_points.position_weight", planning.objective.minimum_time_via_points.position_weight);
  node->declare_parameter(name + "." + "planning.objective.minimum_time_via_points.orientation_weight", planning.objective.minimum_time_via_points.orientation_weight);

  node->declare_parameter(name + "." + "planning.objective.quadratic_form.control_weights", planning.objective.quadratic_form.control_weights);
  node->declare_parameter(name + "." + "planning.objective.quadratic_form.hybrid_cost_minimum_time", planning.objective.quadratic_form.hybrid_cost_minimum_time);
  node->declare_parameter(name + "." + "planning.objective.quadratic_form.integral_form", planning.objective.quadratic_form.integral_form);
  node->declare_parameter(name + "." + "planning.objective.quadratic_form.state_weights", planning.objective.quadratic_form.state_weights);

  node->declare_parameter(name + "." + "planning.terminal_constraint.type", planning.terminal_constraint.type);

  node->declare_parameter(name + "." + "planning.terminal_constraint.l2_ball.radius", planning.terminal_constraint.l2ball.radius);
  node->declare_parameter(name + "." + "planning.terminal_constraint.l2_ball.weight_matrix", planning.terminal_constraint.l2ball.weight_matrix);

  node->declare_parameter(name + "." + "planning.terminal_cost.type", planning.terminal_cost.type);
  node->declare_parameter(name + "." + "planning.terminal_cost.quadratic.final_state_weights", planning.terminal_cost.quadratic.state_weights);

  // Robot
  node->declare_parameter(name + "." + "robot.type", robot.type);

  node->declare_parameter(name + "." + "robot.unicycle.acc_lim_theta", robot.unicycle.acc_lim_theta);
  node->declare_parameter(name + "." + "robot.unicycle.acc_lim_x", robot.unicycle.acc_lim_x);
  node->declare_parameter(name + "." + "robot.unicycle.dec_lim_x", robot.unicycle.dec_lim_x);
  node->declare_parameter(name + "." + "robot.unicycle.max_vel_theta", robot.unicycle.max_vel_theta);
  node->declare_parameter(name + "." + "robot.unicycle.max_vel_x", robot.unicycle.max_vel_x);
  node->declare_parameter(name + "." + "robot.unicycle.max_vel_x_backwards", robot.unicycle.max_vel_x_backwards);

  node->declare_parameter(name + "." + "robot.simple_car.acc_lim_x", robot.simple_car.acc_lim_x);
  node->declare_parameter(name + "." + "robot.simple_car.dec_lim_x", robot.simple_car.dec_lim_x);
  node->declare_parameter(name + "." + "robot.simple_car.front_wheel_driving", robot.simple_car.front_wheel_driving);
  node->declare_parameter(name + "." + "robot.simple_car.max_steering_angle", robot.simple_car.max_steering_angle);
  node->declare_parameter(name + "." + "robot.simple_car.max_steering_rate", robot.simple_car.max_steering_rate);
  node->declare_parameter(name + "." + "robot.simple_car.max_vel_x", robot.simple_car.max_vel_x);
  node->declare_parameter(name + "." + "robot.simple_car.max_vel_x_backwards", robot.simple_car.max_vel_x_backwards);
  node->declare_parameter(name + "." + "robot.simple_car.wheelbase", robot.simple_car.wheelbase);

  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.acc_lim_x", robot.kinematic_bicycle_vel_input.acc_lim_x);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.dec_lim_x", robot.kinematic_bicycle_vel_input.dec_lim_x);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.length_front", robot.kinematic_bicycle_vel_input.length_front);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.length_rear", robot.kinematic_bicycle_vel_input.length_rear);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.max_steering_angle", robot.kinematic_bicycle_vel_input.max_steering_angle);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.max_steering_rate", robot.kinematic_bicycle_vel_input.max_steering_rate);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.max_vel_x", robot.kinematic_bicycle_vel_input.max_vel_x);
  node->declare_parameter(name + "." + "robot.kinematic_bicycle_vel_input.max_vel_x_backwards", robot.kinematic_bicycle_vel_input.max_vel_x_backwards);

  // Solver
  node->declare_parameter(name + "." + "solver.type", solver.type);

  node->declare_parameters(name + "." + "solver.ipopt.integer_options", solver.ipopt.integer_options);
  node->declare_parameter(name + "." + "solver.ipopt.iterations", solver.ipopt.iterations);
  node->declare_parameter(name + "." + "solver.ipopt.max_cpu_time", solver.ipopt.max_cpu_time);
  node->declare_parameters(name + "." + "solver.ipopt.numeric_options", solver.ipopt.numeric_options);
  node->declare_parameters(name + "." + "solver.ipopt.string_options", solver.ipopt.string_options);

  node->declare_parameter(name + "." + "solver.lsq_lm.iterations", solver.lsq_lm.iterations);

  node->declare_parameter(name + "." + "solver.lsq_lm.weight_init_eq", solver.lsq_lm.weight_init_eq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_init_ineq", solver.lsq_lm.weight_init_ineq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_init_bounds", solver.lsq_lm.weight_init_bounds);

  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_factor_eq", solver.lsq_lm.weight_adapt_factor_eq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_factor_ineq", solver.lsq_lm.weight_adapt_factor_ineq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_factor_bounds", solver.lsq_lm.weight_adapt_factor_bounds);

  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_max_eq", solver.lsq_lm.weight_adapt_max_eq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_max_ineq", solver.lsq_lm.weight_adapt_max_ineq);
  node->declare_parameter(name + "." + "solver.lsq_lm.weight_adapt_max_bounds", solver.lsq_lm.weight_adapt_max_bounds);
}

void MpcConfig::loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr node, const std::string name)
{
  node->get_parameter_or(name + "." + "odom_topic", odom_topic, odom_topic);
  node->get_parameter_or(name + "." + "map_frame", map_frame, map_frame);

  node->get_parameter_or(name + "." + "costmap_converter_plugin", costmap_converter_plugin, costmap_converter_plugin);
  node->get_parameter_or(name + "." + "costmap_converter_rate", costmap_converter_rate, costmap_converter_rate);
  node->get_parameter_or(name + "." + "costmap_converter_spin_thread", costmap_converter_spin_thread, costmap_converter_spin_thread);

  // Collision Avoidance
  node->get_parameter_or(name + "." + "collision_avoidance.collision_check_min_resolution_angular", collision_avoidance.collision_check_min_resolution_angular, collision_avoidance.collision_check_min_resolution_angular);
  node->get_parameter_or(name + "." + "collision_avoidance.collision_check_no_poses", collision_avoidance.collision_check_no_poses, collision_avoidance.collision_check_no_poses);
  node->get_parameter_or(name + "." + "collision_avoidance.costmap_obstacles_behind_robot_dist", collision_avoidance.costmap_obstacles_behind_robot_dist, collision_avoidance.costmap_obstacles_behind_robot_dist);
  node->get_parameter_or(name + "." + "collision_avoidance.cutoff_dist", collision_avoidance.cutoff_dist, collision_avoidance.cutoff_dist);
  node->get_parameter_or(name + "." + "collision_avoidance.enable_dynamic_obstacles", collision_avoidance.enable_dynamic_obstacles, collision_avoidance.enable_dynamic_obstacles);
  node->get_parameter_or(name + "." + "collision_avoidance.force_inclusion_dist", collision_avoidance.force_inclusion_dist, collision_avoidance.force_inclusion_dist);
  node->get_parameter_or(name + "." + "collision_avoidance.include_costmap_obstacles", collision_avoidance.include_costmap_obstacles, collision_avoidance.include_costmap_obstacles);
  node->get_parameter_or(name + "." + "collision_avoidance.min_obstacle_dist", collision_avoidance.min_obstacle_dist, collision_avoidance.min_obstacle_dist);

  // Controller
  node->get_parameter_or(name + "." + "controller.allow_init_with_backward_motion", controller.allow_init_with_backwards_motion, controller.allow_init_with_backwards_motion);
  node->get_parameter_or(name + "." + "controller.force_reinit_new_goal_angular", controller.force_reinit_new_goal_angular, controller.force_reinit_new_goal_angular);
  node->get_parameter_or(name + "." + "controller.force_reinit_new_goal_dist", controller.force_reinit_new_goal_dist, controller.force_reinit_new_goal_dist);
  node->get_parameter_or(name + "." + "controller.force_reinit_num_steps", controller.force_reinit_num_steps, controller.force_reinit_num_steps);
  node->get_parameter_or(name + "." + "controller.global_plan_overwrite_orientation", controller.global_plan_overwrite_orientation, controller.global_plan_overwrite_orientation);
  node->get_parameter_or(name + "." + "controller.global_plan_prune_distance", controller.global_plan_prune_distance, controller.global_plan_prune_distance);
  node->get_parameter_or(name + "." + "controller.global_plan_viapoint_sep", controller.global_plan_viapoint_sep, controller.global_plan_viapoint_sep);
  node->get_parameter_or(name + "." + "controller.max_global_plan_lookahead_dist", controller.max_global_plan_lookahead_dist, controller.max_global_plan_lookahead_dist);
  node->get_parameter_or(name + "." + "controller.outer_ocp_iterations", controller.outer_ocp_iterations, controller.outer_ocp_iterations);
  node->get_parameter_or(name + "." + "controller.prefer_x_feedback", controller.prefer_x_feedback, controller.prefer_x_feedback);
  node->get_parameter_or(name + "." + "controller.print_cpu_time", controller.print_cpu_time, controller.print_cpu_time);
  node->get_parameter_or(name + "." + "controller.publish_ocp_results", controller.publish_ocp_results, controller.publish_ocp_results);
  node->get_parameter_or(name + "." + "controller.xy_goal_tolerance", controller.xy_goal_tolerance, controller.xy_goal_tolerance);
  node->get_parameter_or(name + "." + "controller.yaw_goal_tolerance", controller.yaw_goal_tolerance, controller.yaw_goal_tolerance);

  // Footprint Model
  node->get_parameter_or(name + "." + "footprint_model.is_footprint_dynamic", footprint_model.is_footprint_dynamic, footprint_model.is_footprint_dynamic);
  node->get_parameter_or(name + "." + "footprint_model.type", footprint_model.type, footprint_model.type);
  node->get_parameter_or(name + "." + "footprint_model.radius", footprint_model.radius, footprint_model.radius);
  node->get_parameter_or(name + "." + "footprint_model.line_start", footprint_model.line_start, footprint_model.line_start);
  node->get_parameter_or(name + "." + "footprint_model.line_end", footprint_model.line_end, footprint_model.line_end);
  node->get_parameter_or(name + "." + "footprint_model.front_offset", footprint_model.front_offset, footprint_model.front_offset);
  node->get_parameter_or(name + "." + "footprint_model.front_radius", footprint_model.front_radius, footprint_model.front_radius);
  node->get_parameter_or(name + "." + "footprint_model.rear_offset", footprint_model.rear_offset, footprint_model.rear_offset);
  node->get_parameter_or(name + "." + "footprint_model.rear_radius", footprint_model.rear_radius, footprint_model.rear_radius);

  // Grid
  node->get_parameter_or(name + "." + "grid.collocation_method", grid.collocation_method, grid.collocation_method);
  node->get_parameter_or(name + "." + "grid.cost_integration_method", grid.cost_integration_method, grid.cost_integration_method);
  node->get_parameter_or(name + "." + "grid.dt_ref", grid.dt_ref, grid.dt_ref);
  node->get_parameter_or(name + "." + "grid.grid_size_ref", grid.grid_size_ref, grid.grid_size_ref);
  node->get_parameter_or(name + "." + "grid.type", grid.type, grid.type);
  node->get_parameter_or(name + "." + "grid.warm_start", grid.warm_start, grid.warm_start);
  node->get_parameter_or(name + "." + "grid.xf_fixed", grid.xf_fixed, grid.xf_fixed);

  node->get_parameter_or(name + "." + "grid.variable_grid.enable", grid.variable_grid.enable, grid.variable_grid.enable);
  node->get_parameter_or(name + "." + "grid.variable_grid.max_dt", grid.variable_grid.max_dt, grid.variable_grid.max_dt);
  node->get_parameter_or(name + "." + "grid.variable_grid.min_dt", grid.variable_grid.min_dt, grid.variable_grid.min_dt);

  node->get_parameter_or(name + "." + "grid.variable_grid.grid_adaptation.enable", grid.variable_grid.grid_adaptation.enable, grid.variable_grid.grid_adaptation.enable);
  node->get_parameter_or(name + "." + "grid.variable_grid.grid_adaptation.dt_hyst_ratio", grid.variable_grid.grid_adaptation.dt_hyst_ratio, grid.variable_grid.grid_adaptation.dt_hyst_ratio);
  node->get_parameter_or(name + "." + "grid.variable_grid.grid_adaptation.max_grid_size", grid.variable_grid.grid_adaptation.max_grid_size, grid.variable_grid.grid_adaptation.max_grid_size);
  node->get_parameter_or(name + "." + "grid.variable_grid.grid_adaptation.min_grid_size", grid.variable_grid.grid_adaptation.min_grid_size, grid.variable_grid.grid_adaptation.min_grid_size);

  // Planning
  node->get_parameter_or(name + "." + "planning.objective.type", planning.objective.type, planning.objective.type);

  node->get_parameter_or(name + "." + "planning.objective.minimum_time_via_points.via_points_ordered", planning.objective.minimum_time_via_points.via_points_ordered, planning.objective.minimum_time_via_points.via_points_ordered);
  node->get_parameter_or(name + "." + "planning.objective.minimum_time_via_points.position_weight", planning.objective.minimum_time_via_points.position_weight, planning.objective.minimum_time_via_points.position_weight);
  node->get_parameter_or(name + "." + "planning.objective.minimum_time_via_points.orientation_weight", planning.objective.minimum_time_via_points.orientation_weight, planning.objective.minimum_time_via_points.orientation_weight);

  node->get_parameter_or(name + "." + "planning.objective.quadratic_form.control_weights", planning.objective.quadratic_form.control_weights, planning.objective.quadratic_form.control_weights);
  node->get_parameter_or(name + "." + "planning.objective.quadratic_form.hybrid_cost_minimum_time", planning.objective.quadratic_form.hybrid_cost_minimum_time, planning.objective.quadratic_form.hybrid_cost_minimum_time);
  node->get_parameter_or(name + "." + "planning.objective.quadratic_form.integral_form", planning.objective.quadratic_form.integral_form, planning.objective.quadratic_form.integral_form);
  node->get_parameter_or(name + "." + "planning.objective.quadratic_form.state_weights", planning.objective.quadratic_form.state_weights, planning.objective.quadratic_form.state_weights);

  node->get_parameter_or(name + "." + "planning.terminal_constraint.type", planning.terminal_constraint.type, planning.terminal_constraint.type);

  node->get_parameter_or(name + "." + "planning.terminal_constraint.l2_ball.radius", planning.terminal_constraint.l2ball.radius, planning.terminal_constraint.l2ball.radius);
  node->get_parameter_or(name + "." + "planning.terminal_constraint.l2_ball.weight_matrix", planning.terminal_constraint.l2ball.weight_matrix, planning.terminal_constraint.l2ball.weight_matrix);

  node->get_parameter_or(name + "." + "planning.terminal_cost.type", planning.terminal_cost.type, planning.terminal_cost.type);
  node->get_parameter_or(name + "." + "planning.terminal_cost.quadratic.final_state_weights", planning.terminal_cost.quadratic.state_weights, planning.terminal_cost.quadratic.state_weights);

  // Robot
  node->get_parameter_or(name + "." + "robot.type", robot.type, robot.type);

  node->get_parameter_or(name + "." + "robot.unicycle.acc_lim_theta", robot.unicycle.acc_lim_theta, robot.unicycle.acc_lim_theta);
  node->get_parameter_or(name + "." + "robot.unicycle.acc_lim_x", robot.unicycle.acc_lim_x, robot.unicycle.acc_lim_x);
  node->get_parameter_or(name + "." + "robot.unicycle.dec_lim_x", robot.unicycle.dec_lim_x, robot.unicycle.dec_lim_x);
  node->get_parameter_or(name + "." + "robot.unicycle.max_vel_theta", robot.unicycle.max_vel_theta, robot.unicycle.max_vel_theta);
  node->get_parameter_or(name + "." + "robot.unicycle.max_vel_x", robot.unicycle.max_vel_x, robot.unicycle.max_vel_x);
  node->get_parameter_or(name + "." + "robot.unicycle.max_vel_x_backwards", robot.unicycle.max_vel_x_backwards, robot.unicycle.max_vel_x_backwards);

  node->get_parameter_or(name + "." + "robot.simple_car.acc_lim_x", robot.simple_car.acc_lim_x, robot.simple_car.acc_lim_x);
  node->get_parameter_or(name + "." + "robot.simple_car.dec_lim_x", robot.simple_car.dec_lim_x, robot.simple_car.dec_lim_x);
  node->get_parameter_or(name + "." + "robot.simple_car.front_wheel_driving", robot.simple_car.front_wheel_driving, robot.simple_car.front_wheel_driving);
  node->get_parameter_or(name + "." + "robot.simple_car.max_steering_angle", robot.simple_car.max_steering_angle, robot.simple_car.max_steering_angle);
  node->get_parameter_or(name + "." + "robot.simple_car.max_steering_rate", robot.simple_car.max_steering_rate, robot.simple_car.max_steering_rate);
  node->get_parameter_or(name + "." + "robot.simple_car.max_vel_x", robot.simple_car.max_vel_x, robot.simple_car.max_vel_x);
  node->get_parameter_or(name + "." + "robot.simple_car.max_vel_x_backwards", robot.simple_car.max_vel_x_backwards, robot.simple_car.max_vel_x_backwards);
  node->get_parameter_or(name + "." + "robot.simple_car.wheelbase", robot.simple_car.wheelbase, robot.simple_car.wheelbase);

  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.acc_lim_x", robot.kinematic_bicycle_vel_input.acc_lim_x, robot.kinematic_bicycle_vel_input.acc_lim_x);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.dec_lim_x", robot.kinematic_bicycle_vel_input.dec_lim_x, robot.kinematic_bicycle_vel_input.dec_lim_x);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.length_front", robot.kinematic_bicycle_vel_input.length_front, robot.kinematic_bicycle_vel_input.length_front);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.length_rear", robot.kinematic_bicycle_vel_input.length_rear, robot.kinematic_bicycle_vel_input.length_rear);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.max_steering_angle", robot.kinematic_bicycle_vel_input.max_steering_angle, robot.kinematic_bicycle_vel_input.max_steering_angle);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.max_steering_rate", robot.kinematic_bicycle_vel_input.max_steering_rate, robot.kinematic_bicycle_vel_input.max_steering_rate);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.max_vel_x", robot.kinematic_bicycle_vel_input.max_vel_x, robot.kinematic_bicycle_vel_input.max_vel_x);
  node->get_parameter_or(name + "." + "robot.kinematic_bicycle_vel_input.max_vel_x_backwards", robot.kinematic_bicycle_vel_input.max_vel_x_backwards, robot.kinematic_bicycle_vel_input.max_vel_x_backwards);

  // Solver
  node->get_parameter_or(name + "." + "solver.type", solver.type, solver.type);

  node->get_parameters(name + "." + "solver.ipopt.integer_options", solver.ipopt.integer_options);
  node->get_parameter_or(name + "." + "solver.ipopt.iterations", solver.ipopt.iterations, solver.ipopt.iterations);
  node->get_parameter_or(name + "." + "solver.ipopt.max_cpu_time", solver.ipopt.max_cpu_time, solver.ipopt.max_cpu_time);
  node->get_parameters(name + "." + "solver.ipopt.numeric_options", solver.ipopt.numeric_options);
  node->get_parameters(name + "." + "solver.ipopt.string_options", solver.ipopt.string_options);

  node->get_parameter_or(name + "." + "solver.lsq_lm.iterations", solver.lsq_lm.iterations, solver.lsq_lm.iterations);

  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_init_eq", solver.lsq_lm.weight_init_eq, solver.lsq_lm.weight_init_eq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_init_ineq", solver.lsq_lm.weight_init_ineq, solver.lsq_lm.weight_init_ineq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_init_bounds", solver.lsq_lm.weight_init_bounds, solver.lsq_lm.weight_init_bounds);

  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_factor_eq", solver.lsq_lm.weight_adapt_factor_eq, solver.lsq_lm.weight_adapt_factor_eq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_factor_ineq", solver.lsq_lm.weight_adapt_factor_ineq, solver.lsq_lm.weight_adapt_factor_ineq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_factor_bounds", solver.lsq_lm.weight_adapt_factor_bounds, solver.lsq_lm.weight_adapt_factor_bounds);

  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_max_eq", solver.lsq_lm.weight_adapt_max_eq, solver.lsq_lm.weight_adapt_max_eq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_max_ineq", solver.lsq_lm.weight_adapt_max_ineq, solver.lsq_lm.weight_adapt_max_ineq);
  node->get_parameter_or(name + "." + "solver.lsq_lm.weight_adapt_max_bounds", solver.lsq_lm.weight_adapt_max_bounds, solver.lsq_lm.weight_adapt_max_bounds);
}

} // namespace mpc_local_planner
