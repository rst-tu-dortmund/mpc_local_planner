#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H

#include <nav2_util/lifecycle_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <teb_local_planner/distance_calculations.h>

namespace mpc_local_planner
{

class MpcConfig
{
typedef std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2dContainer;
public:
  using UniquePtr = std::unique_ptr<MpcConfig>;

  std::string odom_topic;
  std::string map_frame;
  std::string global_plan_topic;

  std::string costmap_converter_plugin;
  double costmap_converter_rate;
  bool costmap_converter_spin_thread;

  struct CollisionAvoidance
  {
    double collision_check_min_resolution_angular;
    int    collision_check_no_poses;
    double costmap_obstacles_behind_robot_dist; // Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    double cutoff_dist;
    bool   enable_dynamic_obstacles;
    double force_inclusion_dist;
    bool   include_costmap_obstacles; // Specify whether the obstacles in the costmap should be taken into account directly (this is necessary if no seperate clustering and detection is implemented)
    double min_obstacle_dist;
  } collision_avoidance;

  struct Controller
  {
    bool   allow_init_with_backwards_motion;
    double controller_frequency;
    double force_reinit_new_goal_angular;
    double force_reinit_new_goal_dist;
    int    force_reinit_num_steps;
    bool   global_plan_overwrite_orientation; // Some global planners are not considering the orientation at local subgoals between start and global goal, therefore determine it automatically
    double global_plan_prune_distance;
    double global_plan_viapoint_sep; // Min. separation between each two consecutive via-points extracted from the global plan [if negative: disabled]
    double max_global_plan_lookahead_dist; // Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if 0 or negative: disabled; the length is also bounded by the local costmap size]
    int    outer_ocp_iterations;
    bool   prefer_x_feedback;
    bool   print_cpu_time;
    bool   publish_ocp_results;
    double xy_goal_tolerance; // Allowed final euclidean distance to the goal position
    double yaw_goal_tolerance; // Allowed final orientation error to the goal orientation
  } controller;

  struct FootprintModel
  {
    std::string type;
    double radius;
    double front_offset;
    double front_radius;
    double l1;
    double l2;
    std::vector<double> line_start;
    std::vector<double> line_end;
    double rear_offset;
    double rear_radius;
    Point2dContainer vertices;
    bool is_footprint_dynamic; // If true, updated the footprint before checking trajectory feasibility
  } footprint_model;

  struct Grid
  {
    std::string       collocation_method;
    std::string       cost_integration_method;
    double            dt_ref;
    int               grid_size_ref;
    std::string       type;
    bool              warm_start;
    std::vector<bool> xf_fixed;

    struct VariableGrid
    {
      bool enable;
      double max_dt;
      double min_dt;

      struct GridAdaptation
      {
        bool   enable;
        double dt_hyst_ratio;
        int    max_grid_size;
        int    min_grid_size;
      } grid_adaptation;

    } variable_grid;

  } grid;

  struct Planning
  {
    struct Objective
    {
      std::string type;

      struct QuadraticForm
      {
        std::vector<double> control_weights;
        bool                hybrid_cost_minimum_time;
        bool                integral_form;
        std::vector<double> state_weights;
      } quadratic_form;

      struct MinimumTimeViaPoints
      {
        double orientation_weight;
        double position_weight;
        bool   via_points_ordered;
      } minimum_time_via_points;
    } objective;

        struct TerminalConstraint
    {
      std::string type;

      struct L2Ball
      {
        double              radius;
        std::vector<double> weight_matrix;
      } l2ball;
    } terminal_constraint;

    struct TerminalCost
    {
      std::string type;

      struct Quadratic
      {
        std::vector<double> state_weights;

      } quadratic;
    } terminal_cost;
  } planning;

  struct Robot
  {
    std::string type;

    struct Unicycle
    {
      double acc_lim_theta;
      double acc_lim_x;
      double dec_lim_x;
      double max_vel_theta;
      double max_vel_x;
      double max_vel_x_backwards;
    } unicycle;

    struct SimpleCar
    {
      double acc_lim_x;
      double dec_lim_x;
      bool front_wheel_driving;
      double max_steering_angle;
      double max_steering_rate;
      double max_vel_x;
      double max_vel_x_backwards;
      double wheelbase;
    } simple_car;

    struct KinematicBicycleVelInput
    {
      double acc_lim_x;
      double dec_lim_x;
      double length_front;
      double length_rear;
      double max_steering_angle;
      double max_steering_rate;
      double max_vel_x;
      double max_vel_x_backwards;
    } kinematic_bicycle_vel_input;


  } robot;

  struct Solver
  {
    std::string type;

    struct Ipopt
    {
      std::map<std::string, int>         integer_options;
      int                                iterations;
      double                             max_cpu_time;
      std::map<std::string, double>      numeric_options;
      std::map<std::string, std::string> string_options;
    } ipopt;

    struct LsqLm
    {
      int iterations;
      double weight_init_eq;
      double weight_init_ineq;
      double weight_init_bounds;
      double weight_adapt_factor_eq;
      double weight_adapt_factor_ineq;
      double weight_adapt_factor_bounds;
      double weight_adapt_max_eq;
      double weight_adapt_max_ineq;
      double weight_adapt_max_bounds;
    } lsq_lm;
  } solver;

  MpcConfig()
  {
    odom_topic = "odom";
    map_frame= "odom";
    global_plan_topic = "plan";

    costmap_converter_rate      = 5;
    costmap_converter_spin_thread = true;

    // CollisionAvoidance
    collision_avoidance.collision_check_min_resolution_angular = 3.1415;
    collision_avoidance.collision_check_no_poses = -1;
    collision_avoidance.costmap_obstacles_behind_robot_dist = 1.5;
    collision_avoidance.cutoff_dist = 2.;
    collision_avoidance.enable_dynamic_obstacles = false;
    collision_avoidance.force_inclusion_dist = 0.5;
    collision_avoidance.include_costmap_obstacles = true;
    collision_avoidance.min_obstacle_dist = 0.5;

    // Controller
    controller.allow_init_with_backwards_motion = true;
    controller.controller_frequency = 10.;
    controller.force_reinit_new_goal_dist = 1.0;
    controller.force_reinit_new_goal_angular = 0.5 * M_PI;
    controller.force_reinit_num_steps = 0;
    controller.global_plan_overwrite_orientation = true;
    controller.global_plan_prune_distance = 1.0;
    controller.global_plan_viapoint_sep = -1.;
    controller.max_global_plan_lookahead_dist = 1.5;
    controller.outer_ocp_iterations = 1;
    controller.prefer_x_feedback = false;
    controller.print_cpu_time = false;
    controller.publish_ocp_results = false;
    controller.xy_goal_tolerance = 0.2;
    controller.yaw_goal_tolerance = 0.1;

    // Footprint
    footprint_model.is_footprint_dynamic = false;

    // Grid
    grid.collocation_method = "forward_differences";
    grid.cost_integration_method = "left_sum";
    grid.dt_ref = 0.3;
    grid.grid_size_ref = 20;
    grid.type = "fd_grid";
    grid.warm_start = false;
    grid.xf_fixed = {true, true, true};

    grid.variable_grid.enable = true;
    grid.variable_grid.max_dt = 10.0;
    grid.variable_grid.min_dt = 0.;

    grid.variable_grid.grid_adaptation.dt_hyst_ratio = 0.1;
    grid.variable_grid.grid_adaptation.enable = true;
    grid.variable_grid.grid_adaptation.max_grid_size = 50;
    grid.variable_grid.grid_adaptation.min_grid_size = 2;

    // Planning
    planning.objective.type = "minimum_time";

    planning.objective.quadratic_form.hybrid_cost_minimum_time = false;
    planning.objective.quadratic_form.integral_form = false;

    planning.objective.minimum_time_via_points.orientation_weight = 0.;
    planning.objective.minimum_time_via_points.position_weight = 1.;
    planning.objective.minimum_time_via_points.via_points_ordered = false;

    planning.terminal_cost.type = "none";

    planning.terminal_constraint.type = "none";

    planning.terminal_constraint.l2ball.radius = 1.;


    // Robot
    robot.type = "unicycle";

    robot.unicycle.acc_lim_theta = 0.;
    robot.unicycle.acc_lim_x = 0.;
    robot.unicycle.dec_lim_x = 0.;

    robot.unicycle.max_vel_theta = 0.3;
    robot.unicycle.max_vel_x = 0.4;
    robot.unicycle.max_vel_x_backwards = 0.2;

    robot.simple_car.acc_lim_x = 0.;
    robot.simple_car.dec_lim_x = 0.;

    robot.simple_car.front_wheel_driving = false;

    robot.simple_car.max_steering_angle = 1.5;
    robot.simple_car.max_steering_rate = 0.0;

    robot.simple_car.max_vel_x = 0.4;
    robot.simple_car.max_vel_x_backwards = 0.2;

    robot.simple_car.wheelbase = 0.5;

    robot.kinematic_bicycle_vel_input.acc_lim_x = 0.;
    robot.kinematic_bicycle_vel_input.dec_lim_x = 0.;

    robot.kinematic_bicycle_vel_input.length_front = 1.;
    robot.kinematic_bicycle_vel_input.length_rear = 1.;

    robot.kinematic_bicycle_vel_input.max_steering_angle = 1.5;
    robot.kinematic_bicycle_vel_input.max_steering_rate = 0.;

    robot.kinematic_bicycle_vel_input.max_vel_x = 0.4;
    robot.kinematic_bicycle_vel_input.max_vel_x_backwards = 0.2;

    // Solver
    solver.type = "ipopt";

    solver.ipopt.iterations = 100;
    solver.ipopt.max_cpu_time = -1.;

    solver.lsq_lm.iterations = 10;

    solver.lsq_lm.weight_init_eq = 2.;
    solver.lsq_lm.weight_init_ineq = 2.;
    solver.lsq_lm.weight_init_bounds = 2.;

    solver.lsq_lm.weight_adapt_factor_eq = 1.;
    solver.lsq_lm.weight_adapt_factor_ineq = 1.;
    solver.lsq_lm.weight_adapt_factor_bounds = 1.;

    solver.lsq_lm.weight_adapt_max_eq = 500.;
    solver.lsq_lm.weight_adapt_max_ineq = 500.;
    solver.lsq_lm.weight_adapt_max_bounds = 500.;
  }

  void declareParameters(const nav2_util::LifecycleNode::SharedPtr node, const std::string name);

  void loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr node, const std::string name);

  std::mutex& configMutex() {return config_mutex_;}

  private:
  std::mutex config_mutex_;
};

} // namespace mpc_local_planner


#endif // MPC_CONFIG_H
