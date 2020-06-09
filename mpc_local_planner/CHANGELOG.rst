^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mpc_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2020-06-09)
------------------
* Added feasibility check with costmap robot model
* Changed obstacle parameters `cutoff_factor` and `force_inclusion_factor` to `cutoff_dist` and `force_inclusion_dist`
* Added check for obstacle pointer validity in StageInequalitySE2
* Dynamic obstacles: the inequality constraints now use the actual time parameter rather than the time from the previous optimization
* Added hybrid cost (time and control effort)
* Added kinematic bicycle model with velocity input
* Grid: The time difference is now initialized to dt_ref for reference trajectory caching
* Fixed wrong start orientation in point-to-point grid initialization
* Fixed wrong angular computation in midpoint differences
* Added missing install files (thanks to marbosjo)
* Changed minimum CMake version to 3.1
* Contributors: Christoph Rösmann, marbosjo

0.0.2 (2020-03-12)
------------------
* Added option to inherit the robot footprint model from the costmap_2d footprint by setting footprint type to `costmap_2d` (thanks to Alexander Xydes).
  Note that using a polygon as a robot footprint model is computationally more expensive than using two circle or line models.
* Fixed issues in fd discretization grid and graph consistency
* Fixed initialization of linear velocity output commands in simple car and unicycle models (thanks to Thomas Denewiler).
* Fixed wrong parameter namespaces for some grid parameters
* Replaced non-ASCII characters in python script
* Added find_package script for osqp in case control_box_rst is linked against it
* Contributors: Christoph Rösmann, Alexander Xydes, Thomas Denewiler

0.0.1 (2020-02-20)
------------------
* First release
* Contributors: Christoph Rösmann
