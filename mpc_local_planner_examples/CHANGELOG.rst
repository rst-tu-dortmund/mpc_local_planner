^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mpc_local_planner_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2020-06-09)
------------------
* Added feasibility check with costmap robot model
* Changed obstacle parameters `cutoff_factor` and `force_inclusion_factor` to `cutoff_dist` and `force_inclusion_dist`
* Dynamic obstacles: the inequality constraints now use the actual time parameter rather than the time from the previous optimization
* Changed minimum CMake version to 3.1
* Contributors: Christoph Rösmann

0.0.2 (2020-03-12)
------------------
* Added dependency on mpc_local_planner_msgs package
* Default parameter set changed
* Contributors: Christoph Rösmann

0.0.1 (2020-02-20)
------------------
* First release
* Contributors: Christoph Rösmann
