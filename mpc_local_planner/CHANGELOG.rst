^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mpc_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
