mpc_local_planner ROS Package
=============================

The mpc_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack.
It provides a generic and versatile model predictive control implementation with minimum-time and quadratic-form receding-horizon configurations.

## Authors

* Christoph Rösmann <christoph.roesmann@tu-dortmund.de>

### Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the software for published work.*

**Standard MPC and Hypergraph**

- C. Rösmann, M. Krämer, A. Makarow, F. Hoffmann und T. Bertram: Exploiting Sparse Structures in Nonlinear Model Predictive Control with Hypergraphs, IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), New Zealand, July 2018.

**Time-Optimal MPC and Hypergraph**

- C. Rösmann: [Time-optimal nonlinear model predictive control - Direct transcription methods with variable discretization and structural sparsity exploitation](http://dx.doi.org/10.17877/DE290R-20283). Dissertation, TU Dortmund University, Oct. 2019.
- C. Rösmann, F. Hoffmann und T. Bertram: Timed-Elastic-Bands for Time-Optimal Point-to-Point Nonlinear Model Predictive Control, European Control Conference (ECC), Austria, July 2015.
- C. Rösmann, F. Hoffman und T. Bertram: Convergence Analysis of Time-Optimal Model Predictive Control under Limited Computational Resources, European Control Conference (ECC), Denmark, June 2016.


### License

Copyright (c) 2020,
TU Dortmund - Institute of Control Theory and Systems Engineering.
All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

The package depends on third-party packages:
- *control_box_rst*, GPLv3, https://github.com/rst-tu-dortmund/control_box_rst
  Please check also the dependencies of control_box_rst (not all modules
  are included at the moment)
- *Ipopt*, EPL 1.0, https://github.com/coin-or/Ipopt


It depends on other ROS packages, which are listed in the package.xml. They are licensed under BSD resp. Apache 2.0.


### Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install mpc_local_planner


