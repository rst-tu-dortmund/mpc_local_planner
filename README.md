mpc_local_planner ROS Package
=============================

The mpc_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack.
It provides a generic and versatile model predictive control implementation with minimum-time and quadratic-form receding-horizon configurations.

For custom build instructions (e.g. compilation with other third-party solvers) see [this wiki](https://github.com/rst-tu-dortmund/mpc_local_planner/wiki).

Refer to http://wiki.ros.org/mpc_local_planner for more general information and tutorials.

Build status:
- ROS Melodic (*melodic-devel*): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__mpc_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__mpc_local_planner__ubuntu_bionic_amd64/)

## Authors

* Christoph Rösmann <christoph.roesmann@tu-dortmund.de>

## Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the software for published work.*

**Main Paper and Approach**

- C. Rösmann, A. Makarow, and T. Bertram: Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups, [arXiv:2006.03534 '[cs.RO]'](https://arxiv.org/abs/2006.03534), June 2020.

**Standard MPC and Hypergraph**

- C. Rösmann, M. Krämer, A. Makarow, F. Hoffmann and T. Bertram: Exploiting Sparse Structures in Nonlinear Model Predictive Control with Hypergraphs, IEEE/ASME International Conference on Advanced Intelligent Mechatronics (AIM), New Zealand, July 2018.

**Time-Optimal MPC and Hypergraph**

- C. Rösmann: [Time-optimal nonlinear model predictive control, Direct transcription methods with variable discretization and structural sparsity exploitation](http://dx.doi.org/10.17877/DE290R-20283). Dissertation, Technische Universität Dortmund, Oct. 2019.
- C. Rösmann, F. Hoffmann und T. Bertram: Timed-Elastic-Bands for Time-Optimal Point-to-Point Nonlinear Model Predictive Control, European Control Conference (ECC), Austria, July 2015.


<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://www.buymeacoffee.com/assets/img/guidelines/download-assets-sm-2.svg" alt="Buy Me A Coffee"></a>

## License

Copyright (c) 2020, Christoph Rösmann, All rights reserved.

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


## Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install mpc_local_planner
