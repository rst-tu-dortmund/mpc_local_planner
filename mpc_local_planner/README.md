## Overview

The mpc_local_planner package implements a plugin to the
base_local_planner of the 2D navigation stack. It provides a generic and
versatile model predictive control implementation. The package already
provides standard realizations like minimum-time and quadratic-form
receding-horizon control, but can also be easily extended by deriving
from one of the various interface classes.

The following paper describes the approach:

- C. Rösmann, A. Makarow, and T. Bertram: Online Motion Planning based
  on Nonlinear Model Predictive Control with Non-Euclidean Rotation
  Groups, \[\[<https://arxiv.org/abs/2006.03534%7CarXiv:2006.03534>
  '\[cs.RO\]'\]\], June 2020.

## Get Started

The best way to get started is to look at one of the example
configurations in the
[related GitHub repository](https://github.com/rst-tu-dortmund/mpc_local_planner/tree/master/mpc_local_planner_examples).
The `cfg` directory contains several example *yaml* configurations.

It is **highly recommended** to be familiar with the
[navigation](https://index.ros.org/p/navigation) stack and all its
components (global planner, local planner, costmap_2d, \...).

It might also be helpful to be familiar with the
[teb_local_planner](https://index.ros.org/p/teb_local_planner) as the
planners are very similar from the API and parameter point of view (the
teb_local_planner wiki page already provides several tutorials which do
not exist for this planner yet).

Please also have a look at the repository's
[wiki page](https://github.com/rst-tu-dortmund/mpc_local_planner/wiki).

## Tutorials

- [teb_local_planner vs mpc_local_planner](#teb_local_planner-vs-mpc_local_planner):
  This page describes the main differences between teb_local_planner and
  mpc_local_planner

## teb_local_planner vs mpc_local_planner

- **Description:** This page describes the main differences between
  teb_local_planner and mpc_local_planner
- **Keywords:** MPC Trajectory Optimization Local Planner Navigation
- **Tutorial Level:** INTERMEDIATE

Refer to
[this repository wiki page](https://github.com/rst-tu-dortmund/mpc_local_planner/wiki/MPCvsTEB).
