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

#ifndef UTILS_MATH_UTILS_H_
#define UTILS_MATH_UTILS_H_

#include <cmath>

namespace mpc_local_planner {

/**
 * @brief Return the average angle of an arbitrary number of given angles [rad]
 * @param angles vector containing all angles
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double average_angles(const std::vector<double>& angles)
{
    double x = 0, y = 0;
    for (std::vector<double>::const_iterator it = angles.begin(); it != angles.end(); ++it)
    {
        x += std::cos(*it);
        y += std::sin(*it);
    }
    if (x == 0 && y == 0)
        return 0;
    else
        return std::atan2(y, x);
}

/**
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
 */
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
    return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
}

//! Calculate Euclidean distance between two 2D points
inline double distance_points2d(double x1, double y1, double x2, double y2) { return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)); }

/**
 * @brief Calculate the 2d cross product (returns length of the resulting vector along the z-axis in 3d)
 * @param v1 object containing public methods x() and y()
 * @param v2 object containing fields x() and y()
 * @return magnitude that would result in the 3D case (along the z-axis)
 */
template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2)
{
    return v1.x() * v2.y() - v2.x() * v1.y();
}

/**
 * @brief normalize angle to interval [-pi, pi)
 * @remark This function is based on normalize_theta from g2o
 *         see: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/stuff/misc.h
 */
inline double normalize_theta(double theta)
{
    if (theta >= -M_PI && theta < M_PI) return theta;

    double multiplier = std::floor(theta / (2.0 * M_PI));
    theta             = theta - multiplier * 2.0 * M_PI;
    if (theta >= M_PI) theta -= 2.0 * M_PI;
    if (theta < -M_PI) theta += 2.0 * M_PI;

    return theta;
}

/**
 * @brief Return the interpolated angle between two angles [rad]
 * @param angle1
 * @param angle2
 * @param factor in [0,1], or (1,inf) for extrapolation
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double interpolate_angle(double angle1, double angle2, double factor)
{
    return normalize_theta(angle1 + factor * normalize_theta(angle2 - angle1));
}

}  // namespace mpc_local_planner

#endif  // UTILS_MATH_UTILS_H_
