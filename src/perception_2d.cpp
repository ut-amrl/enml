//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
// Copyright 2015 joydeepb@cs.umass.edu
// Computer Science Department, University of Massachusetts Amherst
//
// Data types and helper functions for perception in 2D.

#include "perception_2d.h"

#include <math.h>
#include <iostream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_shared_lib/math/math_util.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;

namespace perception_2d {

template <typename T>
Eigen::Matrix<T, 2, 1> RotateBy90(const Eigen::Matrix<T, 2, 1>& v) {
  return Eigen::Matrix<T, 2, 1>(-v.y(), v.x());
}

void GenerateNormals(const float max_point_neighbor_distance,
                     const std::vector<float>& weights,
                     PointCloudf* point_cloud_ptr,
                     NormalCloudf* normal_cloud_ptr) {
  PointCloudf& point_cloud = *point_cloud_ptr;
  NormalCloudf& normal_cloud = *normal_cloud_ptr;
  normal_cloud.resize(point_cloud.size());
  const int N = weights.size();
  const float kSqMaxDist = math_util::Sq(max_point_neighbor_distance);
  for (int i = 0; i < int(point_cloud.size()); ++i) {
    float weight = 0.0;
    Vector2f tangent(0, 0);
    for (int j = -(N - 1); j < N; ++j) {
      if (j == 0) continue;
      const int k = i + j;
      if (k >= 0 &&
          k < int(point_cloud.size()) &&
          (point_cloud[i] - point_cloud[k]).squaredNorm() < kSqMaxDist) {
        const float w = weights[abs(j)];
        weight += w;
        if (j > 0) {
          tangent += w * (point_cloud[i] - point_cloud[k]).normalized();
        } else {
          tangent += w * (point_cloud[k] - point_cloud[i]).normalized();
        }
      }
    }
    if (weight > 0.0f) {
      if (false) {
        std::cout << "tangent:\n"
                  << tangent
                  << "\nWeight:"
                  << weight
                  << "\n";
      }
      normal_cloud[i] = RotateBy90<float>(tangent / weight).normalized();
    } else {
      point_cloud.erase(point_cloud.begin() + i);
      normal_cloud.erase(normal_cloud.begin() + i);
      --i;
    }
  }
}

void GenerateNormals(const float max_point_neighbor_distance,
                     PointCloudf* point_cloud_ptr,
                     NormalCloudf* normal_cloud_ptr) {
  PointCloudf& point_cloud = *point_cloud_ptr;
  NormalCloudf& normal_cloud = *normal_cloud_ptr;
  normal_cloud.resize(point_cloud.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    float count = 0.0;
    Vector2f normal(0.0, 0.0);
    if (i > 0 &&
        (point_cloud[i] - point_cloud[i-1]).norm() <
        max_point_neighbor_distance) {
      normal += RotateBy90<float>(point_cloud[i] - point_cloud[i-1]).normalized();
      count += 1.0;
    }
    if (i < point_cloud.size() - 1 &&
        (point_cloud[i+1] - point_cloud[i]).norm() <
        max_point_neighbor_distance) {
      normal += RotateBy90<float>(point_cloud[i+1] - point_cloud[i]).normalized();
      count += 1.0;
    }
    if (count > 0.0) {
      normal = (normal / count).normalized();
      normal_cloud[i] = normal;
    } else {
      point_cloud.erase(point_cloud.begin() + i);
      normal_cloud.erase(normal_cloud.begin() + i);
      --i;
    }
  }
}

}  // namespace perception_2d

