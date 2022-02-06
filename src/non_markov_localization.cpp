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
// Copyright 2012 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Implementation of non-Markov Localization.

#include "non_markov_localization.h"

#include <dirent.h>
#include <errno.h>
#include <cmath>
#include <pthread.h>
#include <stdio.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "perception_2d.h"
#include "util/helpers.h"
#include "util/pthread_utils.h"
#include "util/random.h"
#include "util/timer.h"
#include "math/geometry.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include "math/statistics.h"
#include "residual_functors.h"
#include "vector_map/vector_map.h"

// #define ENABLE_TIMING

#ifdef ENABLE_TIMING
  #define TIME_FUNCTION FunctionTimer ft(__FUNCTION__);
#else
  #define TIME_FUNCTION ;
#endif

using ceres::AutoDiffCostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::IterationCallback;
using ceres::IterationSummary;
using Eigen::Affine2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using std::make_pair;
using std::max;
using std::min;
using std::pair;
using std::size_t;
using std::string;
using std::vector;
using vector_localization::LTSConstraint;
using vector_localization::PointToLineConstraint;
using vector_localization::PoseConstraint;
using vector_map::VectorMap;

using namespace geometry;
using namespace math_util;

typedef Eigen::Translation<float, 2> Translation2Df;

namespace {

struct EnmlTiming {
  double find_ltfs = 0.0;
  double find_stfs = 0.0;
  double add_constraints = 0.0;
  double solver = 0.0;
  double update = 0.0;
  double residuals = 0.0;
  double jacobians = 0.0;
  double linear_solver = 0.0;
  double preprocessor = 0.0;
  int ceres_iterations = 0;
  int enml_iterations = 0;
  string ceres_full_report;
  EnmlTiming() {}
  ~EnmlTiming() {
    if (true) {
      printf("========================================================\n");
      printf("  Ceres Full Report:\n%s\n", ceres_full_report.c_str());
      printf("========================================================\n");
      printf("ENML Iters:                          %6d\n"
             "  Ceres Iters:                       %6d\n"
             "  Find LTFs:                         %6.3f\n"
             "  Find STFs:                         %6.3f\n"
             "  Add Constraints:                   %6.3f\n"
             "  Ceres Solve:                       %6.3f\n"
             "    Preprocessor:                    %6.3f\n"
             "    Residual only eval:              %6.3f\n"
             "    Jacobian & residual eval:        %6.3f\n"
             "    Linear solver:                   %6.3f\n"
             "  Total Update:                      %6.3f\n",
            enml_iterations,
            ceres_iterations,
            find_ltfs,
            find_stfs,
            add_constraints,
            solver,
            preprocessor,
            residuals,
            jacobians,
            linear_solver,
            update);
    }
  }
};

static const bool kUseRelativeConstraints = false;
static const size_t kDynamicDiffStride = 4;

// Returns true if the point p lies alongside the line segment p0-p1. That is,
// the closest point to p on the line passing through p0 and p1 is within the
// line segment p0-p1.
bool LiesAlongLineSegment(
    const Vector2f &p0, const Vector2f &p1, const Vector2f &p) {
  return (((p1 - p0).dot(p - p0) >= 0.0) && ((p0 - p1).dot(p - p1) >= 0.0));
}

void SetSolverOptions(
    const vector_localization::NonMarkovLocalization::LocalizationOptions&
        localization_options,
    ceres::Solver::Options* options_ptr) {
  ceres::Solver::Options& solver_options = *options_ptr;

  // solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  solver_options.dense_linear_algebra_library_type = ceres::EIGEN;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.minimizer_type = ceres::TRUST_REGION;

  // solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.num_threads = localization_options.kNumThreads;
  solver_options.max_num_iterations =
      localization_options.max_solver_iterations;
  solver_options.function_tolerance = 1e-7;
  // solver_options.gradient_tolerance = 1e-15;
  // solver_options.initial_trust_region_radius = 0.5;
  // solver_options.max_trust_region_radius = 2.0;
  solver_options.update_state_every_iteration = true;
}

class SRLCallback : public ceres::IterationCallback {
 public:
  SRLCallback(const vector<double>& poses,
              const PointCloudf& point_cloud_e,
              const NormalCloudf& normal_cloud,
              const vector<LTSConstraint*>& constraints,
              void (*callback)(
                  const vector<double>& poses,
                  const PointCloudf& point_cloud_e,
                  const NormalCloudf& normal_cloud,
                  const vector<LTSConstraint*>& constraints)) :
      poses_(poses),
      point_cloud_e_(point_cloud_e), normal_cloud_(normal_cloud),
      constraints_(constraints), callback_(callback) {
  }

  virtual ceres::CallbackReturnType operator()(
      const IterationSummary& summary) {
    callback_(poses_, point_cloud_e_, normal_cloud_,
              constraints_);
    return ceres::SOLVER_CONTINUE;
  }

  const vector<double>& poses_;
  const PointCloudf& point_cloud_e_;
  const NormalCloudf& normal_cloud_;
  const vector<LTSConstraint*>& constraints_;

  void (*callback_)(
      const vector<double>& poses,
      const PointCloudf& point_cloud_e,
      const NormalCloudf& normal_cloud,
      const vector<LTSConstraint*>& constraints);
};

}  // namespace

namespace vector_localization {

NonMarkovLocalization::NonMarkovLocalization(const string& maps_directory) :
    lost_metric_(0),
    terminate_(false),
    next_pose_id_(0),
    t_last_update_(0),
    maps_dir_(maps_directory) {
  CHECK_EQ(sem_init(&update_semaphore_, 0, 0), 0);
  CHECK_EQ(pthread_mutex_init(&update_mutex_, NULL), 0);
  CHECK_EQ(pthread_create(
      &update_thread_, NULL, NonMarkovLocalization::UpdateThreadFunction,
      reinterpret_cast<void*>(this)), 0);
}

NonMarkovLocalization::~NonMarkovLocalization() {
  Terminate();
  CHECK_EQ(pthread_join(update_thread_, NULL), 0);
}

string NonMarkovLocalization::GetMapFileFromName(const string& map) const {
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

void NonMarkovLocalization::Reset() {
  next_pose_id_ = 0;
  ClearPoses();
}

void NonMarkovLocalization::Terminate() {
  if (!terminate_) {
    terminate_ = true;
    CHECK_EQ(sem_post(&update_semaphore_), 0);
  }
}


void* NonMarkovLocalization::UpdateThreadFunction(
    void* non_markov_localization_ptr) {
  NonMarkovLocalization& non_markov_localization =
      *(reinterpret_cast<NonMarkovLocalization*>(non_markov_localization_ptr));

  while (true) {
    int sem_error = 0;
    do {
        sem_error = sem_wait(&(non_markov_localization.update_semaphore_));
    } while (sem_error != 0 && (errno == EINTR || errno == EAGAIN));
    if (sem_error != 0) {
      const string str_error = StringPrintf(
          "Semaphore error (%d) in UpdateThreadFunction", errno);
      perror(str_error.c_str());
      google::LogMessageFatal(__FILE__, __LINE__);
    }
    if (non_markov_localization.terminate_) break;
    // Run update with a scoped lock.
    {
      ScopedLock lock(&non_markov_localization.update_mutex_);
      non_markov_localization.Update();
    }
  }
  return NULL;
}

Affine2f NonMarkovLocalization::RelativePoseTransform(
    unsigned int source, unsigned int target) {
  const Affine2f source_tf = Translation2Df(
      pose_array_[3 * source], pose_array_[3 * source + 1]) *
      Rotation2Df(pose_array_[3 * source + 2]);
  const Affine2f target_tf = Translation2Df(
        pose_array_[3 * target], pose_array_[3 * target + 1]) *
        Rotation2Df(pose_array_[3 * target + 2]);
  return target_tf.inverse(Eigen::Affine) * source_tf;
}

bool NonMarkovLocalization::HaveClassificationsChanged(
    const vector<vector<ObservationType> >& point_classes1,
    const vector<vector<ObservationType> >& point_classes2) {
  if (point_classes1.size() != point_classes2.size()) return true;
  for (size_t i = 0; i < point_classes1.size(); ++i) {
    const vector<ObservationType>& classes1 = point_classes1[i];
    const vector<ObservationType>& classes2 = point_classes2[i];
    if (classes1.size() != classes2.size()) return true;
    for (size_t j = 0; j < classes1.size(); ++j) {
      if (classes1[j] != classes2[j]) return true;
    }
  }
  return false;
}

void NonMarkovLocalization::FindVisualOdometryCorrespondences(
    const size_t min_poses, const size_t max_poses) {
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_.size());
  if (poses_end < min_poses + 1) return;
  vector<vector<PointToPointCorrespondence> > pose_correspondences(
      poses_end - min_poses - 1);
  OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < poses_end - 1; ++i) {
    PointToPointCorrespondence correspondence;
    correspondence.source_pose = i;
    correspondence.target_pose = i + 1;
    Affine2f source_to_target_tf = RelativePoseTransform(i, i + 1);
    for (size_t k = 0; k < point_clouds_[i].size(); ++k) {
      correspondence.source_point = k;
      const Vector2f point(source_to_target_tf * point_clouds_[i][k]);
      const Vector2f normal =
          Rotation2Df(pose_array_[3 * i + 2 + 3] - pose_array_[3 * i + 2]) *
          normal_clouds_[i][k];
      KDNodeValue<float, 2> neighbor_point;
      const float closest_distance = kdtrees_[i + 1]->FindNearestPoint(
          point, localization_options_.kPointMatchThreshold, &neighbor_point);
      const float cosine_angle = neighbor_point.normal.dot(normal);
      if (closest_distance < localization_options_.kPointMatchThreshold &&
          cosine_angle > min_cosine_angle) {
        // Valid point to point match found
        correspondence.target_point = neighbor_point.index;
        pose_correspondences[i - min_poses].push_back(correspondence);
      }
    }
  }
  for (size_t i = 0; i < pose_correspondences.size(); ++i) {
    point_point_correspondences_.insert(
        point_point_correspondences_.end(),
        pose_correspondences[i].begin(), pose_correspondences[i].end());
  }
}

void NonMarkovLocalization::AddSTFConstraints(ceres::Problem* problem) {
  TIME_FUNCTION
  for (size_t i = 0; i < point_point_glob_correspondences_.size(); ++i) {
    PointToPointGlobCorrespondence& correspondence =
        point_point_glob_correspondences_[i];
    if (kUseRelativeConstraints) {
      CHECK_NE(correspondence.pose_index0, correspondence.pose_index1);
      PointToPointRelativeConstraint* constraint =
          new PointToPointRelativeConstraint(
          correspondence.pose_index0, correspondence.pose_index1,
          correspondence.points0, correspondence.points1,
          correspondence.normals0, correspondence.normals1,
          localization_options_.kLaserStdDev,
          localization_options_.kPointPointCorrelationFactor);
      DynamicAutoDiffCostFunction<PointToPointRelativeConstraint,
          kDynamicDiffStride>* cost_function =
          new DynamicAutoDiffCostFunction<PointToPointRelativeConstraint,
              kDynamicDiffStride>(constraint);
      vector<double*> parameter_blocks;
      for (size_t j = 0;
           j <= max(correspondence.pose_index0, correspondence.pose_index1);
           ++j) {
        cost_function->AddParameterBlock(3);
        parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
      }
      cost_function->SetNumResiduals(2);
      problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
    } else {
      DCHECK_NE(correspondence.pose_index0, correspondence.pose_index1);
      PointToPointGlobConstraint* constraint = new PointToPointGlobConstraint(
          correspondence.pose_index0, correspondence.pose_index1,
          correspondence.points0, correspondence.points1,
          correspondence.normals0, correspondence.normals1,
          localization_options_.kLaserStdDev,
          localization_options_.kPointPointCorrelationFactor);
      problem->AddResidualBlock(
          new AutoDiffCostFunction<PointToPointGlobConstraint, 2, 3, 3>(
              constraint), NULL,
              &(pose_array_[3 * correspondence.pose_index0]),
              &(pose_array_[3 * correspondence.pose_index1]));
    }
  }
}

void NonMarkovLocalization::FindSTFCorrespondences(
    const size_t min_poses, const size_t max_poses) {
  static const size_t kMinInterPoseCorrespondence = 10;
  const float min_cosine_angle = cos(localization_options_.kMaxStfAngleError);
  // const float kMaxPoseSqDistance = sq(100.0);
  static const int kPointMatchSeparation = 4;
  const size_t poses_end = min(
      static_cast<size_t>(max_poses + 1), point_clouds_.size());
  CHECK_GT(pose_array_.size(), poses_end - 1);
  vector<vector<PointToPointGlobCorrespondence> > pose_point_correspondences(
      poses_end - min_poses);
  point_point_glob_correspondences_.clear();
  OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < poses_end; ++i) {
    vector<int> point_corrspondences(point_clouds_[i].size(), 0);
    const Vector2f source_pose_location(
        pose_array_[3 * i], pose_array_[3 * i + 1]);
    const Rotation2Df source_pose_rotation(pose_array_[3 * i + 2]);
    for (size_t j = min_poses ; j < poses_end; j += kPointMatchSeparation) {
      if (i == j) continue;
      PointToPointGlobCorrespondence correspondence;
      vector<size_t> source_point_indices;
      correspondence.pose_index0 = i;
      correspondence.pose_index1 = j;
      size_t num_stfs = 0;
      const Vector2f target_pose_location(
          pose_array_[3 * j], pose_array_[3 * j + 1]);
      const Rotation2Df target_pose_rotation(pose_array_[3 * j + 2]);
      // Ignore poses with insufficent overlap.
      // if ((target_pose_location - source_pose_location).squaredNorm() >
      //     kMaxPoseSqDistance) continue;
      Affine2f source_to_target_tf = RelativePoseTransform(i, j);
      for (size_t k = 0; k < point_clouds_[i].size();
           k += localization_options_.num_skip_readings) {
        // Ignore points that already have a valid correspondence: they do not
        // need to be considered as STF objects.
        if (observation_classes_[i][k] != kDfObservation) continue;
        if (point_corrspondences[k] >=
            localization_options_.kMaxCorrespondencesPerPoint) {
          continue;
        }
        const Vector2f point(source_to_target_tf * point_clouds_[i][k]);
        const Vector2f normal =
            Rotation2Df(pose_array_[3 * j + 2] - pose_array_[3 * i + 2]) *
            normal_clouds_[i][k];
        KDNodeValue<float, 2> neighbor_point;
        const float closest_distance = kdtrees_[j]->FindNearestPoint(
            point, localization_options_.kPointMatchThreshold, &neighbor_point);
        const float cosine_angle = neighbor_point.normal.dot(normal);
        if (closest_distance < localization_options_.kPointMatchThreshold &&
            cosine_angle > min_cosine_angle) {
          // Valid point to point match found
          correspondence.points0_indices.push_back(k);
          correspondence.points1_indices.push_back(neighbor_point.index);
          correspondence.points0.push_back(
              point_clouds_[i][k]);
          correspondence.points1.push_back(
              point_clouds_[j][neighbor_point.index]);
          correspondence.normals0.push_back(
              normal_clouds_[i][k]);
          correspondence.normals1.push_back(
              normal_clouds_[j][neighbor_point.index]);
          source_point_indices.push_back(k);
          ++num_stfs;
          ++point_corrspondences[k];
        }
      }
      if (correspondence.points0.size() > kMinInterPoseCorrespondence) {
        pose_point_correspondences[i - min_poses].push_back(correspondence);
        for (size_t j = 0; j < source_point_indices.size(); ++j) {
          observation_classes_[i][source_point_indices[j]] = kStfObservation;
        }
      }
    }
  }
  for (size_t i = 0; i < pose_point_correspondences.size(); ++i) {
    point_point_glob_correspondences_.insert(
        point_point_glob_correspondences_.end(),
        pose_point_correspondences[i].begin(),
        pose_point_correspondences[i].end());
  }
}

void NonMarkovLocalization::FindSinglePoseLtfCorrespondences(
    const vector<Vector2f>& point_cloud,
    const vector<Vector2f>& normal_cloud,
    const double* pose_array,
    vector<int>* line_correspondences_ptr,
    vector<Line2f>* ray_cast_ptr,
    vector<ObservationType>* observation_classes_ptr) {
  static CumulativeFunctionTimer ft_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invocation(&ft_);
  vector<int>& line_correspondences = *line_correspondences_ptr;
  vector<Line2f>& ray_cast = *ray_cast_ptr;
  vector<ObservationType>& observation_classes = * observation_classes_ptr;
  const Vector2f sensor_loc = Vector2f(pose_array[0], pose_array[1]) +
      Rotation2Df(pose_array[2]) *
      Vector2f(localization_options_.sensor_offset.x(),
               localization_options_.sensor_offset.y());
  float sq_max_observed_range = Sq(0.1);
  for (size_t j = 0; j < point_cloud.size();
      j += localization_options_.num_skip_readings) {
    sq_max_observed_range = max<float>(
        point_cloud[j].squaredNorm(),
        sq_max_observed_range);
  }
  const float ray_cast_range =
      min<float>(localization_options_.kMaxRange, sqrt(sq_max_observed_range));
  vector_map_.GetRayToLineCorrespondences(
      sensor_loc,
      pose_array[2],
      point_cloud,
      0.0,
      ray_cast_range,
      ray_cast_ptr,
      line_correspondences_ptr);
  CHECK_EQ(line_correspondences.size(), point_cloud.size());
  int num_ltfs = 0;
  int num_vltfs = 0;
  for (size_t j = 0; j < point_cloud.size();
      j += localization_options_.num_skip_readings) {
    const int correspondence = line_correspondences[j];
    if (correspondence >= 0) {
      const Line2f& line = ray_cast[correspondence];
      ++num_ltfs;
      if (IsValidLTFCorrespondence(
          pose_array, point_cloud[j], normal_cloud[j], line)) {
        observation_classes[j] = kLtfObservation;
        ++num_vltfs;
      }
    }
  }
  // printf("LTFs: %6d %6d %6d\n", int(point_cloud.size()), num_ltfs, num_vltfs);
}

void NonMarkovLocalization::FindLTFCorrespondences(
  const size_t min_poses, const size_t max_poses) {
  static const bool debug = false;
  point_line_correspondences_.resize(point_clouds_.size());
  ray_cast_lines_.resize(point_clouds_.size());
  const size_t num_poses = min(point_clouds_.size(), max_poses + 1);
  if (debug) printf("Finding map correspondences for poses %lu:%lu\n",
      min_poses, num_poses - 1);
  CHECK_GT(pose_array_.size(), num_poses * 3 - 1);
  OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < num_poses; ++i) {
    const size_t j = 3 * i;
    FindSinglePoseLtfCorrespondences(
        point_clouds_[i], normal_clouds_[i], &(pose_array_[j]),
        &(point_line_correspondences_[i]), &(ray_cast_lines_[i]),
        &(observation_classes_[i]));
  }
}

bool NonMarkovLocalization::IsValidLTFCorrespondence(
    const double* pose,
    const Vector2f& point,
    const Vector2f& normal,
    const Line2f& line) const {
  const Vector2f pose_location(pose[0], pose[1]);
  const Rotation2Df pose_rotation(pose[2]);
  const Vector2f point_transformed =
      pose_rotation * point + pose_location;
  const Vector2f normal_transformed = pose_rotation * normal;
  const float dot_product = fabs(line.UnitNormal().dot(normal_transformed));
  const bool angle_match =
      dot_product > cos(localization_options_.kMaxAngleError);
  if (!angle_match) {
    return false;
  }
  const float dist_from_line = line.Distance(point_transformed);
  // const float dist_from_line = line.Distance(point_transformed);
  // const Vector2f n = line.UnitNormal();
  // if (n.dot(pose_location - line.p0) *
  //     n.dot(point_transformed - line.p0) < 0.0f) {
  //   // Observations are on opposite sides of the line.
  //   return (dist_from_line < localization_options_.kMaxPointToLineDistance);
  // }
  return (dist_from_line < localization_options_.kMaxPointToLineDistance);
}

void NonMarkovLocalization::AddSinglePoseLTFConstraints(
    const size_t pose_index,
    const vector<Line2f>& ray_cast,
    const vector<int>& correspondences,
    const vector<ObservationType>& observation_classes,
    const vector<Vector2f>& point_cloud,
    double* pose_array,
    vector<PointToLineConstraint*>* constraints,
    ceres::Problem* problem) {
  DCHECK_EQ(point_cloud.size(), correspondences.size());
  DCHECK_EQ(point_cloud.size(), observation_classes.size());

  vector<Vector2f> line_normals(point_cloud.size(), Vector2f::Zero());
  vector<float> line_offsets(point_cloud.size(), 0.0);
  OMP_PARALLEL_FOR
  for (size_t j = 0; j < point_cloud.size(); ++j) {
    if (observation_classes[j] == kLtfObservation) {
      const int& correspondence = correspondences[j];
      DCHECK_LT(correspondence, ray_cast.size());
      const Line2f& line = ray_cast[correspondence];
      line_normals[j] = line.UnitNormal();
      line_offsets[j] = -line_normals[j].dot(line.p0);
    }
  }
  vector<bool> ltf_valid(observation_classes.size(), false);
  for (size_t k = 0; k < observation_classes.size(); ++k) {
    ltf_valid[k] = (observation_classes[k] == kLtfObservation);
  }
  if (kUseRelativeConstraints) {
    PointToLineRelativeConstraint* constraint =
        new PointToLineRelativeConstraint(
            pose_index, point_cloud, line_normals, line_offsets, ltf_valid,
            localization_options_.kLaserStdDev,
            localization_options_.kPointMapCorrelationFactor);
    DynamicAutoDiffCostFunction<PointToLineRelativeConstraint,
        kDynamicDiffStride>* cost_function =
        new DynamicAutoDiffCostFunction<PointToLineRelativeConstraint,
        kDynamicDiffStride>(constraint);
    vector<double*> parameter_blocks;
    for (size_t i = 0; i <= pose_index; ++i) {
      cost_function->AddParameterBlock(3);
      parameter_blocks.push_back(&(relative_pose_array_[3 * i]));
    }
    cost_function->SetNumResiduals(1);
    problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
  } else {
    PointToLineGlobConstraint* constraint = new PointToLineGlobConstraint(
      pose_index, point_cloud, line_normals, line_offsets, ltf_valid,
      localization_options_.kLaserStdDev,
      localization_options_.kPointMapCorrelationFactor);
    problem->AddResidualBlock(
        new AutoDiffCostFunction<PointToLineGlobConstraint, 1, 3>(
            constraint), NULL, pose_array);
  }
}

void NonMarkovLocalization::AddLTFConstraints(
    const size_t min_poses, const size_t max_poses, ceres::Problem* problem) {
  TIME_FUNCTION
  DCHECK_EQ(point_clouds_.size(), ray_cast_lines_.size());
  DCHECK_EQ(point_clouds_.size(), point_line_correspondences_.size());
  DCHECK_GT(point_clouds_.size(), min_poses);
  point_line_constraints_.clear();
  const size_t num_point_clouds = min(point_clouds_.size(), max_poses + 1);
  for (size_t i = min_poses; i < num_point_clouds; ++i) {
    AddSinglePoseLTFConstraints(
        i, ray_cast_lines_[i], point_line_correspondences_[i],
        observation_classes_[i], point_clouds_[i], &(pose_array_[3 * i]),
        &point_line_constraints_, problem);
  }
}

double NonMarkovLocalization::ObservationLikelihood(
    const Pose2Df& pose,
    const PointCloudf& point_cloud_e,
    const NormalCloudf& normal_cloud) {
  double log_likelihood = 0.0;
  const Vector2f pose_location(pose.translation);
  const Affine2f pose_transform =
      Translation2Df(pose_location) * Rotation2Df(pose.angle);
  const double pose_array[3] = {
    pose.translation.x(),
    pose.translation.y(),
    pose.angle
  };
  vector<int> line_correspondences;
  vector<Line2f> ray_cast;
  vector<ObservationType> obs_classes(point_cloud_e.size(), kDfObservation);
  FindSinglePoseLtfCorrespondences(
      point_cloud_e, normal_cloud, pose_array,
      &line_correspondences, &ray_cast, &obs_classes);
  static const double kCorrelationFactor = 0.05;
  static const double kObstacleWeight =
      -sq(1.0 / localization_options_.kLaserStdDev * kCorrelationFactor);
  for (size_t j = 0; j < point_cloud_e.size(); ++j) {
    const int correspondence = line_correspondences[j];
    if (correspondence < 0) {
      log_likelihood += kObstacleWeight;
      continue;
    }
    const Line2f& line = ray_cast[correspondence];
    const Vector2f& point = point_cloud_e[j];
    const Vector2f point_global = pose_transform * point;
    const Vector2f line_dir = line.Dir();
    const Vector2f line_normal = Perp(line_dir);
    const float line_offset = -line_normal.dot(line.p0);
    const bool valid_correspondence =
        (point_global - line.p0).dot(point_global - line.p1) < 0.0;
    if (valid_correspondence) {
      LTSConstraint constraint(
              0, point, line_normal, line_dir, line.p0, line.p1,
              line_offset, localization_options_.kLaserStdDev,
              kCorrelationFactor);
      double residual = 0.0;
      constraint(pose_array, &residual);
      log_likelihood += -sq(residual);
    } else {
      log_likelihood += kObstacleWeight;
    }
  }
  return (exp(log_likelihood));
}

void NonMarkovLocalization::SensorResettingResample(
    const vector<Vector2f>& point_cloud,
    const vector<Vector2f>& normal_cloud,
    const size_t num_samples,
    float radial_stddev,
    float tangential_stddev,
    float angular_stddev,
    void (*callback)(
        const std::vector<double>& poses,
        const PointCloudf& point_cloud_e,
        const NormalCloudf& normal_cloud,
        const vector<LTSConstraint*>& constraints),
    vector<Pose2Df>* poses_ptr,
    vector<double>* pose_weights_ptr,
    vector<LTSConstraint*>* constraints_ptr) {
  static const bool kUseTrustRegion = true;
  static const bool kSampleLocally = false;
  vector<Pose2Df>& poses = *poses_ptr;
  vector<double>& pose_weights = *pose_weights_ptr;
  vector<double> pose_array;
  const Pose2Df latest_pose = GetLatestPose();

  if (kSampleLocally) {
    pose_array.resize(3 * num_samples);
    poses.resize(num_samples);
    pose_weights.resize(num_samples);
    // Sample around the MLE.
    util_random::Random rand;
    for (size_t i = 0; i < num_samples; ++i) {
      poses[i] = latest_pose;
      const Vector2f random_translation = Rotation2Df(latest_pose.angle) *
          Vector2f(rand.Gaussian(0, radial_stddev),
                   rand.Gaussian(0, tangential_stddev));
      const float random_rotation = rand.Gaussian(0, angular_stddev);
      poses[i].angle = AngleMod(poses[i].angle + random_rotation);
      poses[i].translation = poses[i].translation + random_translation;
      pose_array[3 * i + 0] = poses[i].translation.x();
      pose_array[3 * i + 1] = poses[i].translation.y();
      pose_array[3 * i + 2] = poses[i].angle;
    }
  } else {
    static const float kWindowHalfWidth = 2.5;
    static const float kLengthResolution = 0.5;
    static const float kAngularResolution = DegToRad(30.0);
    pose_array.clear();
    poses.clear();
    for (float x = latest_pose.translation.x() - kWindowHalfWidth;
        x < latest_pose.translation.x() + kWindowHalfWidth;
        x += kLengthResolution) {
      for (float y = latest_pose.translation.y() - kWindowHalfWidth;
          y < latest_pose.translation.y() + kWindowHalfWidth;
          y += kLengthResolution) {
        for (float t = -M_PI; t < M_PI; t += kAngularResolution) {
          const Pose2Df pose(t, x, y);
          poses.push_back(pose);
          pose_array.push_back(pose.translation.x());
          pose_array.push_back(pose.translation.y());
          pose_array.push_back(pose.angle);
        }
      }
    }
    pose_weights.resize(poses.size());
  }

  printf("Initial num samples: %d\n", static_cast<int>(poses.size()));
  // Run gradient descent over all the samples
  static const int kNumIterations = 4;
  static const double kCorrelationFactor = 0.05;
  for (int i = 0; i < kNumIterations; ++i) {
    // Detect duplicate samples.
    static const float kMinLengthSeparation = 0.1;
    static const float kMinAngularSeparation = DegToRad(10.0);
    for (size_t j = 0; j < poses.size(); ++j) {
      const Vector2f p1(pose_array[3 * j], pose_array[3 * j + 1]);
      const float& a1 = pose_array[3 * j + 2];
      for (size_t k = j + 1; k < poses.size(); ++k) {
        const Vector2f p2(pose_array[3 * k], pose_array[3 * k + 1]);
        const float& a2 = pose_array[3 * k + 2];
        const float length_separation = (p1 - p2).norm();
        const float angular_separation = AngleDist(a1, a2);
        if (angular_separation < kMinAngularSeparation &&
            length_separation < kMinLengthSeparation) {
          poses.erase(poses.begin() + k);
          pose_array.erase(
              pose_array.begin() + 3 * k, pose_array.begin() + 3 * k + 3);
          pose_weights.erase(pose_weights.begin() + k);
          --k;
        }
      }
    }
    // Evaluate the correspondences of the samples.
    vector<vector<Line2f> > ray_cast_lines(poses.size());
    vector<vector<int> > line_correspondences(poses.size());
    vector<ObservationType> obs_classes(point_cloud.size(), kDfObservation);
    // OMP_PARALLEL_FOR
    for (size_t j = 0; j < poses.size(); ++j) {
      FindSinglePoseLtfCorrespondences(
          point_cloud, normal_cloud, &(pose_array[3 * j]),
          &(line_correspondences[j]), &(ray_cast_lines[j]), &obs_classes);
    }

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options.minimizer_progress_to_stdout = false;
    solver_options.num_threads = localization_options_.kNumThreads;
    solver_options.max_num_iterations = 2;
    solver_options.function_tolerance = 0.001;
    // solver_options.update_state_every_iteration = true;
    solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    if (kUseTrustRegion) {
      solver_options.minimizer_type = ceres::TRUST_REGION;
      solver_options.initial_trust_region_radius = 0.2;
      solver_options.max_trust_region_radius = 1.0;
    } else {
      solver_options.minimizer_type = ceres::LINE_SEARCH;
      solver_options.line_search_direction_type = ceres::STEEPEST_DESCENT;
      solver_options.line_search_interpolation_type = ceres::CUBIC;
    }
    ceres::Problem problem;
    vector<PointToLineConstraint*> ltf_constraints;
    vector<LTSConstraint*> visibility_constraints;
    // Add LTF constraints for all poses.
    for (size_t k = 0; k < poses.size(); ++k) {
      const Vector2f pose_location(
        pose_array[3 * k + 0], pose_array[3 * k + 1]);
      const Affine2f pose_transform =
          Translation2Df(pose_location) * Rotation2Df(pose_array[3 * k + 2]);
      for (size_t j = 0; j < point_cloud.size(); ++j) {
        const int correspondence = line_correspondences[k][j];
        if (correspondence < 0) continue;
        const Line2f& line = ray_cast_lines[k][correspondence];
        const Vector2f& point = point_cloud[j];
        const Vector2f point_global = pose_transform * point;
        const Vector2f line_dir = line.Dir();
        const Vector2f line_normal = Perp(line_dir);
        const float line_offset = -line_normal.dot(line.p0);
        const bool valid_correspondence =
            (point_global - line.p0).dot(point_global - line.p1) < 0.0f;
        if (true || valid_correspondence) {
          visibility_constraints.push_back(
              new LTSConstraint(
                  k, point, line_normal, line_dir, line.p0, line.p1,
                  line_offset, localization_options_.kLaserStdDev,
                  kCorrelationFactor));
          problem.AddResidualBlock(
                new AutoDiffCostFunction<LTSConstraint, 1, 3>(
                  visibility_constraints.back()),
                  NULL,
                  &(pose_array[3 * k]));
        }
      }
      if (false && kUseTrustRegion) {
        AnchorConstraint* constraint = new AnchorConstraint(
            poses[k].translation.x(), poses[k].translation.y(), 1.0);
        problem.AddResidualBlock(
            new AutoDiffCostFunction<AnchorConstraint, 2, 3>(
                constraint), NULL, &(pose_array[3 * k]));
      }
    }
    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    if (i == kNumIterations - 1 && constraints_ptr != NULL) {
      // Copy over the constraints.
      vector<LTSConstraint*>& return_constraints = *constraints_ptr;
      return_constraints.resize(visibility_constraints.size());
      for (size_t j = 0; j < visibility_constraints.size(); ++j) {
        return_constraints[j] = new LTSConstraint(*visibility_constraints[j]);
      }
    }
  }
  printf("Final num samples: %d\n", static_cast<int>(poses.size()));
  // Copy over the optimized poses and evaluate the weights of all the poses.
  CHECK_EQ(3 * poses.size(), pose_array.size());
  OMP_PARALLEL_FOR
  for (size_t j = 0; j < poses.size(); ++j) {
    poses[j].translation.x() = pose_array[3 * j + 0];
    poses[j].translation.y() = pose_array[3 * j + 1];
    poses[j].angle = pose_array[3 * j + 2];
    pose_weights[j] = ObservationLikelihood(
        poses[j], point_cloud, normal_cloud);
  }

}

void NonMarkovLocalization::AddVisibilityConstraints(
    const size_t min_poses, const size_t max_poses, ceres::Problem* problem) {
  visibility_constraints_.clear();
  const size_t num_point_clouds = min(point_clouds_.size(), max_poses + 1);

  vector<VisibilityGlobConstraint*> constraints(point_clouds_.size());
  OMP_PARALLEL_FOR
  for (size_t i = min_poses; i < num_point_clouds; ++i) {
    const vector<Vector2f>& point_cloud = point_clouds_[i];
    const vector<Vector2f>& normal_cloud = normal_clouds_[i];
    const Vector2f pose_location(
        pose_array_[3 * i + 0], pose_array_[3 * i + 1]);
    const Rotation2Df pose_rotation = Rotation2Df(pose_array_[3 * i + 2]);
    const Affine2f pose_transform =
        Translation2Df(pose_location) * pose_rotation;
    vector<Vector2f> points;
    vector<Vector2f> line_normals;
    vector<Vector2f> line_p0s;
    vector<Vector2f> line_p1s;
    vector<float> line_offsets;
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      if (observation_classes_[i][j] == kDfObservation) continue;
      const int correspondence = point_line_correspondences_[i][j];
      if (correspondence < 0) continue;
      DCHECK_LT(correspondence, ray_cast_lines_[i].size());
      const Line2f& line = ray_cast_lines_[i][correspondence];
      const Vector2f& point = point_cloud[j];
      const Vector2f point_global = pose_transform * point;
      const Vector2f normal = pose_rotation * normal_cloud[j];
      const Vector2f line_dir = line.Dir();
      const Vector2f line_normal = Perp(line_dir);
      const float line_offset = -line_normal.dot(line.p0);
      if (fabs(line_normal.dot(normal)) > cos(DegToRad(40.0)) &&
          (line_normal.dot(pose_location) + line_offset) *
          (line_normal.dot(point_global) + line_offset) < -FLT_MIN &&
          (LiesAlongLineSegment(line.p0, line.p1, point_global)) &&
          (true || LiesAlongLineSegment(line.p0, line.p1, pose_location))) {
        if (kUseRelativeConstraints) {
          VisibilityRelativeConstraint* constraint =
              new VisibilityRelativeConstraint(
                  i, point, line_normal, line_dir, line.p0, line.p1,
                  line_offset, localization_options_.kLaserStdDev,
                  localization_options_.kVisibilityCorrelationFactor);
          DynamicAutoDiffCostFunction<VisibilityRelativeConstraint,
              kDynamicDiffStride>* cost_function = new
              DynamicAutoDiffCostFunction<VisibilityRelativeConstraint,
              kDynamicDiffStride>(constraint);
          vector<double*> parameter_blocks;
          for (size_t j = 0; j <= i; ++j) {
            cost_function->AddParameterBlock(3);
            parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
          }
          cost_function->SetNumResiduals(1);
          problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
        } else {
          points.push_back(point);
          line_normals.push_back(line_normal);
          line_p0s.push_back(line.p0);
          line_p1s.push_back(line.p1);
          line_offsets.push_back(line_offset);
        }
      }
    }
    if (points.size() == 0) continue;
    constraints[i] = new VisibilityGlobConstraint(
        i, points, line_normals, line_p0s, line_p1s,
        line_offsets, localization_options_.kLaserStdDev,
        localization_options_.kVisibilityCorrelationFactor);
  }
  for (size_t i = 0; i < constraints.size(); ++i) {
    if (constraints[i] == NULL) continue;
    visibility_constraints_.push_back(constraints[i]);
    problem->AddResidualBlock(
        new AutoDiffCostFunction<VisibilityGlobConstraint, 1, 3>(
            constraints[i]), NULL, &(pose_array_[3 * i]));
  }
}

void NonMarkovLocalization::AddPoseConstraints(
    const size_t min_poses, const size_t max_poses,
    const std::vector<Pose2Df>& poses,
    ceres::Problem* problem) {
  static const bool debug = false;
  TIME_FUNCTION
  static const float kEpsilon = 1e-6;
  for (size_t i = min_poses + 1; i <= max_poses && i < poses.size(); ++i) {
    // Create a new pose constraint residual from the odometry constraint
    // between pose i and pose i-1, and add it to the Ceres problem.
    const Vector2f translation(poses[i].translation - poses[i-1].translation);
    Vector2f radial_direction;
    Vector2f tangential_direction;
    float rotation;
    Matrix2f axis_transform;
    float radial_translation = 0.0;
    if (fabs(translation.x()) < kEpsilon && fabs(translation.y()) < kEpsilon) {
      radial_direction = Vector2f(cos(poses[i].angle), sin(poses[i].angle));
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = AngleMod(poses[i].angle - poses[i-1].angle);
      axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
      axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
      radial_translation = 0.0;
    } else {
      radial_direction = Vector2f(
          (Rotation2Df(-poses[i - 1].angle) * translation).normalized());
      tangential_direction = Vector2f(Rotation2Df(M_PI_2) * radial_direction);
      rotation = AngleMod(poses[i].angle - poses[i-1].angle);
      axis_transform.block<1, 2>(0, 0) = radial_direction.transpose();
      axis_transform.block<1, 2>(1, 0) = tangential_direction.transpose();
      radial_translation = translation.norm();
    }
    const float radial_std_dev = Clamp<float>(
        localization_options_.kOdometryRadialStdDevRate * radial_translation,
        localization_options_.kOdometryTranslationMinStdDev,
        localization_options_.kOdometryTranslationMaxStdDev);
    const float tangential_std_dev = Clamp<float>(
        localization_options_.kOdometryTangentialStdDevRate *
            radial_translation,
        localization_options_.kOdometryTranslationMinStdDev,
        localization_options_.kOdometryTranslationMaxStdDev);
    const float angular_std_dev = Clamp<float>(
        localization_options_.kOdometryAngularStdDevRate * fabs(rotation),
        localization_options_.kOdometryAngularMinStdDev,
        localization_options_.kOdometryAngularMaxStdDev);
    if (debug) {
      printf("Adding pose constraint %d:%d @ 0x%lx : 0x%lx\n",
             static_cast<int>(i - 1), static_cast<int>(i),
             reinterpret_cast<uint64_t>(&(pose_array_[3 * i - 3])),
             reinterpret_cast<uint64_t>(&(pose_array_[3 * i])));
    }
    if (kUseRelativeConstraints) {
      RelativePoseConstraint* constraint = new RelativePoseConstraint(
          i - 1, i, axis_transform, radial_std_dev, tangential_std_dev,
          angular_std_dev, radial_translation, rotation);
      DynamicAutoDiffCostFunction<RelativePoseConstraint, kDynamicDiffStride>*
          cost_function = new DynamicAutoDiffCostFunction<
              RelativePoseConstraint, kDynamicDiffStride>(constraint);
      vector<double*> parameter_blocks;
      for (size_t j = 0; j <= i; ++j) {
        cost_function->AddParameterBlock(3);
        parameter_blocks.push_back(relative_pose_array_.data() + 3 * j);
      }
      cost_function->SetNumResiduals(3);
      problem->AddResidualBlock(cost_function, NULL, parameter_blocks);
    } else {
      problem->AddResidualBlock(
        new AutoDiffCostFunction<PoseConstraint, 3, 3, 3>(
          new PoseConstraint(axis_transform, radial_std_dev, tangential_std_dev,
                            angular_std_dev, radial_translation, rotation)),
          NULL, &(pose_array_[3 * i - 3]), &(pose_array_[3 * i]));
    }
  }
}

void NonMarkovLocalization::BuildKDTrees(
    const vector< PointCloudf >& point_clouds,
    const vector< NormalCloudf >& normal_clouds) {
  CHECK_EQ(point_clouds.size(), normal_clouds.size());
  kdtrees_.resize(point_clouds.size(), NULL);
  const unsigned int num_point_clouds = point_clouds.size();
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_point_clouds; ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    const NormalCloudf& normal_cloud = normal_clouds[i];
    CHECK_EQ(point_cloud.size(), normal_cloud.size());
    vector<KDNodeValue<float, 2> > values(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      values[j].index = j;
      values[j].point = point_cloud[j];
      values[j].normal = normal_cloud[j];
    }
    if (values.size() > 0)
      kdtrees_[i] = new KDTree<float, 2>(values);
    else
      kdtrees_[i] = new KDTree<float, 2>();
  }
}

void NonMarkovLocalization::ResetGlobalPoses(
    const size_t start, const size_t end,
    const vector< Pose2Df >& poses) {
  CHECK_LT(end, poses.size());
  for (size_t i = start; i <= end; ++i) {
    const Vector2f& p0 = poses[i - 1].translation;
    const Vector2f& p1 = poses[i].translation;
    const double dr = AngleMod(poses[i].angle - poses[i - 1].angle);
    const Vector2f dp = Rotation2Df(-poses[i - 1].angle) * (p1 - p0);
    const Vector2f p1_new =
        Vector2f(pose_array_[3 * (i - 1) + 0], pose_array_[3 * (i - 1) + 1]) +
        Rotation2Df(pose_array_[3 * (i - 1) + 2]) * dp;
    pose_array_[3 * i + 0] = p1_new.x();
    pose_array_[3 * i + 1] = p1_new.y();
    pose_array_[3 * i + 2] = pose_array_[3 * (i - 1) + 2] + dr;
  }
}

void NonMarkovLocalization::ComputeLostMetric() {
  size_t num_visibility_constraints = 0;
  vector<double> residuals(visibility_constraints_.size());
  OMP_PARALLEL_FOR
  for(size_t i = 0; i < visibility_constraints_.size(); ++i) {
    const VisibilityGlobConstraint constraint(
        visibility_constraints_[i]->pose_index,
        visibility_constraints_[i]->points,
        visibility_constraints_[i]->line_normals,
        visibility_constraints_[i]->line_p1s,
        visibility_constraints_[i]->line_p2s,
        visibility_constraints_[i]->line_offsets,
        1.0,
        1.0);
    num_visibility_constraints += constraint.points.size();
    double residual = 0.0;
    const double* pose = &(pose_array_[3 * constraint.pose_index]);
    constraint(pose, &residual);
    residuals[i] = sq(residual);
  }
  // The lost metric is the RMSE of the visibility constraints.
  const double visibility_total_sq_error =
      std::accumulate(residuals.begin(), residuals.end(), 0.0);
  lost_metric_ = sqrt(
    visibility_total_sq_error / static_cast<double>(num_visibility_constraints));
  if (false) {
    printf(" %f %f %lu\n",
          lost_metric_, visibility_total_sq_error, num_visibility_constraints);
  }
}

float NonMarkovLocalization::GetLostMetric() const {
  return lost_metric_;
}

void NonMarkovLocalization::Update() {
  // FunctionTimer ft(__FUNCTION__);
  static const bool debug = false;
  static EnmlTiming timing;
  const double t_update_start = GetMonotonicTime();
  // Accepts:
  //  1. Non-Markov Localization paramters.
  //  2. Vector map.
  //  3. Mutable vector of point clouds of observations from every pose node.
  //  4. Mutable vector of normal clouds of observations from every pose node.
  //  5. Mutable vector of pose nodes.
  // Spawn a new thread for the update
  ceres::Solver::Options solver_options;
  SetSolverOptions(localization_options_, &solver_options);

  vector<double> gradients;
  vector<Matrix2f> covariances;
  int repeat_iterations = 0;
  size_t min_poses = 0;
  const size_t max_poses = point_clouds_.size() - 1;
  ResetObservationClasses(min_poses, max_poses);
  if (point_clouds_.size() < 2) return;
  // Copy over the unoptimized poses.
  for (size_t i = 0; i < poses_.size(); ++i) {
    const int j = 3 * i;
    pose_array_[j + 0] = poses_[i].translation.x();
    pose_array_[j + 1] = poses_[i].translation.y();
    pose_array_[j + 2] = poses_[i].angle;
  }
  bool converged = false;
  for (int i = 0; !converged && i < localization_options_.kMaxRepeatIterations;
       ++i) {
    timing.enml_iterations++;
    ceres::Problem problem;
    if (localization_options_.limit_history) {
      static const int kOpenPoses = 40;
      if (poses_.size() > kOpenPoses) {
        const int N = poses_.size() - kOpenPoses;
        min_poses = N;
        for (int j = 0; j < N; ++j) {
          problem.AddParameterBlock(&pose_array_[3 * j], 3);
        }
      }
    }
    // FunctionTimer callback(StringPrintf("Iteration %d", i).c_str());
    if (debug) {
      printf("Localization update iteration %5d, poses %5lu:%5lu\n",
             i, min_poses, max_poses);
    }
    const double t0 = GetMonotonicTime();
    ResetObservationClasses(min_poses, max_poses);
    FindLTFCorrespondences(min_poses, max_poses);
    const double t1 = GetMonotonicTime();
    timing.find_ltfs += (t1 - t0);
    FindSTFCorrespondences(min_poses, max_poses);
    const double t2 = GetMonotonicTime();
    timing.find_stfs += (t2 - t1);
    if (kUseRelativeConstraints) {
      RecomputeRelativePoses();
    }
    // First add the visibility constraints and mark which ray cast line
    // segments have visibility constraint violations.
    // For those line segments with a large mean visibility constraint error
    // as well as sufficiently many visibility_constraints, disregard the
    // LTF constraints that oppose them.
    if (localization_options_.use_visibility_constraints) {
      AddVisibilityConstraints(min_poses, max_poses, &problem);
    }
    AddLTFConstraints(min_poses, max_poses, &problem);
    if (localization_options_.use_STF_constraints) {
      AddSTFConstraints(&problem);
    }
    AddPoseConstraints(min_poses, max_poses, poses_, &problem);
    if (debug) {
      printf("%9lu LTF constraints, %9lu STF constraints\n",
             point_line_constraints_.size(), point_point_constraints_.size());
    }
    ceres::Solver::Summary summary;
    // The first pose should be constant since it's a "given" for the current
    // non-Markov episode.
    if (kUseRelativeConstraints) {
      problem.SetParameterBlockConstant(&(relative_pose_array_[0]));
    } else {
      problem.SetParameterBlockConstant(&(pose_array_[0]));
    }
    const double t3 = GetMonotonicTime();
    ceres::Solve(solver_options, &problem, &summary);
    const double t4 = GetMonotonicTime();
    timing.add_constraints += (t3 - t2);
    timing.solver += (t4 - t3);
    timing.ceres_iterations +=
        summary.num_successful_steps + summary.num_unsuccessful_steps;
    // timing.ceres_iterations += summary.iterations.size();
    // std::cout << summary.FullReport();
    // std::cout << summary.BriefReport() << "\n";
    if (kUseRelativeConstraints) {
      RecomputeAbsolutePoses();
    }
    if (summary.termination_type == ceres::CONVERGENCE) {
      ++repeat_iterations;
      if (repeat_iterations > localization_options_.kNumRepeatIterations) {
        converged = true;
      }
    }
    if (false) {
      ceres::CRSMatrix jacobian;
      // Evaluate Jacobians
      ceres::Problem::EvaluateOptions evaluate_options;
      evaluate_options.num_threads = localization_options_.kNumThreads;
      for (size_t j = min_poses; j <= max_poses; ++j) {
        if (kUseRelativeConstraints) {
          evaluate_options.parameter_blocks.push_back(
              &(relative_pose_array_[3 * j]));
        } else {
          evaluate_options.parameter_blocks.push_back(&(pose_array_[3 * j]));
        }
      }
      problem.Evaluate(evaluate_options, NULL, NULL, &gradients, &jacobian);
      CHECK_EQ(gradients.size(), pose_array_.size());
    }
    if (false) {
      ceres::Covariance::Options options;
      options.num_threads =  localization_options_.kNumThreads;
      ceres::Covariance cov(options);
      covariances.resize(poses_.size());
      vector<pair<const double*, const double*> > blocks(
          poses_.size(), make_pair<const double*, const double*>(NULL, NULL));
      for (size_t i = 0; i < poses_.size(); ++i) {
        blocks[i] = make_pair<const double*, const double*>(
            &pose_array_[3 * i], &pose_array_[3 * i]);
      }
      cov.Compute(blocks, &problem);
      for (size_t i = 0; i < poses_.size(); ++i) {
        double values[] = {
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        };
        cov.GetCovarianceBlock(
            &pose_array_[3 * i],
            &pose_array_[3 * i],
            &values[0]);
        covariances[i](0, 0) = values[0];
        covariances[i](1, 0) = values[1];
        covariances[i](0, 1) = values[3];
        covariances[i](1, 1) = values[4];
      }
    }
    timing.residuals += summary.residual_evaluation_time_in_seconds;
    timing.jacobians += summary.jacobian_evaluation_time_in_seconds;
    timing.preprocessor += summary.preprocessor_time_in_seconds;
    timing.linear_solver += summary.linear_solver_time_in_seconds;
    if (summary.termination_type == ceres::FAILURE ||
        summary.termination_type == ceres::USER_FAILURE) {
      std::cout << "\nEnML Ceres failure, report follows:\n"
                << summary.FullReport();
    }
    timing.ceres_full_report = summary.FullReport();
    if (converged || i + 1 == localization_options_.kMaxRepeatIterations) {
      // This is the final iteration of the EnML solver.
      ComputeLostMetric();
    }
  }
  if (localization_options_.CorrespondenceCallback != NULL) {
    localization_options_.CorrespondenceCallback(
        pose_array_, point_clouds_, normal_clouds_, ray_cast_lines_,
        point_line_correspondences_, point_point_glob_correspondences_,
        observation_classes_, visibility_constraints_,
        gradients, covariances, poses_, min_poses, max_poses);
  }
  // Copy over the optimized poses.
  for (size_t i = 0; i < poses_.size(); ++i) {
    const int j = 3 * i;
    poses_[i].translation.x() = pose_array_[j + 0];
    poses_[i].translation.y() = pose_array_[j + 1];
    poses_[i].angle = AngleMod(pose_array_[j + 2]);
  }
  // Trim non-Markov episode.
  const int episode_start = FindEpisodeStart();
  if (localization_options_.log_poses) {
    logged_episode_lengths_.push_back(
        static_cast<int>(poses_.size()) - episode_start);
  }
  if (episode_start > 0) {
    TrimEpisode(episode_start);
  }
  latest_mle_pose_.Set(poses_.back());
  const double t_update_end = GetMonotonicTime();
  timing.update += (t_update_end - t_update_start);
}

void NonMarkovLocalization::SaveEpisodeData() {
  static int last_run = 0;
  ++last_run;
  if (last_run < 5) return;
  last_run = 0;
  const size_t num_nodes = poses_.size();
  ScopedFile fid(fopen("constraints.txt", "w"));
  fprintf(fid(), "TotalPoints, LTF_Points\n");
  for (size_t i = 0; i < num_nodes; ++i) {
    size_t num_ltfs = 0;
    for (size_t j = 0; j < observation_classes_[i].size(); ++j) {
      if (observation_classes_[i][j] == kLtfObservation) {
        ++num_ltfs;
      }
    }
    fprintf(fid(), "%6lu, %6lu\n",
            point_clouds_[i].size(), num_ltfs);
  }
}

void NonMarkovLocalization::CountConstraints(
    size_t node_index, size_t* num_points_ptr,
    size_t* num_ltf_constraints_ptr) {
  size_t& num_points = *num_points_ptr;
  size_t& num_ltf_constraints = *num_ltf_constraints_ptr;
  num_points = point_clouds_[node_index].size();
  num_ltf_constraints = 0;
  for (size_t i = 0; i < num_points; ++i) {
    if (observation_classes_[node_index][i] == kLtfObservation) {
      ++num_ltf_constraints;
    }
  }
}

int NonMarkovLocalization::FindEpisodeStart() {
  static const bool debug = false;
  if (false) SaveEpisodeData();

  // Search backwards to find the first occurence of a node with an LTF ratio
  // greater than the threshold.
  const size_t num_nodes = poses_.size();
  int episode_start = num_nodes - 1;
  size_t num_points = 0;
  size_t num_ltf_constraints = 0;
  float ltf_ratio = 0.0;
  const size_t kMinEpisodeLength =
      static_cast<size_t>(localization_options_.kMinEpisodeLength);
  for (; episode_start >= 0 && ltf_ratio < localization_options_.kMinLtfRatio;
       --episode_start) {
    CountConstraints(episode_start, &num_points, &num_ltf_constraints);
    ltf_ratio = static_cast<float>(num_ltf_constraints) /
        static_cast<float>(num_points);
    if (debug) {
      printf("Node %d: %6lu %6lu %f\n",
             episode_start, num_points, num_ltf_constraints, ltf_ratio);
    }
  }
  if (static_cast<int>(num_nodes) - episode_start >
      localization_options_.kMaxHistory) {
    episode_start = num_nodes - localization_options_.kMaxHistory;
  }

  if (episode_start + kMinEpisodeLength > num_nodes) {
    // The detected episode length is smaller than the minimum threshold.
    if (kMinEpisodeLength <= num_nodes) {
      // There are more nodes than the minimum threshold; select the last set.
      episode_start = num_nodes - kMinEpisodeLength;
    } else {
      // There are fewer nodes than the minimum threshold; select all of them.
      episode_start = 0;
    }
  }
  return (episode_start);
}

void NonMarkovLocalization::TrimEpisode(const int pose_index) {
  static const bool debug = false;
  if (debug) {
    printf("Trimming %d nodes out of %lu\n", pose_index, poses_.size());
  }

  if (localization_options_.log_poses) {
    vector<Pose2Df>& logged_poses = logged_poses_.GetLock();
    vector<timespec> &logged_stamps = logged_stamps_.GetLock();
    logged_poses.insert(logged_poses.end(), poses_.begin(),
                        poses_.begin() + pose_index);
    logged_stamps.insert(logged_stamps.end(), timestamps_.begin(), timestamps_.begin() + pose_index);
    logged_stamps_.Unlock();
    logged_poses_.Unlock();
  }

  // Need to trim the nodes list to the latest episode.
  pose_ids_.erase(pose_ids_.begin(), pose_ids_.begin() + pose_index);
  poses_.erase(poses_.begin(), poses_.begin() + pose_index);
  timestamps_.erase(timestamps_.begin(), timestamps_.begin() + pose_index);
  kdtrees_.erase(kdtrees_.begin(), kdtrees_.begin() + pose_index);
  normal_clouds_.erase(normal_clouds_.begin(),
                         normal_clouds_.begin() + pose_index);
  point_clouds_.erase(point_clouds_.begin(),
                        point_clouds_.begin() + pose_index);
  observation_classes_.erase(observation_classes_.begin(),
                             observation_classes_.begin() + pose_index);
  if (debug) printf("pose_array_ before:%lu ", pose_array_.size());
  pose_array_.erase(pose_array_.begin(),
                    pose_array_.begin() + (3 * pose_index));
  if (debug) printf("after:%lu\n", pose_array_.size());
}


void NonMarkovLocalization::SensorUpdate(
    const PointCloudf& point_cloud, const NormalCloudf& normal_cloud, const timespec& laser_time) {
  static const bool debug = false;
  const double t_now = GetMonotonicTime();
  const bool has_moved =
      (pending_rotation_ > 0.0 && pending_translation_ > 0.0);
  const bool force_update = has_moved &&
      (t_now > t_last_update_ + localization_options_.max_update_period);
  if (pending_rotation_ > localization_options_.minimum_node_rotation ||
      pending_translation_ > localization_options_.minimum_node_translation ||
      force_update) {
    // Add to Pending nodes.
    AddPose(point_cloud, normal_cloud, pending_relative_pose_, laser_time);
    t_last_update_ = t_now;
  } else if (debug) {
    printf("Ignoring sensor data, trans:%f rot:%f\n",
           pending_translation_, RadToDeg(pending_rotation_));
  }
}

void NonMarkovLocalization::OdometryUpdate(
    const float dx, const float dy, const float d_theta) {
  const Vector2f delta(dx, dy);
  pending_relative_pose_.angle =
      AngleMod(pending_relative_pose_.angle + d_theta);
  const Rotation2Df rotation(pending_relative_pose_.angle);
  pending_relative_pose_.translation += (rotation * delta);
  pending_translation_ += delta.norm();
  pending_rotation_ += fabs(d_theta);
}

void NonMarkovLocalization::ClearPoses() {
  pose_array_.clear();
  poses_.clear();
  timestamps_.clear();
  pose_ids_.clear();
  point_clouds_.clear();
  normal_clouds_.clear();
  observation_classes_.clear();
  kdtrees_.clear();
  latest_mle_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  latest_pending_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  pending_relative_pose_.Clear();
  pending_rotation_ = 0.0;
  pending_translation_ = 0.0;
  pending_normal_clouds_.clear();
  pending_point_clouds_.clear();
  pending_relative_poses_.clear();
  pending_stamps_.clear();
  latest_pending_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
  latest_mle_pose_.Set(Pose2Df(0.0, Vector2f(0.0, 0.0)));
}

void NonMarkovLocalization::AddPendingPoseNodes()
{
  const size_t num_old_point_clouds = point_clouds_.size();
  const size_t num_old_normal_clouds = normal_clouds_.size();

  for (size_t i = 0; i < pending_point_clouds_.size(); ++i) {
    pose_ids_.push_back(next_pose_id_ + static_cast<uint64_t>(i));
  }
  next_pose_id_ = pose_ids_.back() + 1;

  // Copy over pending point and normal clouds.
  point_clouds_.insert(point_clouds_.end(),
                         pending_point_clouds_.begin(),
                         pending_point_clouds_.end());
  normal_clouds_.insert(normal_clouds_.end(),
                          pending_normal_clouds_.begin(),
                          pending_normal_clouds_.end());

  if (point_clouds_.size() != normal_clouds_.size()) {
    printf("\nOld: %lu,%lu Additional:%lu,%lu\n",
           num_old_point_clouds,
           num_old_normal_clouds,
           pending_point_clouds_.size(),
           pending_normal_clouds_.size());
  }
  // Build KD trees for new point clouds.
  CHECK_GT(point_clouds_.size(), 0);
  CHECK_EQ(point_clouds_.size(), normal_clouds_.size());
  kdtrees_.resize(point_clouds_.size(), NULL);
  const size_t num_new_point_clouds = pending_point_clouds_.size();
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_new_point_clouds; ++i) {
    const PointCloudf& point_cloud = pending_point_clouds_[i];
    const NormalCloudf& normal_cloud = pending_normal_clouds_[i];
    vector<KDNodeValue<float, 2> > values(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      values[j].index = j;
      values[j].point = point_cloud[j];
      values[j].normal = normal_cloud[j];
    }
    kdtrees_[num_old_point_clouds + i] = new KDTree<float, 2>(values);
  }

  // Transform and copy over poses.
  Pose2Df latest_pose = latest_mle_pose_.GetLock();
  for (size_t i = 0; i < pending_relative_poses_.size(); ++i) {
    latest_pose.ApplyPose(pending_relative_poses_[i]);
    poses_.push_back(latest_pose);
    timestamps_.push_back(pending_stamps_[i]);
  }
  latest_mle_pose_.SetUnlock(latest_pose);

  // Add converted point clouds and copy of pose array.
  pose_array_.resize(point_clouds_.size() * 3);
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < num_new_point_clouds; ++i) {
    const size_t pose_array_offset = 3 * (num_old_point_clouds + i);
    const Pose2Df& pose = poses_[num_old_point_clouds + i];
    pose_array_[pose_array_offset + 0] = pose.translation.x();
    pose_array_[pose_array_offset + 1] = pose.translation.y();
    pose_array_[pose_array_offset + 2] = pose.angle;
  }

  CHECK_EQ(pose_ids_.size(), point_clouds_.size());
  pending_point_clouds_.clear();
  pending_normal_clouds_.clear();
  pending_relative_poses_.clear();
  pending_stamps_.clear();
}

void NonMarkovLocalization::AddPose(
    const PointCloudf& point_cloud,
    const NormalCloudf& normal_cloud,
    const Pose2Df& relative_pose,
    const timespec& laser_time) {
  // Add point_cloud, normal_cloud, relative_pose to pending buffer.
  // Reset distance traversed since last node.
  // If (number of pending nodes > threshold) and (update is not in progress) :
  //   Take every pending buffer entry and add to current list of poses.
  //   Spawn a new update.
  CHECK_EQ(point_cloud.size(), normal_cloud.size());
  CHECK_GT(point_cloud.size(), 0);
  pending_point_clouds_.push_back(point_cloud);
  pending_normal_clouds_.push_back(normal_cloud);
  pending_relative_poses_.push_back(relative_pose);
  pending_stamps_.push_back(laser_time);
  CHECK_GT(pending_point_clouds_.size(), 0);

  Pose2Df latest_pending_pose = latest_pending_pose_.GetLock();
  latest_pending_pose.ApplyPose(relative_pose);
  latest_pending_pose_.SetUnlock(latest_pending_pose);

  pending_relative_pose_.Clear();
  pending_translation_ = 0.0;
  pending_rotation_ = 0.0;

  if (static_cast<int>(pending_relative_poses_.size()) >=
      localization_options_.kPoseIncrement) {
    ScopedTryLock lock(&update_mutex_);
    if (lock.Locked()) {
      // Copy over pending nodes to current nodes.
      AddPendingPoseNodes();
      // Reset cumulative pending pose.
      latest_pending_pose_.Set(Pose2Df(0.0, 0.0, 0.0));
      // Start a new background thread for the update.
      CHECK(sem_post(&update_semaphore_) == 0);
    }
  }
}

Pose2Df NonMarkovLocalization::GetLatestPose() const {
  // WARNING: This code fragment could potentially suffer from concurrency
  // issues arising from the fact that the latest_pose_ and latest_pending_pose_
  // reads and the compund pose computation are not performed in an atomic
  // manner.
  Pose2Df latest_pose = latest_mle_pose_.Get();
  Pose2Df latest_pending_pose = latest_pending_pose_.Get();
  latest_pose.ApplyPose(latest_pending_pose);
  latest_pose.ApplyPose(pending_relative_pose_);
  return latest_pose;
}

bool NonMarkovLocalization::GetNodeData(
    vector<Pose2Df>* poses,
    vector<uint64_t>* pose_ids,
    vector<PointCloudf>* point_clouds,
    vector<Pose2Df>* pending_poses,
    vector<vector<ObservationType> >* observation_classes,
    Pose2Df* latest_pose) const {
  ScopedTryLock lock(&update_mutex_);
  if (!lock.Locked()) return false;
  *poses = poses_;
  *pose_ids = pose_ids_;
  *point_clouds = point_clouds_;
  *pending_poses = pending_relative_poses_;
  *observation_classes = observation_classes_;
  *latest_pose = latest_mle_pose_.Get();
  const Pose2Df latest_pending_pose = latest_pending_pose_.Get();
  latest_pose->ApplyPose(latest_pending_pose);
  latest_pose->ApplyPose(pending_relative_pose_);
  return true;
}

void NonMarkovLocalization::SetOptions(const LocalizationOptions& options) {
  localization_options_ = options;
}

Pose2Df NonMarkovLocalization::GetLastMLEPose() const {
  return (latest_mle_pose_.Get());
}

void NonMarkovLocalization::Initialize(const Pose2Df& pose,
                                       const string& map_name) {
  ScopedLock lock(&update_mutex_);
  ClearPoses();
  latest_mle_pose_.Set(pose);
  if (map_name != map_name_) {
    vector_map_.Load(GetMapFileFromName(map_name));
    map_name_ = map_name;
  }
}

string NonMarkovLocalization::GetCurrentMapName() const {
  return (map_name_);
}

bool NonMarkovLocalization::RunningSolver() const {
  ScopedTryLock lock(&update_mutex_);
  return (!lock.Locked());
}

vector<Pose2Df> NonMarkovLocalization::GetLoggedPoses() const {
  return (logged_poses_.Get());
}

vector<timespec> NonMarkovLocalization::GetLoggedStamps() const {
    return (logged_stamps_.Get());
}

std::vector<int> NonMarkovLocalization::GetLoggedEpisodeLengths() const {
  std::vector<int> episode_lengths;
  ScopedLock lock(&update_mutex_);
  episode_lengths = logged_episode_lengths_;
  return (episode_lengths);
}

void NonMarkovLocalization::Finalize() {
  if (pending_point_clouds_.size() > 0) {
    ScopedLock lock(&update_mutex_);
    // Copy over pending nodes to current nodes.
    AddPendingPoseNodes();
    // Reset cumulative pending pose.
    latest_pending_pose_.Set(Pose2Df(0.0, 0.0, 0.0));
    // Start a new background thread for the update.
    CHECK(sem_post(&update_semaphore_) == 0);
  }

  int pending_updates = 0;
  do {
    Sleep(0.01);
    CHECK_EQ(sem_getvalue(&update_semaphore_, &pending_updates), 0);
  } while (pending_updates > 0);

  {
    ScopedLock lock(&update_mutex_);
    TrimEpisode(poses_.size());
  }
}

void NonMarkovLocalization::ResetObservationClasses(
    const size_t start, const size_t end) {
  DCHECK_LT(start, point_clouds_.size());
  DCHECK_LT(end, point_clouds_.size());
  observation_classes_.resize(point_clouds_.size());
  OMP_PARALLEL_FOR
  for (size_t i = start; i <= end; ++i) {
    observation_classes_[i].resize(point_clouds_[i].size());
    for (size_t j = 0; j < point_clouds_[i].size(); ++j) {
      observation_classes_[i][j] = kDfObservation;
    }
  }
}

void NonMarkovLocalization::RecomputeAbsolutePoses() {
  pose_array_.resize(relative_pose_array_.size());
  // The pose array size must be a multiple of 3, since each pose has 3-DOFs.
  CHECK_EQ((pose_array_.size() % 3), 0);
  // The first 3-DOF pose value is absolute.
  pose_array_[0] = relative_pose_array_[0];
  pose_array_[1] = relative_pose_array_[1];
  pose_array_[2] = relative_pose_array_[2];
  for (size_t i = 3; i < pose_array_.size(); ++i) {
    pose_array_[i] = pose_array_[i - 3] + relative_pose_array_[i];
  }
}

void NonMarkovLocalization::RecomputeRelativePoses() {
  relative_pose_array_.resize(pose_array_.size());
  // The pose array size must be a multiple of 3, since each pose has 3-DOFs.
  CHECK_EQ((pose_array_.size() % 3), 0);
  // The first 3-DOF pose value is absolute.
  relative_pose_array_[0] = pose_array_[0];
  relative_pose_array_[1] = pose_array_[1];
  relative_pose_array_[2] = pose_array_[2];
  for (size_t i = 3; i < pose_array_.size(); ++i) {
    relative_pose_array_[i] = pose_array_[i] - pose_array_[i - 3];
  }
}

}  // namespace vector_localization
