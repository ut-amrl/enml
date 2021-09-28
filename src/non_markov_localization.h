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
// Interface for non-Markov Localization.

#ifndef NON_MARKOV_LOCALIZATION_H
#define NON_MARKOV_LOCALIZATION_H

#include <pthread.h>
#include <semaphore.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "math/geometry.h"
#include "math/line2d.h"
#include "kdtree.h"
#include "perception_2d.h"
#include "util/pthread_utils.h"
#include "sensor_msgs/LaserScan.h"
#include "vector_map/vector_map.h"

namespace vector_localization {

struct LTSConstraint;
struct PointToLineConstraint;
struct PointToPointConstraint;
struct VisibilityConstraint;
struct VisibilityGlobConstraint;

class NonMarkovLocalization {
 public:

  enum ObservationType {
    kLtfObservation = 0,
    kStfObservation = 1,
    kDfObservation = 2,
  };

  // Struct to keep track of the correspondence between a pair of observed
  // points, each point being from a different pose.
  struct PointToPointCorrespondence {
    // Index of the source pose.
    std::size_t source_pose;
    // Index of the point in the source pose for this correspondence.
    std::size_t source_point;
    // Index of the target pose.
    std::size_t target_pose;
    // Index of the point in the target pose for this correspondence.
    std::size_t target_point;
  };

  // Struct to keep track of the correspondence between STFs from a pair of
  // poses.
  struct PointToPointGlobCorrespondence {
    // Index number of the source pose that these points were observed from.
    std::size_t pose_index0;
    // Index number of the target pose that these points are related to.
    std::size_t pose_index1;
    // The index of the points from pose0.
    std::vector<size_t> points0_indices;
    // The index of the points from pose1.
    std::vector<size_t> points1_indices;
    // The coordinates of the point in the pose0 local coordinate frame.
    std::vector<Eigen::Vector2f> points0;
    // The coordinates of the point in the pose1 local coordinate frame.
    std::vector<Eigen::Vector2f> points1;
    // The coordinates of the normal in the pose0 local coordinate frame.
    std::vector<Eigen::Vector2f> normals0;
    // The coordinates of the normal in the pose1 local coordinate frame.
    std::vector<Eigen::Vector2f> normals1;
  };

  struct LocalizationOptions {
    // Minimum distance that the robot should move before considering a new pose
    // for non-Markov localization.
    float minimum_node_translation;

    // Minimum angle that the robot should turn before considering a new pose
    // for non-Markov localization.
    float minimum_node_rotation;

    // The maximum refresh period that non-Markov Localization will ignore
    // non-zero odometry updates over. If the robot has moved within this much
    // time, and even if the motion is smaller than minimum_node_translation and
    // minimum_node_rotation, a pose update will be forced.
    double max_update_period;
    float kMinRange;
    float kMaxRange;
    float kMaxPointToLineDistance;
    float kPointMapCorrelationFactor;
    float kPointPointCorrelationFactor;
    float kOdometryRadialStdDevRate;
    float kOdometryTangentialStdDevRate;
    float kOdometryAngularStdDevRate;
    float kOdometryTranslationMinStdDev;
    float kOdometryTranslationMaxStdDev;
    float kOdometryAngularMinStdDev;
    float kOdometryAngularMaxStdDev;
    float kPointMatchThreshold;
    float kMaxAngleError;
    float kMapHuberLossThreshold;
    float kLaserStdDev;
    float kMinLtfRatio;
    float kMaxStfAngleError;
    unsigned int num_skip_readings;
    unsigned int kMinEpisodeLength;
    int kMaxRepeatIterations;
    int kNumRepeatIterations;
    int kMaxCorrespondencesPerPoint;
    int kNumThreads;
    int kPoseIncrement;
    int kMaxHistory;
    int max_solver_iterations;
    bool use_STF_constraints;
    bool use_visibility_constraints;
    float kVisibilityCorrelationFactor;
    bool use_visual_odometry;
    bool log_poses;
    bool limit_history;
    // Location of the sensor with respect to the robot center. Assumes that
    // the sensor is forward-facing w.r.t. the robot.
    Eigen::Vector3f sensor_offset;
    void (*CorrespondenceCallback)(
      const std::vector<double>& poses,
      const std::vector<perception_2d::PointCloudf>& point_clouds,
      const std::vector<perception_2d::NormalCloudf>& normal_clouds,
      const std::vector<std::vector<geometry::Line2f> >& ray_cast_lines,
      const std::vector<std::vector<int> >& point_line_correspondences,
      const std::vector<PointToPointGlobCorrespondence>&
          point_point_correspondences,
      const std::vector<std::vector<ObservationType> >& classifications,
      const std::vector<VisibilityGlobConstraint*>& visibility_constraints,
      const std::vector<double>& gradients,
      const std::vector<Eigen::Matrix2f>& covariances,
      const std::vector<perception_2d::Pose2Df>& odometry_poses,
      const size_t start_pose,
      const size_t end_pose);

    LocalizationOptions() : log_poses(false), CorrespondenceCallback(NULL) {}
  };

  explicit NonMarkovLocalization(const std::string& maps_directory);
  virtual ~NonMarkovLocalization();

  // Notifies concurrently running threads to terminate.
  void Terminate();

  // Resets the pose IDs.
  void Reset();

  // Clears the current list of poses and associated observations.
  void ClearPoses();

  // Returns the latest pose estimate based on all odometry messages observed so
  // far.
  perception_2d::Pose2Df GetLatestPose() const;

  // Return the latest pose of the MLE optimized node.
  perception_2d::Pose2Df GetLastMLEPose() const;

  // Return the last computed lost metric.
  float GetLostMetric() const;

  // Returns copies of vectors of the poses, point clouds, and pending poses.
  bool GetNodeData(
      std::vector<perception_2d::Pose2Df>* poses,
      std::vector<uint64_t>* pose_ids,
      std::vector<perception_2d::PointCloudf>* point_clouds,
      std::vector<perception_2d::Pose2Df>* pending_poses,
      std::vector<std::vector<ObservationType> >* observation_classes,
      perception_2d::Pose2Df* latest_pose) const;

  // Account for odometry update.
  void OdometryUpdate(const float dx, const float dy, const float d_theta);

  // Account for observation update.
  void SensorUpdate(
      const perception_2d::PointCloudf& point_cloud,
      const perception_2d::NormalCloudf& normal_cloud,
      const timespec& laser_time);

  // Set localization options.
  void SetOptions(const LocalizationOptions& options);

  // Set initial MLE pose.
  void Initialize(const perception_2d::Pose2Df& pose,
                  const std::string& map_name);

  // Returns the current map name.
  std::string GetCurrentMapName() const;

  // Returns true when the background MLE solver is running.
  bool RunningSolver() const;

  // Returns a copy of the logged poses.
  std::vector<perception_2d::Pose2Df> GetLoggedPoses() const;

  // Returns a copy of the logged stamps.
  std::vector<timespec> GetLoggedStamps() const;

  // Returns the logged history of episode lengths.
  std::vector<int> GetLoggedEpisodeLengths() const;

  // Runs the MLE solver on any remaining pending nodes, and clears the latest
  // non-Markov episode.
  void Finalize();

  // Sample around the MLE and check if the estimate can be corrected by SRL.
  void SensorResettingResample(
      const perception_2d::PointCloudf& point_cloud,
      const perception_2d::NormalCloudf& normal_cloud,
      const std::size_t num_samples,
      float radial_stddev,
      float tangential_stddev,
      float angular_stddev,
      void (*callback)(
          const std::vector<double>& poses,
          const perception_2d::PointCloudf& point_cloud_e,
          const perception_2d::NormalCloudf& normal_cloud,
          const std::vector<vector_localization::LTSConstraint*>& constraints),
      std::vector<perception_2d::Pose2Df>* poses_ptr,
      std::vector<double>* pose_weights_ptr,
      std::vector<vector_localization::LTSConstraint*>* constraints_ptr);

  // Compute and return the likelihood of observing the specified point cloud
  // from the specified pose.
  double ObservationLikelihood(const perception_2d::Pose2Df& pose,
                               const perception_2d::PointCloudf& point_cloud,
                               const perception_2d::NormalCloudf& normal_cloud);

 private:
  // Add a new pose node along with the associated observations. Returns the
  // number of pose nodes in use at the moment.
  void AddPose(const perception_2d::PointCloudf& point_cloud,
               const perception_2d::NormalCloudf& normal_cloud,
               const perception_2d::Pose2Df& relative_pose,
               const timespec& laser_time);

  // Find point to map LTF correspondences for every point from every pose.
  // The points will be read from the member variable point_clouds_, and the
  // results of the correspondence matching will be stored in the member
  // variables ray_cast_lines_ and point_line_correspondences_.
  void FindSinglePoseLtfCorrespondences(
      const perception_2d::PointCloudf& point_cloud,
      const perception_2d::NormalCloudf& normal_cloud,
      const double* pose_array,
      std::vector<int>* line_correspondences_ptr,
      std::vector<geometry::Line2f>* ray_cast_ptr,
      std::vector<ObservationType>* observation_classes);

  // Find point to map LTF correspondences for every point from every pose.
  // The points will be read from the member variable point_clouds_, and the
  // results of the correspondence matching will be stored in the member
  // variables ray_cast_lines_ and point_line_correspondences_.
  void FindLTFCorrespondences(const std::size_t min_poses,
                              const std::size_t max_poses);

  // Find point to point STF correspondences for every point from every pose
  // to every other pose with overlapping scans. The points will be read from
  // the member variable point_clouds_, and the results of the correspondence
  // matching will be stored in the member variable
  // point_point_correspondences_.
  void FindSTFCorrespondences(const std::size_t min_poses,
                              const std::size_t max_poses);

  // Find correspondences between successive observations.
  void FindVisualOdometryCorrespondences(const std::size_t min_poses,
                                         const std::size_t max_poses);

  // Add a single point to map LTF constraint.
  void AddSinglePoseLTFConstraints(
      const std::size_t pose_index,
      const std::vector<geometry::Line2f>& ray_cast,
      const std::vector<int>& correspondences,
      const std::vector<ObservationType>& observation_classes,
      const perception_2d::PointCloudf& point_cloud,
      double* pose_array,
      std::vector<PointToLineConstraint*>* constraints,
      ceres::Problem* problem);

  // Add point to map LTF constraints based on previously computed
  // correspondences.
  // For each constraint, a new residual will be created and added to the
  // point_line_constraints_ member variable. A new autodiff residual will
  // be created from every constraint and added to the Ceres @problem.
  void AddLTFConstraints(const std::size_t min_poses,
                         const std::size_t max_poses,
                         ceres::Problem* problem);

  // Add point to map visibility LTF constraints based on previously computed
  // correspondences. These constraints try to minimize observations that
  // violate visibility constraints given the map.
  // For each constraint, a new residual will be created and added to the
  // visibility_constraints_ member variable. A new autodiff residual will
  // be created from every constraint and added to the Ceres @problem.
  void AddVisibilityConstraints(const std::size_t min_poses,
                                const std::size_t max_poses,
                                ceres::Problem* problem);

  // Add point to point STF constraints based on previously computed
  // correspondences stored in the member variable point_point_correspondences_.
  // For each constraint, a new residual will be created and added to the
  // point_point_constraints_ member variable. A new autodiff residual will
  // be created from every constraint and added to the Ceres @problem.
  void AddSTFConstraints(ceres::Problem* problem);

  // Reset the global pose array from @start to @end (both inclusive), using
  // relative poses computed from @poses.
  void ResetGlobalPoses(const std::size_t start, const std::size_t end,
                        const std::vector<perception_2d::Pose2Df>& poses);

  // Add pose constraints based on odometry between successive poses. A new
  // autodiff residual will be created from every constraint and added to
  // the Ceres @problem.
  void AddPoseConstraints(const std::size_t min_poses,
                          const std::size_t max_poses,
                          const std::vector<perception_2d::Pose2Df>& poses,
                          ceres::Problem* problem);

  // Resets observation classifications from pose indices [@start, @end]. The
  // classifications will later be updated as observation correspondences are
  // evaluated.
  void ResetObservationClasses(const std::size_t start, const std::size_t end);

  // Save the number of LTF constraints for every node in the history to a file
  // named 'constraints.txt' in CSV format.
  void SaveEpisodeData();

  // Finds the pose index of the start of the latest non-Markov episode.
  int FindEpisodeStart();

  // Trims (removes) pose nodes prior to the specified pose index.
  void TrimEpisode(const int pose_index);

  // Returns true iff the @point observed by the robot at @pose has a valid
  // LTF correspondence to @line.
  bool IsValidLTFCorrespondence(const double* pose,
                                const Eigen::Vector2f& point,
                                const Eigen::Vector2f& normal,
                                const geometry::Line2f& line) const;

  // Buld KD Trees for every pose with the provided @point_clouds and
  // @normal_clouds.
  void BuildKDTrees(
      const std::vector<perception_2d::PointCloudf>& point_clouds,
      const std::vector<perception_2d::NormalCloudf>& normal_clouds);

  // Compute the relative transform to transform a point in the pose with the
  // index @source to the pose with the index @target.
  Eigen::Affine2f RelativePoseTransform(unsigned int source,
                                        unsigned int target);

  // Add the pending pose nodes along with the observations to the vectors of
  // current poses, point clouds, and normal clouds.
  void AddPendingPoseNodes();

  // Add the latest pose node along with the associated observation(s) and
  // update the location estimates.
  void Update();

  // Check if the observation classifications have chaged or not.
  bool HaveClassificationsChanged(
      const std::vector<std::vector<ObservationType> >& point_classes1,
      const std::vector<std::vector<ObservationType> >& point_classes2);

  // The always-running update thread function.
  static void* UpdateThreadFunction(void* non_markov_localization);

  // Returns the number of observed points and number of valid LTF constraints
  // for the pose node entry indicated by @node_index.
  void CountConstraints(std::size_t node_index,
                        std::size_t* num_points_ptr,
                        std::size_t* num_ltf_constraints_ptr);

  // Recomputes the relative pose array from the absolute pose array.
  void RecomputeRelativePoses();

  // Recomputes the absolute pose array from the relative pose array.
  void RecomputeAbsolutePoses();

  // Computes an error metric that idicates how "lost" the EnML estimate is.
  void ComputeLostMetric();

  // Get map file name from map name.
  std::string GetMapFileFromName(const std::string& map) const;

  // The error metric of how lost the EnML estimate is.
  float lost_metric_;

  // Mutex to control access to pose nodes and point clouds. When locked, it
  // means that an update is concurrently running in the background. When
  // unlocked, it means that the pose nodes and point clouds are safe to be
  // modified, and there is no update running concurrently. See the
  // @update_semaphore_ for documentation of the order in which this mutex and
  // the @update_semaphore_ should be updated.
  mutable pthread_mutex_t update_mutex_;

  // Semaphore to control the producer - consumer nature of non-Markov
  // Localization. The semaphore is incremented when it's necessary to run the
  // update in the background thread, and decremented by the update thread once
  // done.
  //
  // Producer / Consumer model:
  // The producer is the thread that adds new pose data, the consumer is the
  // background thread that does the actual pose update.
  //
  // Once new sensor information is available to the producer:
  //  1. The producer first tries to lock the @update_mutex_ to get exclusive
  //     write-access to the pose nodes data structures.
  //  2. If the lock was succesful, it then adds the new data to the pose, point
  //     clouds and normal clouds arrays, and increments the @update_semaphore_.
  //  3. Finally it unlocks the @update_mutex_.
  //
  // The consumer:
  //  1. Waits on a background thread until the @update_semaphore_ is
  //     incremented by the producer.
  //  2. It then waits for a lock on the @update_mutex_ to get exclusive
  //     write-access to the pose nodes data structures.
  //  3. It then updates the MLE pose estimates by running non-Markov
  //     Localization.
  //  4. Finally it unlocks the @update_mutex_.
  sem_t update_semaphore_;

  // Handle to the update thread.
  pthread_t update_thread_;

  // Indicates that the update thread and batch localization (if running) should
  // keep running. It gets set to false by the destructor, or if the instance
  // is explicitly asked to terminate.
  bool terminate_;

  // The vector map on which localization is being performed.
  vector_map::VectorMap vector_map_;

  // The accumulated translation and rotation of the robot since the last pose
  // node.
  perception_2d::Pose2Df pending_relative_pose_;

  // The sum of the magnitude of translations of the robot since the the last
  // pose node.
  float pending_translation_;

  // The sum of the magnitude of rotations of the robot since the the last
  // pose node.
  float pending_rotation_;

  // Point clouds for every pose node, in Eigen::Vector2f format.
  std::vector<perception_2d::PointCloudf> point_clouds_;

  // Normal clouds for every pose node, in Eigen::Vector2f format.
  std::vector<perception_2d::NormalCloudf> normal_clouds_;

  // Poses of every node.
  std::vector<perception_2d::Pose2Df> poses_;

  // Timestamps of every node
  std::vector<timespec> timestamps_;

  // Unique IDs for each pose.
  std::vector<uint64_t> pose_ids_;

  // Unique ID of the next pose that will be added.
  uint64_t next_pose_id_;

  // KD Trees of point clouds.
  std::vector<KDTree<float, 2>* > kdtrees_;

  // List of lines visible from each pose as determined by performing analytic
  // ray casting on the map using the latest pose estimates.
  std::vector<std::vector<geometry::Line2f> > ray_cast_lines_;

  // List of correspondences between each point in the point clouds, and
  // the corresponding list of visible lines as stored in @ray_cast_lines.
  std::vector<std::vector<int> > point_line_correspondences_;

  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<PointToPointCorrespondence> point_point_correspondences_;

  // List of point to point constraints obtained by matching observed points
  // from overlapping poses.
  std::vector<PointToPointGlobCorrespondence> point_point_glob_correspondences_;

    // List of point to line LTF constraints.
  std::vector<PointToLineConstraint*> point_line_constraints_;

  // List of LTF visibility constraints.
  std::vector<VisibilityGlobConstraint*> visibility_constraints_;

  // List of point to point STF constraints.
  std::vector<PointToPointConstraint*> point_point_constraints_;

  // Array containing the 3DOF poses being optimized.
  std::vector<double> pose_array_;

  // Array containing the 3DOF pose of the initial pose of the episode, and the
  // relative poses of every subsequent pose.
  std::vector<double> relative_pose_array_;

  // Latest pose estimate of the latest MLE optimized pose node.
  ThreadSafe<perception_2d::Pose2Df> latest_mle_pose_;

  // The pose of the latest pending pose node, relative to the latest MLE
  // optimized pose node, @latest_pose_.
  ThreadSafe<perception_2d::Pose2Df> latest_pending_pose_;

  // Pending point clouds in Eigen::Vector2f format, that yet need to be added
  // to the MLE pose array.
  std::vector<perception_2d::PointCloudf> pending_point_clouds_;

  // Pending normal clouds in Eigen::Vector2f format, that yet need to be added
  // to the MLE pose array.
  std::vector<perception_2d::NormalCloudf> pending_normal_clouds_;

  // Pending relative poses of every node, that yet need to be added to the MLE
  // pose array.
  std::vector<perception_2d::Pose2Df> pending_relative_poses_;

  // Pending timestamps of nodes that yet need to be added to the stamps
  // corresponding to the MLE pose array.
  std::vector<timespec> pending_stamps_;

  // The history of logged poses generated by non-Markov localization. The poses
  // are logged if localization_options_.log_poses is set to true, and are
  // saved by the TrimEpisode function when the poses are removed from the
  // latest non-Markov episode.
  ThreadSafe<std::vector<perception_2d::Pose2Df> > logged_poses_;

  // The history of logged timestamps corresponding to poses generated by
  // non-Markov localization. The stamps (and corresponding poses)
  // are logged if localization_options_.log_poses is set to true, and are
  // saved by the TrimEpisode function when the poses are removed from the
  // latest non-Markov episode.
  ThreadSafe<std::vector<timespec>> logged_stamps_;

  // The history of the episode lengths for every MLE update.
  std::vector<int> logged_episode_lengths_;

  // Options pertaining to non-Markov Localization.
  LocalizationOptions localization_options_;

  // Classifications of every point, from every pose, as being an LTF, STF, or
  // DF.
  std::vector<std::vector<ObservationType> > observation_classes_;

  // The last time a pose update was performed.
  double t_last_update_;

  // The directory where all maps are stored.
  const std::string maps_dir_;

  // The current map name.
  std::string map_name_;
};

}  // namespace vector_localization

#endif  // NON_MARKOV_LOCALIZATION_H
