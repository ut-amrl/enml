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
// Main entry point for non-Markov localization.

#include <execinfo.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pthread.h>
#include <queue>
#include <string>
#include <termios.h>
#include <utility>
#include <vector>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

#include "non_markov_localization.h"
#include "perception_2d.h"
#include "popt_pp/popt_pp.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/helpers.h"
#include "shared/util/pthread_utils.h"
#include "shared/util/random.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"
#include "residual_functors.h"
#include "config_reader/config_reader.h"
#include "visualization/visualization.h"

using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using Eigen::Vector3f;
using perception_2d::GenerateNormals;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using ros::ServiceServer;
using ros::Subscriber;
using std::max;
using std::min;
using std::pair;
using std::queue;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using vector_localization::NonMarkovLocalization;
using vector_map::VectorMap;

using namespace geometry;
using namespace math_util;

typedef KDNodeValue<float, 2> KDNodeValue2f;

namespace {
// Name of the topic that scan data is published on.
CONFIG_STRING(scan_topic, "RobotConfig.scan_topic");

// Name of the topic that odometry is published on.
CONFIG_STRING(odom_topic, "RobotConfig.odometry_topic");

// Name of the topic that location reset commands are published on.
CONFIG_STRING(initialpose_topic, "RobotConfig.initialpose_topic");

// ROS message for publishing SE(2) pose with map name.
amrl_msgs::Localization2DMsg localization_msg_;
geometry_msgs::PoseStamped pose_msg_;     //added by mk, ryan
geometry_msgs::PoseStamped pose_msg_en;     //added by mk, ryan
//geometry_msgs::PoseWithCovarianceStamped pose_msg_;     //added by mk, ryan

// ROS message for visualization with WebViz.
amrl_msgs::VisualizationMsg visualization_msg_;

util_random::Random rand_;
}  // namespace


// Name of the map to localize the robot on.
string kMapName;
// Robot's starting location.
Vector2f kStartingLocation = Vector2f(-9.0, 42.837);
// Robot's starting angle.
float kStartingAngle = DegToRad(-165.0);
// Scaling constant to correct for error in angular odometry.
float kOdometryRotationScale = 1.0;
// Scaling constant to correct for error in translation odometry.
float kOdometryTranslationScale = 1.0;
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;
// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// The maximum expected odometry-reported translation difference per timestep.
float kSqMaxOdometryDeltaLoc = Sq(0.2);
// The maximum expected odometry-reported rotation difference per timestep.
float kMaxOdometryDeltaAngle = DegToRad(15.0);
// Mutex to ensure only a single relocalization call is made at a time.
pthread_mutex_t relocalization_mutex_ = PTHREAD_MUTEX_INITIALIZER;

// The directory where all the maps are stored.
const char* maps_dir_ = "maps";

// Directory containing all config files, including `common.lua` and `enml.lua`
const char* config_dir_ = "config";

// name of the robot configuration file in config_dir_ to use
const char* robot_config_ = "robot.lua";

// Index of test set. This will determine which file the results of the test are
// saved to.
int test_set_index_ = -1;

// Indicates that a statistical test is being run, and to save the localization
// results to the file with the specified index.
int statistical_test_index_ = -1;

// The fraction of additive odometry noise for statistical tests.
double odometry_additive_noise_ = 0.05;

//static const uint32_t kTrajectoryColor = 0x6FFF0000;
static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
// static const uint32_t kTrajectoryColor = 0xFFC0C0C0;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFF994CD9;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;

bool run_ = true;
int debug_level_ = -1;
int publishiter=0;
bool initialized_=false;

// ROS publisher to publish visualization messages.
ros::Publisher visualization_publisher_;

// ROS publisher to publish the latest robot localization.
ros::Publisher localization_publisher_;
ros::Publisher pose_publisher_;
ros::Publisher pose_publisher_2;
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf2_listener; 
geometry_msgs::TransformStamped map_en_to_map;
tf::Transform init_transform;
//map_en_to_map= tf_buffer.lookupTransform("map_en", "map", ros::Time(0), ros::Duration(1.0) );

// Parameters and settings for Non-Markov Localization.
NonMarkovLocalization::LocalizationOptions localization_options_;

// Main class instance for Non-Markov Localization.
NonMarkovLocalization* localization_;

// The last observed laser scan, used for auto localization.
sensor_msgs::LaserScan last_laser_scan_;

// Directory where images of every episode at every timestep will be saved.
char* episode_images_path = NULL;

// Determines whether STFS will be saved for later object mapping or not.
bool save_stfs_ = false;

// Corrections to angle-dependent errors of the laser scanner.
vector<float> laser_corrections_;

// Resolution of the laser corrections lookup table.
float laser_corrections_resolution_ = 0.0;

// Boolean flag to indicate use of laser scanner corrections.
bool use_laser_corrections_ = false;

// Flag to write LTF observations to disk for error correction computation.
bool save_ltfs_ = false;

// Suppress stdout.
bool quiet_ = false;

// Accept noise-free odometry, and apply noise by mimicing encoders of a
// four=-wheel omnidirectional robot.
void ApplyNoiseModel(
    const float dx,
    const float dy,
    const float da,
    const float e,
    float* dx_n,
    float* dy_n,
    float* da_n) {
  // Wheel radius.
  const float R = 0.1;
  Eigen::Matrix<float, 4, 3> M_vel_to_enc;
  Eigen::Matrix<float, 3, 4> M_enc_to_vel;
  const float C = cos(DegToRad(45.0));
  M_vel_to_enc <<
      C, C, R,
      -C, C, R,
      -C, -C, R,
      C, -C, R;
  const float kSqRt2 = sqrt(2.0);
  M_enc_to_vel <<
      kSqRt2, -kSqRt2, -kSqRt2, kSqRt2,
      kSqRt2, kSqRt2, -kSqRt2, -kSqRt2,
      1.0 / R, 1.0 / R, 1.0 / R, 1.0 / R;
  M_enc_to_vel = M_enc_to_vel / 4.0;
  const Eigen::Vector3f delta(dx, dy, da);
  // Compute noise-free encoder reading.
  const Eigen::Vector4f enc = M_vel_to_enc * delta;
  Eigen::Vector4f enc_noisy = Eigen::Vector4f::Zero();
  // Add noise.
  for (int i = 0; i < 4; ++i) {
    enc_noisy(i) = enc(i) + rand_.Gaussian(0, e * enc(i));
  }
  const Eigen::Vector3f delta_noisy = M_enc_to_vel * enc_noisy;
  *dx_n = delta_noisy(0);
  *dy_n = delta_noisy(1);
  *da_n = delta_noisy(2);
}

void PublishLocation(
    const string& map_name, const float x, const float y, const float angle) {
  localization_msg_.header.stamp.fromSec(GetWallTime());
  localization_msg_.map = map_name;
  localization_msg_.pose.x = x;
  localization_msg_.pose.y = y;
  localization_msg_.pose.theta = angle;
  localization_publisher_.publish(localization_msg_);

  //-------------------------
  //
  if(publishiter>3){

      try{
          map_en_to_map= tf_buffer.lookupTransform("map_en", "map", ros::Time(0), ros::Duration(1.0) );
      }
      catch (tf2::TransformException &ex)
      {
          ROS_WARN("lookupTransform Failed: %s", ex.what());
          publishiter=0;
          return;
      }


      geometry_msgs::PoseStamped tmp_pose;     //added by mk, ryan
      //tmp_pose.header.stamp.fromSec(GetWallTime());
      tmp_pose.header.stamp= ros::Time::now();
      tmp_pose.header.frame_id= "map_en";
      tmp_pose.pose.position.x = x;
      tmp_pose.pose.position.y = y;
      //printf("map-en: x: %.2lf, y: %.2lf \n", x, y);
      //
      pose_msg_en.header.stamp.fromSec(GetWallTime());
      pose_msg_en.header.frame_id= "map_en";
      pose_msg_en.pose=tmp_pose.pose;
      pose_publisher_2.publish(pose_msg_en);

      geometry_msgs::Quaternion q;

      double temp_roll=0.0;
      double temp_pitch=0.0;
      double t0 = cos(angle* 0.5);
      double t1 = sin(angle* 0.5);
      double t2 = cos(temp_roll * 0.5);
      double t3 = sin(temp_roll * 0.5);
      double t4 = cos(temp_pitch * 0.5);
      double t5 = sin(temp_pitch * 0.5);
      q.w = t0 * t2 * t4 + t1 * t3 * t5;
      q.x = t0 * t3 * t4 - t1 * t2 * t5;
      q.y = t0 * t2 * t5 + t1 * t3 * t4;
      q.z = t1 * t2 * t4 - t0 * t3 * t5;

      tmp_pose.pose.orientation.x =q.x;
      tmp_pose.pose.orientation.y =q.y;
      tmp_pose.pose.orientation.z =q.z;
      tmp_pose.pose.orientation.w =q.w;

      geometry_msgs::PoseStamped pose_out;     //added by mk, ryan
      //tf2::doTransform(tmp_pose, tmp_pose, map_en_to_map);
      try{
          tf_buffer.transform(tmp_pose,pose_out, "map");
      }
      catch (tf2::TransformException &ex)
      {
          ROS_WARN("Transform Failed: %s", ex.what());
          
          publishiter=0;
          return;
      
      }

      //printf("----- map: x: %.2lf, y: %.2lf", pose_out.pose.position.x, pose_out.pose.position.y);
      //pose_msg_.header.stamp.fromSec(GetWallTime());
      pose_msg_.header.stamp= ros::Time::now();
      pose_msg_.header.frame_id= "map";
      pose_msg_.pose=pose_out.pose;
      //pose_msg_.pose.pose.position.x = tmp_pose.x;
      //pose_msg_.pose.pose.position.y = tmp_pose.y;
      //pose_msg_.pose.covariance={0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
                            //0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            //0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            //0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};


      pose_publisher_.publish(pose_msg_);
      publishiter=0;

      /*
      static tf2_ros::TransformBroadcaster br;
      geometry_msgs::TransformStamped transformStamped;

      //transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.stamp.fromSec(GetWallTime()); 
      transformStamped.header.frame_id = "map_en";
      transformStamped.child_frame_id = "base";
      transformStamped.transform.translation.x = pose_msg_en.pose.position.x;
      transformStamped.transform.translation.y = pose_msg_en.pose.position.y;
      transformStamped.transform.translation.z = 0.0;
      tf2::Quaternion q_;
      q_.setRPY(0, 0, angle);
      transformStamped.transform.rotation.x = q_.x();
      transformStamped.transform.rotation.y = q_.y();
      transformStamped.transform.rotation.z = q_.z();
      transformStamped.transform.rotation.w = q_.w();

      br.sendTransform(transformStamped);
      */


     //static tf::TransformBroadcaster br;
      //tf::Transform transform;
      //transform.setOrigin(tf::Vector3(pose_msg_en.pose.position.x, ppose_msg_en.pose.position.y,0.0));
      //tf::Quaternion quat(pose_msg_en.pose.orientation.x, pose_msg_en.pose.orientation.y, pose_msg_en.pose.orientation.z, pose_msg_en.pose.orientation.w);
      //transform.setRotation(quat);
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map_en", "base"));
  
  }

  publishiter++;

 //ROS_INFO("send_transform");



  //-----------------------
  /*
 //publish global pose with geometry_msgs
  pose_msg_.header.stamp.fromSec(GetWallTime());
  pose_msg_.header.frame_id= "map_en";
  pose_msg_.pose.position.x = x;
  pose_msg_.pose.position.y = y;
  geometry_msgs::Quaternion q;
  //tf::Quaternion q;
  //q.setRPY(0,0,angle);

  double temp_roll=0.0;
  double temp_pitch=0.0;
  double t0 = cos(angle* 0.5);
  double t1 = sin(angle* 0.5);
  double t2 = cos(temp_roll * 0.5);
  double t3 = sin(temp_roll * 0.5);
  double t4 = cos(temp_pitch * 0.5);
  double t5 = sin(temp_pitch * 0.5);
  q.w = t0 * t2 * t4 + t1 * t3 * t5;
  q.x = t0 * t3 * t4 - t1 * t2 * t5;
  q.y = t0 * t2 * t5 + t1 * t3 * t4;
  q.z = t1 * t2 * t4 - t0 * t3 * t5;

  pose_msg_.pose.orientation.x =q.x;
  pose_msg_.pose.orientation.y =q.y;
  pose_msg_.pose.orientation.z =q.z;
  pose_msg_.pose.orientation.w =q.w;

  pose_publisher_.publish(pose_msg_);



  //publish tf frames
  //static tf::TransformBroadcaster br;
  //tf::Transform transform;
  //transform.setOrigin(tf::Vector3(x, y,0.0));
  //tf::Quaternion quat;
  //quat.setRPY(0,0,angle);
  //transform.setRotation(quat);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map_en", "base"));

  //static tf::TransformBroadcaster br2;
  //if(initialized_)
  //{
      //br2.sendTransform(tf::StampedTransform(init_transform, ros::Time::now(), "map_en", "odom"));
  //}

  //publish tf frames
  //static tf::TransformBroadcaster br;
  //tf::Transform transform;
  //transform.setOrigin(tf::Vector3(x, y,0.0));
  //tf::Quaternion quat;
  //quat.setRPY(0,0,angle);
  //transform.setRotation(quat);
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map_en", "base_footprint"));
  */

}

void PublishLocation() {
  Pose2Df pose = localization_->GetLatestPose();
  const string map = localization_->GetCurrentMapName();
  PublishLocation(map, pose.translation.x(), pose.translation.y(), pose.angle);
}

void PublishTrace() {
  Pose2Df latest_pose = localization_->GetLatestPose();
  static vector<Vector2f> trace;
  static Vector2f lastLoc;
  static bool initialized = false;
  const Vector2f curLoc(
        latest_pose.translation.x(), latest_pose.translation.y());
  if (!initialized) {
    trace.push_back(curLoc);
    lastLoc = curLoc;
    initialized = true;
    printf("Init trace\n");
    return;
  }
  if((curLoc-lastLoc).squaredNorm()>Sq(0.05)){
    trace.push_back(curLoc);
    lastLoc = curLoc;
  }

  for(unsigned int i = 0; i + 1 < trace.size(); i++){
    if((trace[i]-trace[i+1]).squaredNorm()>Sq(5.0))
      continue;
    visualization::DrawLine(trace[i], trace[i + 1], 0xFFc0c0c0, visualization_msg_);
  }
}

void ClearDisplay() {
  visualization::ClearVisualizationMsg(visualization_msg_);
}

void PublishDisplay() {
  visualization_publisher_.publish(visualization_msg_);
}

int kbhit() {
  if(!run_) return 0;
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(const bool blocking) {
  struct termios ttystate;

  //get the terminal state
  tcgetattr(STDIN_FILENO, &ttystate);

  if (blocking) {
    //turn off canonical mode
    ttystate.c_lflag &= ~ICANON;
    //minimum of number input read.
    ttystate.c_cc[VMIN] = 1;
  } else {
    //turn on canonical mode
    ttystate.c_lflag |= ICANON;
  }
  //set the terminal attributes.
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool LoadConfiguration(NonMarkovLocalization::LocalizationOptions* options) {
  #define ENML_FLOAT_CONFIG(x) \
      CONFIG_FLOAT(x, "enml."#x)
  #define ENML_INT_CONFIG(x) \
      CONFIG_INT(x, "enml."#x)
  #define ENML_UINT_CONFIG(x) \
      CONFIG_UINT(x, "enml."#x)
  #define ENML_BOOL_CONFIG(x) \
      CONFIG_BOOL(x, "enml."#x)
  #define ENML_STRING_CONFIG(x) \
      CONFIG_STRING(x, "enml."#x)
  ENML_FLOAT_CONFIG(starting_loc_x);
  ENML_FLOAT_CONFIG(starting_loc_y);
  ENML_FLOAT_CONFIG(starting_angle);
  ENML_FLOAT_CONFIG(odometry_translation_scale);
  ENML_FLOAT_CONFIG(odometry_rotation_scale);
  ENML_FLOAT_CONFIG(max_odometry_delta_loc);
  ENML_FLOAT_CONFIG(max_odometry_delta_angle);
  ENML_FLOAT_CONFIG(min_point_cloud_range);
  ENML_FLOAT_CONFIG(max_point_cloud_range);
  ENML_FLOAT_CONFIG(max_normal_point_distance);

  CONFIG_FLOAT(robot_sensor_offset_x, "enml.robot_sensor_offset.x");
  CONFIG_FLOAT(robot_sensor_offset_y, "enml.robot_sensor_offset.y");
  CONFIG_FLOAT(robot_sensor_offset_z, "enml.robot_sensor_offset.z");
  ENML_FLOAT_CONFIG(min_rotation);
  ENML_FLOAT_CONFIG(min_translation);
  ENML_INT_CONFIG(max_correspondences_per_point);
  ENML_FLOAT_CONFIG(max_point_to_line_distance);
  ENML_FLOAT_CONFIG(laser_std_dev);
  ENML_FLOAT_CONFIG(map_correlation_factor);
  ENML_FLOAT_CONFIG(point_correlation_factor);
  ENML_FLOAT_CONFIG(odometry_radial_stddev_rate);
  ENML_FLOAT_CONFIG(odometry_tangential_stddev_rate);
  ENML_FLOAT_CONFIG(odometry_angular_stddev_rate);
  ENML_FLOAT_CONFIG(odometry_translation_min_stddev);
  ENML_FLOAT_CONFIG(odometry_translation_max_stddev);
  ENML_FLOAT_CONFIG(odometry_rotation_min_stddev);
  ENML_FLOAT_CONFIG(odometry_rotation_max_stddev);
  ENML_FLOAT_CONFIG(point_match_threshold);
  ENML_FLOAT_CONFIG(max_stf_angle_error);
  ENML_FLOAT_CONFIG(max_angle_error);
  ENML_INT_CONFIG(pose_increment);
  ENML_INT_CONFIG(max_history);
  ENML_FLOAT_CONFIG(map_huber_loss);
  ENML_INT_CONFIG(max_solver_iterations);
  ENML_INT_CONFIG(max_repeat_iterations);
  ENML_INT_CONFIG(num_repeat_iterations);
  ENML_FLOAT_CONFIG(min_ltf_ratio);
  ENML_UINT_CONFIG(min_episode_length);
  ENML_UINT_CONFIG(num_threads);
  ENML_BOOL_CONFIG(use_visibility_constraints);
  ENML_BOOL_CONFIG(limit_history);
  ENML_FLOAT_CONFIG(visibility_correlation_factor);
  ENML_UINT_CONFIG(num_skip_readings);
  ENML_FLOAT_CONFIG(max_update_period);
  ENML_STRING_CONFIG(map_name);
  config_reader::ConfigReader reader({
    std::string(config_dir_) + "/common.lua",
    std::string(config_dir_) + "/" + std::string(robot_config_),
    std::string(config_dir_) + "/enml.lua"
  });
  options->kMinRange = CONFIG_min_point_cloud_range;
  options->kMaxRange = CONFIG_max_point_cloud_range;
  options->kMaxPointToLineDistance = CONFIG_max_point_to_line_distance;
  options->kPointMapCorrelationFactor = CONFIG_map_correlation_factor;
  options->kPointPointCorrelationFactor = CONFIG_point_correlation_factor;
  options->kOdometryRadialStdDevRate = CONFIG_odometry_radial_stddev_rate;
  options->kOdometryTangentialStdDevRate = CONFIG_odometry_tangential_stddev_rate;
  options->kOdometryAngularStdDevRate = CONFIG_odometry_angular_stddev_rate;
  options->kOdometryTranslationMinStdDev = CONFIG_odometry_translation_min_stddev;
  options->kOdometryTranslationMaxStdDev = CONFIG_odometry_translation_max_stddev;
  options->kOdometryAngularMinStdDev = CONFIG_odometry_rotation_min_stddev;
  options->kOdometryAngularMaxStdDev = CONFIG_odometry_rotation_max_stddev;
  options->kPointMatchThreshold = CONFIG_point_match_threshold;
  options->kMaxAngleError = CONFIG_max_angle_error;
  options->kMapHuberLossThreshold = CONFIG_map_huber_loss;
  options->kLaserStdDev = CONFIG_laser_std_dev;
  options->kMinLtfRatio = CONFIG_min_ltf_ratio;
  options->kMaxStfAngleError = CONFIG_max_stf_angle_error;
  options->num_skip_readings = CONFIG_num_skip_readings;
  options->kMinEpisodeLength = CONFIG_min_episode_length;
  options->kMaxRepeatIterations = CONFIG_max_repeat_iterations;
  options->kNumRepeatIterations = CONFIG_num_repeat_iterations;
  options->kMaxCorrespondencesPerPoint = CONFIG_max_correspondences_per_point;
  options->kNumThreads = CONFIG_num_threads;
  options->kPoseIncrement = CONFIG_pose_increment;
  options->kMaxHistory = CONFIG_max_history;
  options->max_solver_iterations = CONFIG_max_solver_iterations;
  options->use_visibility_constraints = CONFIG_use_visibility_constraints;
  options->kVisibilityCorrelationFactor = CONFIG_visibility_correlation_factor;
  options->limit_history = CONFIG_limit_history;

  kStartingLocation = Vector2f(CONFIG_starting_loc_x, CONFIG_starting_loc_y);
  kStartingAngle = CONFIG_starting_angle;
  kOdometryTranslationScale = CONFIG_odometry_translation_scale;
  kOdometryRotationScale = CONFIG_odometry_rotation_scale;
  kSqMaxOdometryDeltaLoc = Sq(CONFIG_max_odometry_delta_loc);
  kMaxOdometryDeltaAngle = CONFIG_max_odometry_delta_angle;
  kMinPointCloudRange = CONFIG_min_point_cloud_range;
  kMaxPointCloudRange = CONFIG_max_point_cloud_range;
  kMaxNormalPointDistance = CONFIG_max_normal_point_distance;
  kMapName = CONFIG_map_name;

#ifdef NDEBUG
  options->kNumThreads = CONFIG_num_threads;
#else
  options->kNumThreads = 1;
#endif

  options->kMinRange = kMinPointCloudRange;
  options->kMaxRange = kMaxPointCloudRange;
  options->sensor_offset.x() = CONFIG_robot_sensor_offset_x;
  options->sensor_offset.y() = CONFIG_robot_sensor_offset_y;
  options->sensor_offset.z() = CONFIG_robot_sensor_offset_z;
  options->minimum_node_rotation = CONFIG_min_rotation;
  options->minimum_node_translation = CONFIG_min_translation;
  options->kMaxCorrespondencesPerPoint = CONFIG_max_correspondences_per_point;
  options->kMaxPointToLineDistance = CONFIG_max_point_to_line_distance;
  options->kLaserStdDev = CONFIG_laser_std_dev;
  options->kPointMapCorrelationFactor = CONFIG_map_correlation_factor;
  options->kPointPointCorrelationFactor = CONFIG_point_correlation_factor;
  options->kOdometryRadialStdDevRate = CONFIG_odometry_radial_stddev_rate;
  options->kOdometryTangentialStdDevRate = CONFIG_odometry_tangential_stddev_rate;
  options->kOdometryAngularStdDevRate = CONFIG_odometry_angular_stddev_rate;
  options->kOdometryTranslationMinStdDev = CONFIG_odometry_translation_min_stddev;
  options->kOdometryTranslationMaxStdDev = CONFIG_odometry_translation_max_stddev;
  options->kOdometryAngularMinStdDev = CONFIG_odometry_rotation_min_stddev;
  options->kOdometryAngularMaxStdDev = CONFIG_odometry_rotation_max_stddev;
  options->kPointMatchThreshold = CONFIG_point_match_threshold;
  options->kMaxStfAngleError = CONFIG_max_stf_angle_error;
  options->kMaxAngleError = CONFIG_max_angle_error;
  options->kPoseIncrement = CONFIG_pose_increment;
  options->kMaxHistory = CONFIG_max_history;
  options->kMapHuberLossThreshold = CONFIG_map_huber_loss;
  options->max_solver_iterations = CONFIG_max_solver_iterations;
  options->kMaxRepeatIterations = CONFIG_max_repeat_iterations;
  options->kNumRepeatIterations = CONFIG_num_repeat_iterations;
  options->kMinLtfRatio = CONFIG_min_ltf_ratio;
  options->kMinEpisodeLength = CONFIG_min_episode_length;
  options->use_visibility_constraints = CONFIG_use_visibility_constraints;
  options->kVisibilityCorrelationFactor = CONFIG_visibility_correlation_factor;
  options->num_skip_readings = CONFIG_num_skip_readings;
  options->max_update_period = CONFIG_max_update_period;
  return true;
}

void SaveStfs(
    const string& map_name,
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const string& bag_file,
    double timestamp) {
  static const bool kSaveAllObservations = true;
  static const bool kDisplaySteps = false;
  // static const float kMaxMapLineOffset = 0.2;
  const string stfs_file = bag_file + ".stfs";
  ScopedFile fid(stfs_file, "w");
  fprintf(fid(), "%s\n", map_name.c_str());
  fprintf(fid(), "%lf\n", timestamp);
  VectorMap map(std::string(maps_dir_) + "/" + map_name + ".txt");
  if (kDisplaySteps) {
    visualization::ClearVisualizationMsg(visualization_msg_);
    nonblock(true);
  }
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const Rotation2Df pose_rotation(poses[i].angle);
    const Affine2f pose_transform =
        Translation2f(poses[i].translation) * pose_rotation;
    CHECK(false) << "ERROR: Unimplemented " << __FILE__ << ":" << __LINE__;
    // const vector<int>& visibility_list = *map.getVisibilityList(
    //     poses[i].translation.x(), poses[i].translation.y());
    for (unsigned int j = 0; j < point_clouds[i].size(); ++j) {
      const Vector2f p = pose_transform * point_clouds[i][j];
      if (kDisplaySteps) {
        visualization::DrawLine(p, poses[i].translation, 0x1FC0C0C0, visualization_msg_);
      }
      if (!kSaveAllObservations &&
          classifications[i][j] != NonMarkovLocalization::kStfObservation) {
        if (kDisplaySteps) {
          visualization::DrawPoint(p, kLtfPointColor, visualization_msg_);
        }
        continue;
      }
      if (!kSaveAllObservations) {
        CHECK(false) << "ERROR: Unimplemented " << __FILE__ << ":" << __LINE__;
        // const Vector2f p_g(p.x(), p.y());
        // bool map_match = false;
        // for (size_t l = 0; !map_match && l < visibility_list.size(); ++l) {
        //   const float line_offset =
        //       map.Line(visibility_list[l]).closestDistFromLine(p_g, false);
        //   const bool along_line =
        //       (p_g - map.Line(visibility_list[l]).P0()).dot(
        //       p_g - map.Line(visibility_list[l]).P1()) < 0.0;
        //   map_match = along_line && line_offset < kMaxMapLineOffset;
        // }
        // if (map_match) {
        //   if (kDisplaySteps) {
        //     DrawPoint(p, kLtfPointColor, &display_message_);
        //   }
        //   continue;
        // }
      }
      const Vector2f n = pose_rotation * normal_clouds[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f, %.4f,%.4f, %.4f,%.4f\n",
          poses[i].translation.x(), poses[i].translation.y(),
          poses[i].angle, p.x(), p.y(), n.x(), n.y());
      if (kDisplaySteps) {
        visualization::DrawPoint(p, kStfPointColor, visualization_msg_);
      }
    }
    if (kDisplaySteps) {
      PublishLocation(kMapName, poses[i].translation.x(),
                      poses[i].translation.y(), poses[i].angle);
      PublishDisplay();
      while (kbhit() == 0) {
        Sleep(0.02);
      }
      fgetc(stdin);
      printf("\r");
      fflush(stdout);
    }
  }
  if (kDisplaySteps) {
    PublishDisplay();
  }
}

void SaveEpisodeStats(FILE* fid) {
  vector<Pose2Df> poses;
  vector<PointCloudf> point_clouds;
  vector<Pose2Df> pending_poses;
  vector<uint64_t> pose_ids;
  vector<vector<NonMarkovLocalization::ObservationType> > observation_classes;
  Pose2Df latest_pose;
  if (!localization_->GetNodeData(
          &poses,
          &pose_ids,
          &point_clouds,
          &pending_poses,
          &observation_classes,
          &latest_pose) ||
      poses.size() < 1 ||
      poses.size() != observation_classes.size()
     ) {
    return;
  }
  const string map_name = localization_->GetCurrentMapName();
  int num_ltfs = 0;
  int num_stfs = 0;
  int num_dfs = 0;
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      switch (observation_classes[i][j]) {
        case NonMarkovLocalization::kLtfObservation : {
          ++num_ltfs;
        } break;
        case NonMarkovLocalization::kStfObservation : {
          ++num_stfs;
        } break;
        case NonMarkovLocalization::kDfObservation : {
          ++num_dfs;
        } break;
        default : {
        } break;
      }
    }
  }
  const int episode_length = static_cast<int>(point_clouds.size());
  fprintf(fid,
          "%s, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d\n",
          map_name.c_str(),
          poses.back().translation.x(),
          poses.back().translation.y(),
          poses.back().angle,
          poses.front().translation.x(),
          poses.front().translation.y(),
          poses.front().angle,
          episode_length,
          num_ltfs,
          num_stfs,
          num_dfs);
}

bool AddPose(const sensor_msgs::LaserScanPtr& laser_message,
             const vector<double>& keyframes,
             Vector2f* relative_location_ptr,
             float* relative_angle_ptr,
             Vector2f* global_location_ptr,
             float* global_angle_ptr,
             vector<PointCloudf>* point_clouds,
             vector<NormalCloudf>* normal_clouds,
             vector<Pose2Df>* poses,
              vector<double>* timestamps,
             Vector2f* odometry_location,
             float* odometry_angle) {
  static const bool debug = false;
  // Aliasing dereferenced pointers for readability.
  float& relative_angle = *relative_angle_ptr;
  Vector2f& relative_location = *relative_location_ptr;
  float& global_angle = *global_angle_ptr;
  Vector2f& global_location = *global_location_ptr;
  const bool keyframe = binary_search(keyframes.begin(), keyframes.end(),
                                      laser_message->header.stamp.toSec());
  if (!keyframe && relative_location.norm() <
      localization_options_.minimum_node_translation &&
      fabs(relative_angle) < localization_options_.minimum_node_rotation) {
    // Ignore this pose since the robot has not moved since the last update.
    return false;
  }
  if (debug) {
    printf("Relative pose: %6.3f, %6.3f %4.1f\u00b0\n", relative_location.x(),
        relative_location.y(), DegToRad(relative_angle));
  }
  // Update global pose
  global_location = Rotation2Df(global_angle) * relative_location +
      global_location;
  global_angle = relative_angle + global_angle;

  // Update odometry pose
  *odometry_location = Rotation2Df(*odometry_angle) * relative_location +
      *odometry_location;
  *odometry_angle = relative_angle + *odometry_angle;

  // Compute global pose and covariance.
  Pose2Df pose;
  pose.translation = global_location;
  pose.angle = global_angle;

  // Add laser scan to list of entries.
  PointCloudf point_cloud;
  const Vector2f sensor_offset(
      localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
  for (unsigned int i = 0; i < laser_message->ranges.size(); ++i) {
    if (laser_message->ranges[i] < kMinPointCloudRange ||
        laser_message->ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message->ranges[i])) continue;
    const float angle = laser_message->angle_min +
        laser_message->angle_increment * static_cast<float>(i);
    if (angle < laser_message->angle_min + DegToRad(15.0) ||
        angle > laser_message->angle_max - DegToRad(15.0)) continue;
    float range = laser_message->ranges[i];
    if (use_laser_corrections_) {
      const int angle_index =
          floor((angle + M_PI) / laser_corrections_resolution_);
      CHECK_GE(angle_index, 0);
      CHECK_LT(angle_index, static_cast<int>(laser_corrections_.size()));
      range = range * laser_corrections_[angle_index];
    }
    point_cloud.push_back(
        sensor_offset + Rotation2Df(angle) * Vector2f(range, 0.0));
  }

  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud, &normal_cloud);
  if (point_cloud.size() > 1 || 1) {
    point_clouds->push_back(point_cloud);
    normal_clouds->push_back(normal_cloud);
    poses->push_back(pose);
    timestamps->push_back(laser_message->header.stamp.toSec());
    // Reset odometry accumulation.
    relative_angle = 0.0;
    relative_location = Vector2f(0.0, 0.0);
  }
  return true;
}

bool LoadLaserMessage(const rosbag::MessageInstance& message,
                      const vector<double>& keyframes,
                      Vector2f* relative_location,
                      float* relative_angle,
                      Vector2f* global_location,
                      float* global_angle,
                      vector<PointCloudf>* point_clouds,
                      vector<NormalCloudf>* normal_clouds,
                      vector<Pose2Df>* poses,
                      vector<double>* timestamps,
                      Vector2f* odometry_location,
                      float* odometry_angle) {
  sensor_msgs::LaserScanPtr laser_message =
      message.instantiate<sensor_msgs::LaserScan>();
  if (laser_message != NULL &&
      message.getTopic() == CONFIG_scan_topic) {
    if (false && debug_level_ > 1) {
      printf("Laser Msg,    t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    AddPose(laser_message, keyframes, relative_location, relative_angle,
            global_location, global_angle, point_clouds, normal_clouds,
            poses, timestamps, odometry_location, odometry_angle);
    return true;
  }
  return false;
}

bool LoadOdometryMessage(const rosbag::MessageInstance& message,
                         const Vector2f& odometry_location,
                         const float& odometry_angle,
                         Vector2f* relative_location,
                         float* relative_angle) {
  nav_msgs::OdometryPtr odometry_message =
      message.instantiate<nav_msgs::Odometry>();
  const string topic_name = message.getTopic();
  if (odometry_message != NULL &&
      message.getTopic() == CONFIG_odom_topic) {
    if (debug_level_ > 2) {
      printf("Odometry Msg, t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    const Vector2f odometry_message_location(
        odometry_message->pose.pose.position.x,
        odometry_message->pose.pose.position.y);
    *relative_location =  kOdometryTranslationScale * (
        Rotation2Df(-odometry_angle) *
        (odometry_message_location - odometry_location));
    const float odometry_message_angle =
        2.0 * atan2(odometry_message->pose.pose.orientation.z,
                    odometry_message->pose.pose.orientation.w);
    *relative_angle = kOdometryRotationScale *
        AngleDiff(odometry_message_angle , odometry_angle);
    if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
      relative_location->x() +=
          rand_.Gaussian(0, odometry_additive_noise_ * relative_location->x());
      relative_location->y() +=
          rand_.Gaussian(0, odometry_additive_noise_ * relative_location->y());
      (*relative_angle) +=
          rand_.Gaussian(0, odometry_additive_noise_ * (*relative_angle));
    }
    return true;
  }

  return false;
}

bool LoadSetLocationMessage(const rosbag::MessageInstance& message,
                            Vector2f* global_location,
                            float* global_angle,
                            string* map_name) {
  amrl_msgs::Localization2DMsgPtr set_location_message =
      message.instantiate<amrl_msgs::Localization2DMsg>();
  const string topic_name = message.getTopic();
  if (set_location_message != NULL &&
      message.getTopic() == CONFIG_initialpose_topic)  {
    if (debug_level_ > 1) {
      printf("Set Location, t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    *global_angle = set_location_message->pose.theta;
    global_location->x() = set_location_message->pose.x;
    global_location->y() = set_location_message->pose.y;
    *map_name = set_location_message->map;
    return true;
  }
  return false;
}

void LoadRosBag(const string& bagName, int max_laser_poses, double time_skip,
                const vector<double>& keyframes,
                vector<PointCloudf>* point_clouds,
                vector<NormalCloudf>* normal_clouds,
                vector<Pose2Df>* poses,
                vector<double>* timestamps,
                double* t_duration,
                string* map_name) {
  CHECK_NOTNULL(point_clouds);
  CHECK_NOTNULL(normal_clouds);
  CHECK_NOTNULL(poses);
  // Last angle as determined by integration of robot odometry from an arbitrary
  // starting location.
  float odometry_angle(0.0);
  // Last location as determined by integration of robot odometry from an
  // arbitrary starting location.
  Vector2f odometry_location(0.0, 0.0);
  // Last angle as determined by integration of robot odometry since location of
  // last pose update.
  float relative_angle(0.0);
  // Last location as determined by integration of robot odometry since location
  // of last pose update.
  Vector2f relative_location(0.0, 0.0);
  // Estimated robot global angle as determined by integration of robot odometry
  // from set starting location.
  float global_angle(kStartingAngle);
  // Estimated robot global location as determined by integration of robot
  // odometry from set starting location.
  Vector2f global_location(kStartingLocation);
  rosbag::Bag bag;
  printf("Opening bag file %s...", bagName.c_str()); fflush(stdout);
  bag.open(bagName,rosbag::bagmode::Read);
  printf(" Done.\n"); fflush(stdout);

  const std::vector<std::string> topics({
    CONFIG_scan_topic,
    CONFIG_odom_topic,
    CONFIG_initialpose_topic
  });

  printf("Reading bag file..."); fflush(stdout);
  if (false && debug_level_ > 1) printf("\n");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double bag_time_start = -1.0;
  double bag_time = 0.0;
  for (rosbag::View::iterator it = view.begin();
       run_ && it != view.end() &&
       (max_laser_poses < 0 ||
       static_cast<int>(poses->size()) < max_laser_poses); ++it) {
    const rosbag::MessageInstance &message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }

    // Ignore messages before elapsed time_skip.
    if (bag_time < bag_time_start + time_skip) continue;

    if (LoadSetLocationMessage(message, &global_location,
                                &global_angle, map_name)) {
      continue;
    }

    // Check to see if this is a laser scan message.
    if (LoadLaserMessage(message, keyframes, &relative_location,
        &relative_angle, &global_location, &global_angle, point_clouds,
        normal_clouds, poses, timestamps, &odometry_location,
        &odometry_angle)) {
      continue;
    }

    // Check to see if this is an odometry message.
    if (LoadOdometryMessage(message, odometry_location, odometry_angle,
      &relative_location, &relative_angle)) {
      continue;
    }
  }
  *t_duration = bag_time - bag_time_start;
  if (false && debug_level_ > 1) printf("\n");
  printf(" Done.\n"); fflush(stdout);
  printf("%d poses loaded.\n", static_cast<int>(point_clouds->size()));
}

void DrawVisibilityConstraints(
    const vector<vector_localization::VisibilityGlobConstraint*>&
        visibility_constraints,
    const vector<double>& poses) {
  return;
  // TODO(Joydeepb): This causes a segfault when "enml.limit_history = true;".
  for (size_t i = 0; i < visibility_constraints.size(); ++i) {
    const vector_localization::VisibilityGlobConstraint& constraint =
        *visibility_constraints[i];
    const size_t& pose_index = constraint.pose_index;
    const Vector2f pose_translation(
        poses[3 * pose_index], poses[3 * pose_index + 1]);
    const Affine2f pose_transform =
        Translation2f(poses[3 * pose_index], poses[3 * pose_index + 1]) *
        Rotation2Df(poses[3 * pose_index + 2]);
    for (size_t j = 0; j < constraint.points.size(); ++j) {
      const Vector2f point_global = pose_transform * constraint.points[j];
      // Compute the line offset error from the constraint.
      const float point_offset =
          constraint.line_normals[j].dot(point_global - constraint.line_p1s[j]);
      const Vector2f point_anchor =
          point_global - constraint.line_normals[j] * point_offset;
      visualization::DrawLine(point_global, point_anchor, 0xFFFF0000, visualization_msg_);
      const float alpha = Clamp(fabs(point_offset) / 5.0, 0.0, 0.2);
      const uint32_t colour =
          (static_cast<uint32_t>(255.0 * alpha) << 24) | 0xFF0000;
      visualization::DrawLine(point_global, pose_translation, colour, visualization_msg_);
    }
  }
}

void DrawStfs(
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
    point_point_correspondences, const vector<double>& poses,
    const vector<vector< Vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds) {
  const double n = point_point_correspondences.size();
  double mse = 0;
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    CHECK_EQ(point_point_correspondences[i].points0.size(),
             point_point_correspondences[i].points1.size());
    const Affine2f pose_tf0 =
        Translation2f(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]) *
        Rotation2Df(poses[3 * pose_index0 + 2]);
    const Affine2f pose_tf1 =
        Translation2f(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]) *
        Rotation2Df(poses[3 * pose_index1 + 2]);
    for (size_t j = 0; j < point_point_correspondences[i].points0.size(); ++j) {
      const Vector2f p0 = pose_tf0 * point_point_correspondences[i].points0[j];
      const Vector2f p1 = pose_tf1 * point_point_correspondences[i].points1[j];
      visualization::DrawLine(p0, p1, kStfCorrespondenceColor, visualization_msg_);
      mse += (p0 - p1).squaredNorm() / n;
    }
  }
  if (false) {
    printf("\nmse:%f n:%f\n", mse, n);
    if (mse > 2.0 && n > 20.0) run_ = false;
  }
}


void DrawLtfs(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses, const vector<vector< Vector2f> >& point_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const vector<vector<Line2f> >& ray_cast_lines,
    const vector<vector<int> >& point_line_correspondences) {
  static const bool kDrawPredictedScan = false;
  if (kDrawPredictedScan) {
    const Vector2f sensor_offset(
        localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
    static VectorMap map_;
    map_.Load(localization_->GetCurrentMapName());
    for (size_t i = start_pose; i <= end_pose; ++i) {
    // for (size_t i = start_pose; i <= start_pose; ++i) {
      const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
      const float pose_angle = poses[3 * i + 2];
      const Rotation2Df pose_rotation(pose_angle);
      const Vector2f laser_loc =
          pose_location + pose_rotation * sensor_offset;
      static const int kNumRays = 180;
      static const float kAngleMax = DegToRad(135);
      vector<float> scan;
      map_.GetPredictedScan(
          laser_loc,
          0,
          localization_options_.kMaxRange,
          pose_angle - kAngleMax,
          pose_angle + kAngleMax,
          kNumRays,
          &scan);
      CHECK_EQ(kNumRays, scan.size());
      const float da = 2.0f * kAngleMax / float(kNumRays);
      for (size_t j = 0; j < kNumRays; ++j) {
        if (scan[j] > 0.95 * localization_options_.kMaxRange) continue;
        const float a = pose_angle -kAngleMax + float(j) * da;
        const Vector2f p = laser_loc + Vector2f(scan[j] * cos(a), scan[j] * sin(a));
        visualization::DrawLine(laser_loc, p, 0x0FFF0000, visualization_msg_);
      }
    }
  }

  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    for (size_t j = 0; j < point_clouds[i].size(); ++j) {
      if (classifications[i][j] != NonMarkovLocalization::kLtfObservation) {
        continue;
      }
      const Vector2f& point = Rotation2Df(pose_angle) * point_clouds[i][j] + pose_location;
      const int correspondence = point_line_correspondences[i][j];
      const Line2f& line = ray_cast_lines[i][correspondence];
      const Vector2f point_projected = line.Projection(point);
      visualization::DrawLine(point, point_projected, kLtfCorrespondenceColor,
              visualization_msg_);
    }
  }
}

void DrawObservations(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses,
    const vector<vector< Vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications) {
  static const bool kDisplayTangents = false;
  int num_ltfs = 0, num_stfs = 0, num_dfs = 0;
  for (size_t i = 0; i <= end_pose; ++i) {
    const vector<Vector2f> &point_cloud = point_clouds[i];
    const vector<Vector2f> &normal_cloud = normal_clouds[i];
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    const Rotation2Df pose_rotation(pose_angle);
    const Affine2f pose_transform =
        Translation2f(pose_location) * pose_rotation;
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      const Vector2f point = pose_transform * point_cloud[j];
      uint32_t point_color = 0xF0C0C0C0;
      bool valid_point = false;
      if (i >= start_pose) {
        switch (classifications[i][j]) {
          case NonMarkovLocalization::kLtfObservation : {
            point_color = kLtfPointColor;
            ++num_ltfs;
          } break;
          case NonMarkovLocalization::kStfObservation : {
            // DrawLine(point, pose_location, 0x1F994CD9, &display_message_);
            point_color = kStfPointColor;
            ++num_stfs;
            valid_point = true;
          } break;
          case NonMarkovLocalization::kDfObservation : {
            point_color = kDfPointColor;
            ++num_dfs;
            valid_point = true;
            if (i == end_pose) continue;
          } break;
        }
      }
      if (kDisplayTangents && valid_point) {
        const Vector2f normal = pose_rotation * normal_cloud[j];
        // const Vector2f tangent = 0.05 * Perp(normal);
        const Vector2f dir = 0.05 * normal;
        visualization::DrawLine(
            point + dir, point - dir, 0x7FFF4000, visualization_msg_);
      }
      visualization::DrawPoint(point, point_color, visualization_msg_);
    }
  }
  // printf("LTFs:%10d STFs:%10d DFs:%10d\n", num_ltfs, num_stfs, num_dfs);
}

void DrawGradients(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& gradients,
    const vector<double>& poses) {
  if (gradients.size() != poses.size()) return;
  for (size_t i = start_pose, j = 3 * start_pose;
       i <= end_pose; ++i, j += 3) {
    const Vector2f location_gradient(gradients[j], gradients[j + 1]);
    const Vector2f pose_location(poses[j], poses[j + 1]);
    const Rotation2Df pose_rotation(poses[j + 2]);
    const Vector2f p2 = pose_location - location_gradient;
    visualization::DrawLine(pose_location, p2, 0xFF0000FF, visualization_msg_);
  }
}

void DrawPoseCovariance(const Vector2f& pose, const Matrix2f& covariance) {
  static const float kDTheta = DegToRad(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;
    visualization::DrawLine(v1_global, v2_global, kPoseCovarianceColor, visualization_msg_);
  }
}

void DrawPoses(const size_t start_pose, const size_t end_pose,
    const vector<Pose2Df>& odometry_poses, const vector<double>& poses,
    const vector<Matrix2f>& covariances) {
  static const bool kDrawCovariances = false;
  Vector2f pose_location_last(0.0, 0.0);
  float pose_angle_last = 0.0;
  const bool valid_covariances = (covariances.size() > end_pose);
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    if (i > start_pose) {
      visualization::DrawLine(pose_location, pose_location_last, kTrajectoryColor, visualization_msg_);
      const Vector2f odometry =
          Rotation2Df(pose_angle_last) *
          Rotation2Df(-odometry_poses[i - 1].angle) *
          (odometry_poses[i].translation - odometry_poses[i - 1].translation);
      visualization::DrawLine(pose_location, Vector2f(pose_location_last + odometry), kOdometryColor, visualization_msg_);
      if (kDrawCovariances && valid_covariances) {
        DrawPoseCovariance(pose_location, covariances[i]);
      }
    }
    pose_location_last = pose_location;
    pose_angle_last = poses[3 * i + 2];
  }
}

void DrawRayCasts(
    const size_t start_pose, const size_t end_pose,
    const vector<vector<Line2f> >& ray_cast_lines,
    const vector<double>& poses) {
  const uint32_t colour = 0xFFCF8D00;
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    for (size_t j = 0; j < ray_cast_lines[i].size(); ++j) {
      visualization::DrawLine(ray_cast_lines[i][j].p0, ray_cast_lines[i][j].p1,
               colour, visualization_msg_);
      /*
      DrawLine(pose_location, ray_cast_lines[i][j].P0(),
               colour, &display_message_);
      DrawLine(pose_location, ray_cast_lines[i][j].P1(),
               colour, &display_message_);
      */
    }
  }
}

void DrawFactor(const Vector2f& p0,
                const Vector2f& p1,
                const uint32_t edge_color,
                const uint32_t factor_node_color,
                const uint32_t variable_node_color) {
  visualization::DrawPoint(p0, variable_node_color, visualization_msg_);
  visualization::DrawPoint(p1, variable_node_color, visualization_msg_);
  visualization::DrawPoint(Vector2f(0.5 * (p0 + p1)), factor_node_color, visualization_msg_);
  visualization::DrawLine(p0, p1, edge_color, visualization_msg_);
}

void DrawFactorGraph(
    const vector<double>& poses,
    const vector<vector<int> >& point_line_correspondences,
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const size_t start_pose, const size_t end_pose) {
  static const uint32_t kFactorEdgeColor = 0xC0000000;
  static const uint32_t kFactorFactorNodeColor = 0xAF00FF00;
  static const uint32_t kFactorVariableNodeColor = 0x6F0000FF;
  static const bool kDrawFactorGraph = true;

  if (!kDrawFactorGraph) return;

  // Draw factor for odometry.
  for (size_t i = start_pose; i < end_pose; ++i) {
    const Vector2f p0(poses[3 * i], poses[3 * i + 1]);
    const Vector2f p1(poses[3 * (i + 1)], poses[3 * (i + 1) + 1]);
    DrawFactor(p0,
               p1,
               kFactorEdgeColor,
               kFactorFactorNodeColor,
               kFactorVariableNodeColor);
  }

  // Draw factors for STFs
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    if (point_point_correspondences[i].points0.size() < 1) continue;
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    const Vector2f p0(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]);
    const Vector2f p1(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]);
    DrawFactor(p0,
               p1,
               kFactorEdgeColor,
               kFactorFactorNodeColor,
               kFactorVariableNodeColor);
  }
}

void CorrespondenceCallback(
    const vector<double>& poses,
    const vector<vector< Vector2f> >& point_clouds,
    const vector< NormalCloudf >& normal_clouds,
    const vector<vector<Line2f> >& ray_cast_lines,
    const vector<vector<int> >& point_line_correspondences,
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const vector<vector_localization::VisibilityGlobConstraint*>&
        visibility_constraints,
    const vector<double>& gradients,
    const vector<Matrix2f>& covariances,
    const vector<Pose2Df>& odometry_poses,
    const size_t start_pose, const size_t end_pose) {
  static const bool kDisplayLtfCorrespondences = false;
  static const bool kDisplayStfCorrespondences = false;
  static const bool kDisplayRayCasts = false;
  static const bool kDisplayFactorGraph = false;
  CHECK_EQ(poses.size(), point_clouds.size() * 3);
  CHECK_EQ(point_clouds.size(), ray_cast_lines.size());
  CHECK_EQ(point_clouds.size(), point_line_correspondences.size());
  ClearDisplay();
  if (!run_) {
    localization_->Terminate();
  }
  if (debug_level_ >= 1) {
    PublishTrace();
  }
  DrawPoses(start_pose, end_pose, odometry_poses, poses, covariances);
  DrawGradients(start_pose, end_pose, gradients, poses);
  DrawObservations(start_pose, end_pose, poses, point_clouds, normal_clouds,
                   classifications);
  DrawVisibilityConstraints(visibility_constraints, poses);
  if (kDisplayRayCasts) {
    DrawRayCasts(start_pose, end_pose, ray_cast_lines, poses);
  }
  if (kDisplayFactorGraph) {
    DrawFactorGraph(poses,
                    point_line_correspondences,
                    point_point_correspondences,
                    start_pose,
                    end_pose);
  }
  if (kDisplayLtfCorrespondences) {
    DrawLtfs(start_pose, end_pose, poses, point_clouds, classifications,
             ray_cast_lines, point_line_correspondences);
  }
  if (kDisplayStfCorrespondences) {
    DrawStfs(point_point_correspondences, poses, point_clouds, normal_clouds);
  }
  PublishDisplay();
}

void SaveLoggedPoses(const string& filename,
                     const vector<Pose2Df>& logged_poses,
                     const vector<double>& timestamps) {
  ScopedFile fid(filename, "w");
  for (size_t i = 0; i < logged_poses.size(); ++i) {
    fprintf(fid(), "%f %f %f %f\n",
            timestamps[i],
            logged_poses[i].translation.x(),
            logged_poses[i].translation.y(),
            logged_poses[i].angle);
  }
}

void LoadKeyframes(const string& file, vector<double>* keyframes) {
  ScopedFile fid(file, "r");
  if (fid() == NULL) return;
  double t = 0.0;
  keyframes->clear();
  while (fscanf(fid(), "%lf\n", &t) == 1) {
    keyframes->push_back(t);
  }
  sort(keyframes->begin(), keyframes->end());
  printf("Loaded %lu keyframes\n", keyframes->size());
}

void SaveSensorErrors(
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications) {
  VectorMap map(std::string(maps_dir_) + "/" + kMapName + ".txt");
  ScopedFile fid("results/sensor_errors.txt", "w");
  printf("Saving sensor errors... ");
  fflush(stdout);
  visualization::ClearVisualizationMsg(visualization_msg_);
  const Vector2f laser_loc(
      localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
  for (size_t i = 0; i < poses.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Vector2f pose_loc_g =
        poses[i].translation + Rotation2Df(poses[i].angle) * laser_loc;
    vector<Vector2f> point_cloud_g(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      point_cloud_g[j] = Vector2f(point_cloud[j].x(), point_cloud[j].y());
    }
    PointCloudf predicted_point_cloud;
    map.GetPredictedPointCloud(pose_loc_g, 0.01, localization_options_.kMaxRange, point_cloud_g, &predicted_point_cloud);
    const Affine2f pose_tf = Translation2f(pose_loc_g) * Rotation2Df(poses[i].angle);
    for (size_t j = 0; j < point_cloud_g.size(); ++j) {
      const Vector2f observed_point = pose_tf * point_cloud_g[j];
      const Vector2f expected_point = predicted_point_cloud[j];
      const float observed_range = (point_cloud_g[j] - laser_loc).norm();
      const float expected_range = (expected_point - pose_loc_g).norm();
      const bool not_expected = expected_range >= localization_options_.kMaxRange;
      if (classifications[i][j] != NonMarkovLocalization::kLtfObservation ||
          not_expected) {
        continue;
      }
      visualization::DrawPoint(expected_point, 0xFFFF0000, visualization_msg_);
      visualization::DrawPoint(observed_point, 0x4FFF7700, visualization_msg_);
      visualization::DrawLine(expected_point, pose_loc_g, 0x4FC0C0C0, visualization_msg_);
      const float angle = atan2(point_cloud_g[j].y(), point_cloud_g[j].x());
      fprintf(fid(), "%.4f %.4f %.4f\n", angle, expected_range, observed_range);
    }
  }
  PublishDisplay();
  printf("Done.\n");
}

void StandardOdometryCallback(const nav_msgs::Odometry& last_odometry_msg,
                              const nav_msgs::Odometry& odometry_msg) {
  const float old_theta = 2.0 * atan2(
      last_odometry_msg.pose.pose.orientation.z,
      last_odometry_msg.pose.pose.orientation.w);
  const float new_theta = 2.0 * atan2(
      odometry_msg.pose.pose.orientation.z,
      odometry_msg.pose.pose.orientation.w);
  const Vector2f p_old(last_odometry_msg.pose.pose.position.x,
                       last_odometry_msg.pose.pose.position.y);
  const Vector2f p_new(odometry_msg.pose.pose.position.x,
                       odometry_msg.pose.pose.position.y);
  Vector2f p_delta =
      Rotation2Df(-old_theta) * (p_new - p_old);
  float d_theta = AngleDiff(new_theta, old_theta);
  if (debug_level_ > 1) {
    printf("Standard Odometry %8.3f %8.3f %8.3f, t=%f\n",
           p_delta.x(), p_delta.y(), RadToDeg(d_theta),
           odometry_msg.header.stamp.toSec());
  }
  if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
    ApplyNoiseModel(p_delta.x(),
                    p_delta.y(),
                    d_theta,
                    odometry_additive_noise_,
                    &(p_delta.x()),
                    &(p_delta.y()),
                    &(d_theta));
  }
  localization_->OdometryUpdate(
      kOdometryTranslationScale * p_delta.x(),
      kOdometryTranslationScale * p_delta.y(), d_theta);
  PublishLocation();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  static nav_msgs::Odometry last_msg_;
  static const float kMaxDist = 2.0;
  static Vector2f last_pos(0, 0);
  const Vector2f new_pos(msg.pose.pose.position.x, msg.pose.pose.position.y);
  if ((new_pos - last_pos).squaredNorm() < Sq(kMaxDist)) {
    StandardOdometryCallback(last_msg_, msg);
  }
  last_pos = new_pos;
  last_msg_ = msg;
}

void LaserCallback(const sensor_msgs::LaserScan& laser_message) {
  static vector<float> normal_weights_;
  if (normal_weights_.size() == 0) {
    static const float stddev = 3;
    for (int i = 0; i < 3; ++i) {
      normal_weights_.push_back(exp(-Sq(float(i) / stddev)));
    }
  }
  if (debug_level_ > 1) {
    printf("LaserScan, t=%f\n", laser_message.header.stamp.toSec());
  }
  PointCloudf point_cloud;
  const Vector2f sensor_offset(
      localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
  for (size_t i = 0; i < laser_message.ranges.size(); ++i) {
    if (laser_message.ranges[i] < kMinPointCloudRange ||
        laser_message.ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message.ranges[i])) continue;
    const float angle = laser_message.angle_min +
        laser_message.angle_increment * static_cast<float>(i);
    float range = laser_message.ranges[i];
    point_cloud.push_back(
        sensor_offset + Rotation2Df(angle) * Vector2f(range, 0.0));
  }
  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance,
      normal_weights_, &point_cloud, &normal_cloud);
  if (point_cloud.size() > 1) {
    if (debug_level_ > 1) {
      printf("Sensor update, t=%f\n", laser_message.header.stamp.toSec());
    }
    localization_->SensorUpdate(point_cloud, normal_cloud);
  }
  last_laser_scan_ = laser_message;
}

// GUI interaction state enumeration.
enum GuiInteractionState {
  kGuiStateNone = 0,
  kGuiStateDraw = 1,
};

bool LaserPointComparator(const pair<float, Vector2f>& p1,
                          const pair<float, Vector2f>& p2) {
  return (p1.first < p2.first);
}

float CheckLaserOverlap(const sensor_msgs::LaserScan& scan1,
                       const Pose2Df& pose1,
                       const sensor_msgs::LaserScan& scan2,
                       const Pose2Df& pose2) {
  static ScopedFile fid("overlap.txt", "w");
  const vector<float>& ranges1 = scan1.ranges;
  const vector<float>& ranges2 = scan2.ranges;
  if (ranges1.size() != ranges2.size()) return 0.0;

  // Compute the relative delta transform.
  const Vector2f delta_loc =
      Rotation2Df(-pose1.angle) * (pose2.translation - pose1.translation);
  const float delta_angle = AngleDiff(pose2.angle, pose1.angle);
  const Affine2f delta_transform =
      Translation2f(delta_loc) * Rotation2Df(delta_angle);
  vector<pair<float, Vector2f> > new_point_cloud(ranges2.size());
  for (size_t i = 0; i < ranges2.size(); ++i) {
    const float scan_angle =
        scan2.angle_min + scan2.angle_increment * static_cast<float>(i);
    const Vector2f scan_point = delta_transform * Rotation2Df(scan_angle) *
        Vector2f(ranges2[i], 0.0);
    const float point_angle = atan2(scan_point.y(), scan_point.x());
    new_point_cloud[i] = std::make_pair(point_angle, scan_point);
  }
  std::sort(new_point_cloud.begin(), new_point_cloud.end(),
      LaserPointComparator);
  float diff = 0.0;
  size_t j = 0;
  for (size_t i = 0; i < ranges1.size(); ++i) {
    if (ranges1[i] < scan1.range_min || ranges1[i] > scan1.range_max ||
        ranges2[i] < scan2.range_min || ranges2[i] > scan2.range_max) {
      continue;
    }
    const float scan_angle =
        scan1.angle_min + scan1.angle_increment * static_cast<float>(i);
    for(; j < new_point_cloud.size() &&
        new_point_cloud[j].first < scan_angle; ++j) {
    }
    diff += fabs(ranges1[i] - new_point_cloud[j].second.norm());
  }
  fprintf(fid(), "%f\n", diff);
  // printf("Laser diff: %6.2f mean: %6.4f\n",
  //        diff, diff / static_cast<float>(ranges1.size()));
  return diff;
}

void SRLVisualize(
    const vector<double>& poses,
    const PointCloudf& point_cloud_e,
    const NormalCloudf& normal_cloud,
    const vector<vector_localization::LTSConstraint*>& constraints) {
  const size_t num_samples = poses.size() / 3;
  visualization::ClearVisualizationMsg(visualization_msg_);
  double max_weight = 0.0;
  size_t best_pose = 0;
  vector<double> pose_weights(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    const Pose2Df pose(poses[3 * i + 2], poses[3 * i + 0], poses[3 * i + 1]);
    pose_weights[i] = localization_->ObservationLikelihood(
        pose, point_cloud_e, normal_cloud);
    if (pose_weights[i] > max_weight) {
      max_weight = pose_weights[i];
      best_pose = i;
    }
  }
  printf("Best SRL weight: %23.20f\n", max_weight);
  for (size_t i = 0; i < num_samples; ++i) {
    const Affine2f pose_transform =
        Translation2f(poses[3 * i + 0], poses[3 * i + 1]) *
        Rotation2Df(poses[3 * i + 2]);
    const Vector2f line_p0(poses[3 * i + 0], poses[3 * i + 1]);
    const Vector2f line_p1 =
        line_p0 + Rotation2Df(poses[3 * i + 2]) * Vector2f(0.3, 0.0);
    const double alpha = max(0.05, pose_weights[i] / max_weight);
    const uint32_t alpha_channel =
        (static_cast<uint32_t>(255.0 * alpha) << 24) & 0xFF000000;
    visualization::DrawPoint(line_p0, alpha_channel | 0xFF0000, visualization_msg_);
    visualization::DrawLine(line_p0, line_p1, alpha_channel | 0x3F3F3F, visualization_msg_);
    if (pose_weights[i] == max_weight) {
      for (size_t j = 0; j < point_cloud_e.size(); ++j) {
        const Vector2f point = pose_transform * point_cloud_e[j];
        visualization::DrawPoint(point, 0xFFFF7700, visualization_msg_);
        visualization::DrawLine(point, line_p0, 0x1FFF7700, visualization_msg_);
      }
    }
  }
  const Affine2f best_pose_transform =
      Translation2f(poses[3 * best_pose + 0], poses[3 * best_pose + 1]) *
      Rotation2Df(poses[3 * best_pose + 2]);
  for (size_t i = 0; i < constraints.size(); ++i) {
    if (constraints[i]->pose_index != best_pose) continue;
    const vector_localization::LTSConstraint& constraint = *constraints[i];
    const Vector2f p = best_pose_transform * constraint.point;
    const float offset = p.dot(constraint.line_normal) + constraint.line_offset;
    const Vector2f p_line = p - constraint.line_normal * offset;
    visualization::DrawLine(p, p_line, 0xFF00FF00, visualization_msg_);
  }
  PublishDisplay();
}

void SensorResettingResample(const sensor_msgs::LaserScan& laser_message) {
  printf("SRL with laser t=%f\n", laser_message.header.stamp.toSec());
  PointCloudf point_cloud;
  const Vector2f sensor_offset(
      localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
  for (size_t i = 0; i < laser_message.ranges.size(); ++i) {
    if (laser_message.ranges[i] < kMinPointCloudRange ||
        laser_message.ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message.ranges[i])) continue;
    const float angle = laser_message.angle_min +
        laser_message.angle_increment * static_cast<float>(i);
    point_cloud.push_back(sensor_offset +
        Rotation2Df(angle) * Vector2f(laser_message.ranges[i], 0.0));
  }
  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud, &normal_cloud);

  vector<Pose2Df> poses;
  vector<double> pose_weights;
  vector<vector_localization::LTSConstraint*> constraints;
  static const size_t kNumSamples = 500;
  static const float kTangentialStdDev = 0.3;
  static const float kRadialStdDev = 0.3;
  static const float kAngularStdDev = DegToRad(20.0);
  printf("Running SRL...");
  fflush(stdout);
  localization_->SensorResettingResample(
      point_cloud, normal_cloud, kNumSamples, kRadialStdDev,
      kTangentialStdDev, kAngularStdDev, &SRLVisualize, &poses, &pose_weights,
      &constraints);
  printf(" Done.\n");
  fflush(stdout);

  double max_weight = 0.0;
  for (size_t i = 0; i < pose_weights.size(); ++i) {
    max_weight = max(max_weight, pose_weights[i]);
  }
  printf("Best weight: %f\n", max_weight);

  vector<double> pose_array(poses.size() * 3);
  for (size_t i = 0; i < poses.size(); ++i) {
    pose_array[3 * i + 0] = poses[i].translation.x();
    pose_array[3 * i + 1] = poses[i].translation.y();
    pose_array[3 * i + 2] = poses[i].angle;
  }
  SRLVisualize(pose_array, point_cloud, normal_cloud, constraints);
}

void InteractivePause() {
  bool ready = false;
  while(run_ && !ready) {
    if (kbhit() != 0) {
      switch(fgetc(stdin)) {
        case ' ': {
          ready = true;
        } break;
        case 'q': {
          run_ = false;
        } break;
      }
      printf("\n"); fflush(stdout);
    }
    Sleep(0.05);
  }
}

void PlayBagFile(const string& bag_file,
                 int max_poses,
                 bool use_point_constraints,
                 double time_skip,
                 ros::NodeHandle* node_handle,
                 double* observation_error_ptr) {
  const bool compute_error = (observation_error_ptr != NULL);
  double& observation_error = *observation_error_ptr;
  double num_observed_points = 0.0;
  if (compute_error) {
    observation_error = 0.0;
  }
  localization_options_.use_STF_constraints = use_point_constraints;
  localization_options_.log_poses = true;
  localization_options_.CorrespondenceCallback =
      ((debug_level_ > 0) ? CorrespondenceCallback : NULL);
  localization_->SetOptions(localization_options_);
  localization_->Initialize(Pose2Df(kStartingAngle, kStartingLocation),
                           kMapName);

  rosbag::Bag bag;
  if (!quiet_) printf("Processing bag file %s\n", bag_file.c_str());
  bag.open(bag_file.c_str(), rosbag::bagmode::Read);
  const double t_start = GetMonotonicTime();

  const std::vector<std::string> topics({
    CONFIG_scan_topic,
    CONFIG_odom_topic,
    CONFIG_initialpose_topic,
  });

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double bag_time_start = -1.0;
  double bag_time = 0.0;
  int num_laser_scans = 0;

  nav_msgs::Odometry last_standard_odometry;
  sensor_msgs::LaserScan last_laser_scan;
  Pose2Df last_laser_pose(0, 0, 0);
  bool standard_odometry_initialized = false;
  vector<Pose2Df> pose_trajectory;
  nonblock(true);
  for (rosbag::View::iterator it = view.begin();
       run_ && it != view.end(); ++it) {
    const rosbag::MessageInstance &message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }
    if (!quiet_ && debug_level_ > 1) {
      printf("message from %s\n", message.getTopic().c_str());
    }
    // Ignore messages before elapsed time_skip.
    if (bag_time < bag_time_start + time_skip) continue;
    if (!quiet_) {
      const double elapsed_time = bag_time - bag_time_start;
      const int hh = rint(elapsed_time / 3600.0);
      const int mm = rint(fmod(elapsed_time, 3600.0) / 60.0);
      const float ss = fmod(elapsed_time, 60.0);
      if (false) {
        printf("\r%02d:%02d:%04.1f (%.1f) Lost:%.3f",
              hh, mm, ss, elapsed_time, localization_->GetLostMetric());
      }
      printf("\r%02d:%02d:%04.1f (%.1f) Pose:%8.3f,%8.3f,%6.2f\u00b0",
             hh, mm, ss, elapsed_time,
             last_laser_pose.translation.x(),
             last_laser_pose.translation.y(),
             RadToDeg(last_laser_pose.angle));
    }
    fflush(stdout);
    int keycode = -1;
    if (run_ && kbhit() != 0) {
      keycode = fgetc(stdin);
      printf("\n"); fflush(stdout);
      switch(keycode){
        case ' ': {
          printf("Paused\n");
          keycode = 0;
          while (keycode !=' ' && run_) {
            Sleep(0.01);
            ros::spinOnce();
            if (kbhit()!=0) {
              keycode = fgetc(stdin);
            }
            if (keycode == 'r') {
              SensorResettingResample(last_laser_scan);
              keycode = 0;
            } else if (keycode == 'q') {
              printf("\nExiting.\n");
              exit(0);
            }
          }
        } break;
        case 'q': {
          printf("\nExiting.\n");
          exit(0);
        } break;
      }
    }

    // Check to see if this is a laser scan message.
    {
      sensor_msgs::LaserScanPtr laser_message =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_message != NULL &&
          message.getTopic() == CONFIG_scan_topic) {
        ++num_laser_scans;
        LaserCallback(*laser_message);
        while(localization_->RunningSolver()) {
          Sleep(0.01);
        }
        last_laser_scan = *laser_message;
        last_laser_pose = localization_->GetLatestPose();
        pose_trajectory.push_back(localization_->GetLatestPose());
        PublishLocation();
        continue;
      }
    }

    // Check to see if this is a standardized odometry message.
    {
      nav_msgs::OdometryPtr odometry_message =
          message.instantiate<nav_msgs::Odometry>();
      if (odometry_message != NULL &&
          message.getTopic() == CONFIG_odom_topic) {
        if (standard_odometry_initialized) {
          StandardOdometryCallback(last_standard_odometry, *odometry_message);
        } else {
          standard_odometry_initialized = true;
        }
        last_standard_odometry = *odometry_message;
      }
    }

    {
      Vector2f init_location;
      float init_angle;
      string init_map;
      if (LoadSetLocationMessage(message, &init_location, &init_angle,
          &init_map)) {
        if (debug_level_ > 0) {
          printf("Initializing location to %s: %f,%f, %f\u00b0\n",
                 init_map.c_str(), init_location.x(), init_location.y(),
                 DegToRad(init_angle));
        }
        const Pose2Df init_pose(init_angle, init_location.x(),
                                init_location.y());
        localization_->Initialize(init_pose, init_map);
      }
    }
  }
  localization_->Finalize();
  const double process_time = GetMonotonicTime() - t_start;
  const double bag_duration = bag_time - bag_time_start;
  const vector<Pose2Df> logged_poses = localization_->GetLoggedPoses();
  const vector<int> episode_lengths = localization_->GetLoggedEpisodeLengths();
  printf("Done in %.3fs, bag time %.3fs (%.3fx).\n",
        process_time, bag_duration, bag_duration / process_time);
  printf("%d laser scans, %lu logged poses\n",
        num_laser_scans, logged_poses.size());

  if (statistical_test_index_ >= 0) {
    const string file_name = StringPrintf(
        "results/enml/non_markov_test_%d.txt", statistical_test_index_);
    ScopedFile fid(file_name, "w");
    CHECK_NOTNULL(fid());
    for (unsigned int i = 0; i < pose_trajectory.size(); ++i) {
      fprintf(fid, "%f,%f,%f\n",
              pose_trajectory[i].translation.x(),
              pose_trajectory[i].translation.y(),
              pose_trajectory[i].angle);
    }
  }

  if (compute_error) {
    observation_error = observation_error / num_observed_points;
  }
}

void InitializeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  // if (debug_level_ > -1) {
  //   printf("Initialize %s %f,%f %f\u00b0\n",
  //       msg.map.c_str(), msg.pose.x, msg.pose.y, RadToDeg(msg.pose.theta));
  // }
  // localization_->Initialize(
  //     Pose2Df(msg.pose.theta, Vector2f(msg.pose.x, msg.pose.y)), msg.map);
  // localization_publisher_.publish(msg);
  //From this intiial_pose we have to set the 
  double yaw = atan2(2.0*(msg.pose.pose.orientation.y*msg.pose.pose.orientation.z + msg.pose.pose.orientation.w*msg.pose.pose.orientation.x), msg.pose.pose.orientation.w*msg.pose.pose.orientation.w - msg.pose.pose.orientation.x*msg.pose.pose.orientation.x - msg.pose.pose.orientation.y*msg.pose.pose.orientation.y + msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
  localization_->Initialize(
                        Pose2Df(yaw, Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y)), "ASE4");
  amrl_msgs::Localization2DMsg loc_msg_local;
  loc_msg_local.header = msg.header;
  loc_msg_local.pose.x = msg.pose.pose.position.x;
  loc_msg_local.pose.y = msg.pose.pose.position.y;
  loc_msg_local.pose.theta = yaw;
  localization_publisher_.publish(loc_msg_local);

//publish tf frames
  //static tf::TransformBroadcaster br;
  //init_transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y,0.0));
  //tf::Quaternion quat;
  //quat.setRPY(0,0,yaw);
  //init_transform.setRotation(quat);
  //br.sendTransform(tf::StampedTransform(init_transform, ros::Time::now(), "map_en", "odom"));
  //initialized_=true;
  //std::cout<<"sending map_en <--> odom"<<std::endl;


}




//void InitializeCallback(const amrl_msgs::Localization2DMsg& msg) {
//void InitializeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  //if (debug_level_ > -1) {
    //printf("Initialize %s %f,%f %f\u00b0\n",
        //msg.map.c_str(), msg.pose.x, msg.pose.y, RadToDeg(msg.pose.theta));
  //}
  //localization_->Initialize(
      //Pose2Df(msg.pose.theta, Vector2f(msg.pose.x, msg.pose.y)), msg.map);
  //localization_publisher_.publish(msg);

  //double yaw = atan2(2.0*(msg.pose.pose.orientation.y*msg.pose.pose.orientation.z + msg.pose.pose.orientation.w*msg.pose.pose.orientation.x), msg.pose.pose.orientation.w*msg.pose.pose.orientation.w - msg.pose.pose.orientation.x*msg.pose.pose.orientation.x - msg.pose.pose.orientation.y*msg.pose.pose.orientation.y + msg.pose.pose.orientation.z*msg.pose.pose.orientation.z);
  //localization_->Initialize(
                        //Pose2Df(yaw, Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y)), "AHG2");
  //amrl_msgs::Localization2DMsg loc_msg_local;
  //loc_msg_local.header = msg.header;
  //loc_msg_local.pose.x = msg.pose.pose.position.x;
  //loc_msg_local.pose.y = msg.pose.pose.position.y;
  //loc_msg_local.pose.theta = yaw;
  //localization_publisher_.publish(loc_msg_local);



  //From this intiial_pose we have to set the 
//}

void OnlineLocalize(bool use_point_constraints, ros::NodeHandle* node) {
  // Subscribe to laser scanner.
  localization_options_.use_STF_constraints = use_point_constraints;
  localization_options_.log_poses = true;
  localization_options_.CorrespondenceCallback =
      ((debug_level_ > 0) ? CorrespondenceCallback : NULL);
  localization_->SetOptions(localization_options_);
  localization_->Initialize(Pose2Df(kStartingAngle, kStartingLocation),
                           kMapName);

  Subscriber laser_subscriber =
      node->subscribe(CONFIG_scan_topic, 1, LaserCallback);
  Subscriber odom_subscriber =
      node->subscribe(CONFIG_odom_topic, 1, OdometryCallback);
  Subscriber initialize_subscriber =
      node->subscribe("/initialpose_a1_121", 1, InitializeCallback); // /set_pose

  ClearDisplay();
  PublishDisplay();

  while (run_ && ros::ok()) {
    Sleep(0.02);
    // TODO: Handle dynamic reloading of config.
    ros::spinOnce();
  }
}

void PrintBackTrace(FILE* file) {
  void *trace[16];
  char **messages = (char **)NULL;
  int i, trace_size = 0;

  trace_size = backtrace(trace, 16);
  messages = backtrace_symbols(trace, trace_size);
  fprintf(file, "Stack Trace:\n");
  for (i=1; i<trace_size; ++i) {
    fprintf(file, "#%d %s\n", i - 1, messages[i]);
    std::string addr2line_command =
        StringPrintf("addr2line %p -e /proc/%d/exe", trace[i], getpid());
    fprintf(file, "%s", ExecuteCommand(addr2line_command.c_str()).c_str());
  }
}

void SegfaultHandler(int t) {
  printf("Segmentation Fault\n");
  PrintBackTrace(stderr);
  exit(-1);
}

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void InitializeMessages() {
  ros_helpers::InitRosHeader("map", &localization_msg_.header);
  ros_helpers::InitRosHeader("map", &pose_msg_.header);
}

int main(int argc, char** argv) {
  //InitHandleStop(&run_, 0);
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  // gflags::ParseCommandLineFlags(&argc, &argv, true);

  // CHECK(signal(SIGSEGV, &SegfaultHandler) != SIG_ERR);

  char* bag_file = NULL;
  char* keyframes_file = NULL;
  int max_laser_poses = -1;
  bool disable_stfs = false;
  double time_skip = 0;
  bool unique_node_name = false;
  bool return_initial_poses = false;
  char* robot_config_opt = NULL;



  static struct poptOption options[] = {
    { "config_dir", 'c', POPT_ARG_STRING, &config_dir_, 1, "Config directory", "STRING"},
    { "robot_config", 'r', POPT_ARG_STRING, &robot_config_, 1, "Robot configuration file", "STRING"},
    { "maps_dir", 'm', POPT_ARG_STRING, &maps_dir_, 1, "Maps directory", "STRING"},
    { "debug" , 'd', POPT_ARG_INT, &debug_level_, 1, "Debug level", "NUM" },
    { "bag-file", 'b', POPT_ARG_STRING, &bag_file, 1, "ROS bagfile to use",
        "STRING"},
    { "max-poses" , 'n', POPT_ARG_INT, &max_laser_poses, 1,
        "Maximum number of laser poses to optimize", "NUM" },
    { "time-skip", 's', POPT_ARG_DOUBLE, &time_skip, 1,
      "Time to skip from the bag file", "NUM" },
    { "test-set", 't', POPT_ARG_INT, &test_set_index_, 1,
        "Test set index", "NUM" },
    { "statistical-test", 'T', POPT_ARG_INT, &statistical_test_index_, 1,
        "Statistical test index", "NUM" },
    { "noise", 'N', POPT_ARG_DOUBLE, &odometry_additive_noise_, 1,
        "Statistical test additive random noise", "NUM" },
    { "keyframes", 'k', POPT_ARG_STRING, &keyframes_file, 1,
        "Keyframes file", "STRING" },
    { "episode-images", 'e', POPT_ARG_STRING, &episode_images_path, 1,
        "Epidode images path", "STRING" },
    { "unique_node_name", 'u', POPT_ARG_NONE, &unique_node_name, 0,
        "Use unique ROS node name", "NONE" },
    { "initial_poses", 'i', POPT_ARG_NONE, &return_initial_poses, 0,
        "Return intial, instead of final, pose estimates", "NONE" },
    { "disable-stfs", 'p', POPT_ARG_NONE, &disable_stfs, 0,
        "Disable STFs", "NONE"},
    { "save-ltfs", 'l', POPT_ARG_NONE, &save_ltfs_, 0,
        "Save LTFs", "NONE"},
    { "quiet", 'q', POPT_ARG_NONE, &quiet_, 0,
        "Quiet", "NONE"},
    { "robot_config", 'r', POPT_ARG_STRING, &robot_config_opt, 0,
        "Robot config file", "STRING"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  localization_ = new NonMarkovLocalization(maps_dir_);
  CHECK(LoadConfiguration(&localization_options_));

  // if (bag_file == NULL) unique_node_name = true;
  const bool running_tests =
      (test_set_index_ >= 0 || statistical_test_index_ >= 0);
  if (running_tests) {
    const unsigned long seed = time(NULL);
    rand_ = util_random::Random(seed);
    printf("Seeding with %lu test=%.5f\n", seed, rand_.Gaussian(0, 1));
  }
  const string node_name =
      (running_tests || unique_node_name) ?
      StringPrintf("NonMarkovLocalization_%lu",
                   static_cast<uint64_t>(GetWallTime() * 1000000.0)) :
      string("NonMarkovLocalization");
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  ros::NodeHandle ros_node;
  InitializeMessages();
  localization_publisher_ =
      ros_node.advertise<amrl_msgs::Localization2DMsg>(
      "localization", 1, true);
  {
    visualization_publisher_ =
        ros_node.advertise<amrl_msgs::VisualizationMsg>(
        "visualization", 1, true);
    visualization_msg_ = visualization::NewVisualizationMessage("map", "enml");
  }
  pose_publisher_ =
      ros_node.advertise<geometry_msgs::PoseStamped>(
      "global_pose_a1_121", 1, true);

 pose_publisher_2 =
      ros_node.advertise<geometry_msgs::PoseStamped>(
      "global_pose_a1_121_en", 1, true);


  pose_msg_.pose.orientation.w=1.0;
  pose_msg_en.pose.orientation.w=1.0;

  tf2_listener= new tf2_ros::TransformListener(tf_buffer);

  if (bag_file != NULL) {
    PlayBagFile(
        bag_file, max_laser_poses, !disable_stfs, time_skip, &ros_node, NULL);
  } else {
    OnlineLocalize(!disable_stfs, &ros_node);
  }

  delete localization_;
  return 0;
}
