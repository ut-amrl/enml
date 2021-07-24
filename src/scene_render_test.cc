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
// Copyright 2021 joydeepb@cs.utexas.edu
// The University of Texas at Austin
//
// Debug visualization of analytic scene rendering.

#include <signal.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"
#include "visualization/visualization.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"

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
using geometry::Line2f;
using ros::Publisher;
using ros::Subscriber;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using vector_map::TrimOcclusion;
using vector_map::VectorMap;

string maps_dir_ = ros::package::getPath("amrl_maps");
amrl_msgs::VisualizationMsg viz_msg_;
Publisher viz_pub_;
VectorMap map_;
bool run_ = true;

DEFINE_string(map, "AHG2", "Name of map");
DEFINE_string(maps_dir, "", "Maps directory");
DECLARE_double(min_line_length);
DEFINE_double(max_range, 30, "Max range");
DEFINE_double(x, 5, "X coordinate for test");
DEFINE_double(y, 71, "Y coordinate for test");

string GetMapFileFromName(const string& map) {
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}


void InitMessages() {
  viz_msg_.header.frame_id = "map";
  viz_msg_.header.seq = 0;
  viz_msg_.ns = "scene_render_test";
}

void PublishAndClear(float delay) {
  const int kNumRepeat = 10;
  viz_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < kNumRepeat; ++i) {
    viz_pub_.publish(viz_msg_);
    ros::spinOnce();
    Sleep(delay / double(kNumRepeat));
  }
  visualization::ClearVisualizationMsg(viz_msg_);
}

void DrawLines(const vector<Line2f>& lines, uint32_t color) {
  for (auto l : lines) {
    visualization::DrawLine(l.p0, l.p1, color, viz_msg_);
  }
}

void DrawScene(const Vector2f& loc, const vector<Line2f>& scene) {
  for (auto l : scene) {
    visualization::DrawLine(l.p0, l.p1, 0xffff0000, viz_msg_);
    visualization::DrawLine(loc, l.p0, 0xffc0c0c0, viz_msg_);
    visualization::DrawLine(loc, l.p1, 0xffc0c0c0, viz_msg_);
  }
}

void DrawLineExtent(const Vector2f& loc, const Line2f& l) {
  visualization::DrawLine(l.p0, l.p1, 0xffff0000, viz_msg_);
  visualization::DrawLine(loc, l.p0, 0xffc0c0c0, viz_msg_);
  visualization::DrawLine(loc, l.p1, 0xffc0c0c0, viz_msg_);
}

void AnimateSceneRender() {
  const Vector2f loc(FLAGS_x, FLAGS_y);
  VectorMap map(GetMapFileFromName(FLAGS_map));
  static const unsigned int MaxLines = 2000;
  const float eps = Sq(FLAGS_min_line_length);
  vector<Line2f> scene;
  vector<Line2f> lines_list;
  map.GetSceneLines(loc, FLAGS_max_range, &lines_list);
  vector<Line2f> render;

  visualization::DrawCross(loc, 0.5, 0xffff0000, viz_msg_);
  DrawLines(lines_list, 0xff0000ff);
  PublishAndClear(1.0);

  const Vector2f test_loc(0.417, 55);

  for(size_t i = 0; i < lines_list.size() && i < MaxLines; ++i) {
    printf("\r%5d / %5d", int(i), int(lines_list.size()));
    fflush(stdout);
    Line2f cur_line = lines_list[i];
    // Check if any part of cur_line is unoccluded by present list of lines,
    // as seen from loc.
    visualization::DrawCross(loc, 0.5, 0xffff0000, viz_msg_);
    DrawLines(scene, 0xff00e000);
    DrawLines(scene, 0xff00e000);
    DrawLineExtent(loc, cur_line);
    const bool trace = (cur_line.Distance(test_loc) < 0.01);
    size_t trace_len = 0;
    if (trace) {
      printf("\n\n %f %f %f %f\n", cur_line.p0.x(), cur_line.p0.y(), cur_line.p1.x(), cur_line.p1.y());
    }
    for(size_t j = 0; j < scene.size() && cur_line.SqLength() >= eps; ++j) {
      if (scene[j].SqLength() < eps) continue;
      if (trace) trace_len = lines_list.size();
      TrimOcclusion(loc, scene[j], &cur_line, &lines_list, eps);
      if (trace) {
        if (trace_len != lines_list.size()) {
        printf("Broken1 %f %f %f %f\n", cur_line.p0.x(), cur_line.p0.y(), cur_line.p1.x(), cur_line.p1.y());
        printf("Broken2 %f %f %f %f\n", lines_list.back().p0.x(), lines_list.back().p0.y(), lines_list.back().p1.x(), lines_list.back().p1.y());
        } else if (cur_line.SqLength() < eps) {
          printf("Line deleted by %f %f %f %f\n", scene[j].p0.x(), scene[j].p0.y(), scene[j].p1.x(), scene[j].p1.y());
          printf("After: %f %f %f %f\n", cur_line.p0.x(), cur_line.p0.y(), cur_line.p1.x(), cur_line.p1.y());
        }
      }
    }
    if (trace && cur_line.SqLength() < eps) printf("Line Deleted\n");

    if (cur_line.SqLength() > eps) { //At least part of cur_line is unoccluded
      for(size_t j = 0; j < scene.size(); ++j) {
        if (scene[j].SqLength() < eps) continue;
        TrimOcclusion(loc, cur_line, &scene[j], &lines_list, eps);
      }
      if (trace) {
      printf("Added %f %f %f %f\n", cur_line.p0.x(), cur_line.p0.y(), cur_line.p1.x(), cur_line.p1.y());
    }
      // Add the visible part of cur_line.
      scene.push_back(cur_line);
    }
    PublishAndClear(0.01);
  }
  for(const Line2f& l : scene) {
    if (l.SqLength() > eps && 
        geometry::DistanceFromLineSegment(loc, l.p0, l.p1) < FLAGS_max_range) {
      render.push_back(l);
    }
  }
  if (lines_list.size() >= MaxLines) {
    fprintf(stderr,
            "Runaway Analytic Scene Render at %.30f,%.30f\n",
            loc.x(), loc.y());
  }
  visualization::DrawCross(loc, 0.5, 0xffff0000, viz_msg_);
  DrawScene(loc, render);
  PublishAndClear(1.0);
  printf("\nScene lines: %d\n", int(render.size()));
}

void SigHandler(int) {
  run_ = false;
  printf("\nExiting...\n");
  // exit(0);
}

void TestTrimOcclusion() {
  const Vector2f loc(FLAGS_x, FLAGS_y);
  Line2f l1(0.416146, 56.394455, 0.416146, 38.016346);
  Line2f l2(0.304153, 56.037613, 0.304167, 55.743950);
  vector<Line2f> lines;
  TrimOcclusion(loc, l2, &l1, &lines);
}

void SetPoseCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f loc(msg.pose.x, msg.pose.y);
  // VectorMap map(GetMapFileFromName(msg.map));
  vector<Line2f> scene;
  map_.SceneRender(loc, FLAGS_max_range, -M_PI, M_PI, &scene);
  visualization::DrawCross(loc, 0.5, 0xffff0000, viz_msg_);
  DrawScene(loc, scene);
  PublishAndClear(1.0);
  printf("\nScene lines: %d\n", int(scene.size()));
}

void TestSceneRender(ros::NodeHandle& nh) {
  Subscriber sub = nh.subscribe("set_pose", 1, SetPoseCallback);
  while (ros::ok()) {
    Sleep(0.05);
    ros::spinOnce();
  }
}

vector<int> map_visibility_;

void TrackVisibilityLocalizationCallback(
    const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f loc(msg.pose.x, msg.pose.y);
  // VectorMap map(GetMapFileFromName(msg.map));
  map_visibility_.resize(map_.lines.size(), 0);
  vector<Line2f> scene;
  map_.SceneRender(loc, FLAGS_max_range, -M_PI, M_PI, &scene);
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const Line2f& l = map_.lines[i];
    for (const Line2f& k : scene) {
      if (vector_map::Overlaps(k, l)) {
        ++map_visibility_[i];
        break;
      }
    }
  }
}

void PrintSpinner() {
  const vector<char> ch = {'|', '/', '-', '\\' };
  const int kNum = ch.size();
  static int i = 0;
  printf("\r%c ", ch[i]);
  fflush(stdout);
  i = (i + 1) % kNum;
}

void TrackVisibility(ros::NodeHandle& nh) {
  Subscriber sub = nh.subscribe("localization", 1, 
      TrackVisibilityLocalizationCallback);
  while (run_) {
    Sleep(0.05);
    ros::spinOnce();
    PrintSpinner();
  }
  int num_visible_lines = 0;
  vector<Line2f> visible_lines;
  for (size_t i = 0; i < map_visibility_.size(); ++i) {
    if (map_visibility_[i] > 0) {
      ++num_visible_lines;
      visible_lines.push_back(map_.lines[i]);
    }
  }
  map_.lines = visible_lines;
  map_.Save(map_.file_name + ".pruned");
  printf("Visible: %d / %d\n", num_visible_lines, int(map_visibility_.size()));
  DrawLines(visible_lines, 0xffff0000);
  PublishAndClear(2.0);
}

DEFINE_bool(trim_occlusion, false, "Test TrimOcclusion");
DEFINE_bool(track_visibility, false, "Track visibility of all map lines");

int main(int num_args, char** args) {
  google::InitGoogleLogging(args[0]);
  google::ParseCommandLineFlags(&num_args, &args, false);

  ros::init(num_args, args, "scene_render_test");
  ros::NodeHandle ros_node;
  signal(SIGINT, SigHandler);
  if (maps_dir_.empty()) {
    if (FLAGS_maps_dir.empty()) {
      fprintf(stderr, "ERROR: Unable to locate amrl_maps, please specify using the --maps_dir flag\n");
      return(1);
    }
    maps_dir_ = FLAGS_maps_dir;
  }
  map_.Load(GetMapFileFromName(FLAGS_map));

  viz_pub_ = ros_node.advertise<amrl_msgs::VisualizationMsg>(
      "visualization", 1, true);
  InitMessages();

  if (FLAGS_trim_occlusion) {
    TestTrimOcclusion();
    return 0;
  } else if (FLAGS_track_visibility) {
    TrackVisibility(ros_node);
    return 0;
  }
  
  // AnimateSceneRender();
  TestSceneRender(ros_node);
  return 0;
}