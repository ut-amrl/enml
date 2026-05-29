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
// Copyright 2020 joydeepb@cs.utexas.edu
// Computer Science Department, University of Texas at Austin
//
// Helper tool to add initialization message to ROS bag files.

#include <cmath>
#include <unordered_map>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "shared/math/math_util.h"

using std::string;
using std::unordered_map;
using std::vector;

DEFINE_double(x, 0, "Initial pose x coordinate");
DEFINE_double(y, 0, "Initial pose y coordinate");
DEFINE_double(theta, 0, "Initial pose angle");
DEFINE_string(in, "", "Input bag file");
DEFINE_string(out, "", "Output bag file");
DECLARE_string(helpmatch);

geometry_msgs::msg::PoseWithCovarianceStamped InitMsg(
    float x, float y, float theta, const rclcpp::Time& time) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = time;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.w = std::cos(0.5 * theta);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(0.5 * theta);
  msg.pose.covariance = {
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0.25, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, math_util::DegToRad(4.0)};
  return msg;
}

void ProcessBagFile(const string& in_file,
                    const string& out_file,
                    float x,
                    float y,
                    float theta) {
  printf("Input: %s\nOutput: %s\n", in_file.c_str(), out_file.c_str());
  rosbag2_cpp::Reader reader;
  rosbag2_cpp::Writer writer;
  reader.open(in_file);
  writer.open(out_file);

  unordered_map<string, string> topic_types;
  for (const auto& topic_metadata : reader.get_all_topics_and_types()) {
    topic_types[topic_metadata.name] = topic_metadata.type;
  }

  bool written_init = false;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (!written_init) {
      const rclcpp::Time stamp(message->recv_timestamp);
      const auto init_msg = InitMsg(x, y, theta, stamp);
      writer.write(init_msg, "/initialpose", stamp);
      written_init = true;
      printf("Written init.\n");
    }
    const auto topic_type = topic_types.find(message->topic_name);
    CHECK(topic_type != topic_types.end()) << message->topic_name;
    writer.write(message, message->topic_name, topic_type->second);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  gflags::SetUsageMessage(
      "./bin/add_initialization --in INBAG --out OUTBAG "
      "--x X --y Y --theta THETA");
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  if (FLAGS_in.empty() || FLAGS_out.empty()) {
    gflags::ShowUsageWithFlagsRestrict(argv[0], "initialization");
    return 1;
  }
  ProcessBagFile(FLAGS_in, FLAGS_out, FLAGS_x, FLAGS_y, FLAGS_theta);
  rclcpp::shutdown();
  return 0;
}
