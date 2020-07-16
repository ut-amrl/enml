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
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "shared/math/math_util.h"

using std::string;
using std::vector;

DEFINE_double(x, 0, "Initial pose x coordinate");
DEFINE_double(y, 0, "Initial pose y coordinate");
DEFINE_double(theta, 0, "Initial pose angle");
DEFINE_string(in, "", "Input bag file");
DEFINE_string(out, "", "Output bag file");
DECLARE_string(helpmatch);

geometry_msgs::PoseWithCovarianceStamped InitMsg(
    float x, float y, float theta, const ros::Time& time) {
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.seq = 0;
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
  rosbag::Bag in_bag;
  rosbag::Bag out_bag;
  printf("Input: %s\nOutput: %s\n", in_file.c_str(), out_file.c_str());
  in_bag.open(in_file.c_str(), rosbag::bagmode::Read);
  out_bag.open(out_file.c_str(), rosbag::bagmode::Write);

  bool written_init = false;
  for(rosbag::MessageInstance const& m : rosbag::View(in_bag)) {
    if (!written_init) {
      const auto init_msg = InitMsg(x, y, theta, m.getTime());
      out_bag.write("/initialpose", m.getTime(), init_msg);
      written_init = true;
      printf("Written init.\n");
    }
    // printf("Msg: %f\n", m.getTime().toSec());
    out_bag.write(m.getTopic(), m.getTime(), m);
  }
}

int main(int argc, char* argv[]) {
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
  return 0;
}
