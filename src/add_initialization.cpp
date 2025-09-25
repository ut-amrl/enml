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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shared/math/math_util.h"

using std::string;
using std::vector;

DEFINE_double(x, 0, "Initial pose x coordinate");
DEFINE_double(y, 0, "Initial pose y coordinate");
DEFINE_double(theta, 0, "Initial pose angle");
DEFINE_string(in, "", "Input bag file");
DEFINE_string(out, "", "Output bag file");
DEFINE_string(topic, "/initialpose", "Topic name for initialization message");
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

    // Setup ROS2 rosbag2 reader
    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_cpp::StorageOptions read_options;
    read_options.uri = in_file;
    read_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(read_options, converter_options);

    // Setup ROS2 rosbag2 writer
    rosbag2_cpp::writers::SequentialWriter writer;
    rosbag2_cpp::StorageOptions write_options;
    write_options.uri = out_file;
    write_options.storage_id = "sqlite3";

    writer.open(write_options, converter_options);

    bool written_init = false;
    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        rclcpp::Time message_time(serialized_message->time_stamp);

        if (!written_init) {
            const auto init_msg = InitMsg(x, y, theta, message_time);

            // Serialize and write the initialization message
            rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serialization;
            rclcpp::SerializedMessage serialized_init_msg;
            serialization.serialize_message(&init_msg, &serialized_init_msg);

            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = FLAGS_topic;
            bag_message->time_stamp = serialized_message->time_stamp;
            bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t(serialized_init_msg.release_rcl_serialized_message()),
                [](rcutils_uint8_array_t* data) {
                    auto ret = rcutils_uint8_array_fini(data);
                    (void)ret;
                    delete data;
                });

            writer.write(bag_message);
            written_init = true;
            printf("Written init.\n");
        }

        // Write the original message
        writer.write(serialized_message);
    }
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    gflags::SetUsageMessage(
        "./bin/add_initialization --in INBAG --out OUTBAG "
        "--x X --y Y --theta THETA [--topic TOPIC]");
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    if (FLAGS_in.empty() || FLAGS_out.empty()) {
        gflags::ShowUsageWithFlagsRestrict(argv[0], "initialization");
        return 1;
    }
    ProcessBagFile(FLAGS_in, FLAGS_out, FLAGS_x, FLAGS_y, FLAGS_theta);
    return 0;
}
