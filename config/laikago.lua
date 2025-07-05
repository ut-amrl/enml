RobotConfig = {
  name = "laikago_gazebo";
  scan_topic = "/scan";
  odometry_topic = "/odom";
  initialpose_topic = "/initialpose";
  pointcloud_topic = "";
  
  -- ROS Publisher Topics
  visualization_topic = DefaultRosConfig.visualization_topic,
  localization_topic = DefaultRosConfig.localization_topic,
  localization_ros_topic = DefaultRosConfig.localization_ros_topic,
  
  -- ROS Subscriber Topics  
  set_pose_topic = DefaultRosConfig.set_pose_topic,
  
  -- ROS Node and Frame Names
  node_name = DefaultRosConfig.node_name,
  map_frame = DefaultRosConfig.map_frame,
  visualization_frame = DefaultRosConfig.visualization_frame,
  
  -- Package and File Configuration
  maps_package = DefaultRosConfig.maps_package,
  map_file_extension = DefaultRosConfig.map_file_extension,
};

