RobotConfig = {
  name = "ut_automata";
  scan_topic = "/scan";
  pointcloud_topic = "";
  odometry_topic = "/odom";
  initialpose_topic = "/initialpose";
};

enml = {
  map_name = "UT_AUTOmata_Track";
  starting_loc_x = 1.0;
  starting_loc_y = 3.0;
  starting_angle = deg2rad(0.0);

  -- Hokuyo UST-10lx Sensor parameters.
  laser_std_dev = 0.05;
  min_point_cloud_range = 0.02;
  max_point_cloud_range = 9.9;
  max_normal_point_distance = 0.05;
  robot_sensor_offset = vec3(0.22, 0.0, 0.15);
  num_skip_readings = 400; --4 this is only used for STF
  num_skip_readings_ltf = 2;

  -- -- Odometry parameters.
  min_rotation = deg2rad(5);
  min_translation = 0.05; --0.1 0.01
  max_odometry_delta_loc = 0.2;
  max_odometry_delta_angle = deg2rad(15.0);
  odometry_rotation_scale = 1.0;
  odometry_translation_scale = 1.0;

  -- Parameters for Odometry constraints.
  max_update_period = 0.5;
  odometry_radial_stddev_rate = 0.1;
  odometry_tangential_stddev_rate = 0.1;
  odometry_angular_stddev_rate = 0.5; --0.5
  odometry_translation_min_stddev = 0.001;
  odometry_translation_max_stddev = 10.5;
  odometry_rotation_min_stddev = deg2rad(10.0);
  odometry_rotation_max_stddev = deg2rad(1500.0);

  -- Parameters for LTF constraints.
  map_huber_loss = 0.1;
  max_point_to_line_distance = 0.15;
  max_angle_error = deg2rad(35.0);
  map_correlation_factor = 1.0 / 5.0;

  -- Parameters for STF constraints.
  point_match_threshold = 0.0;
  max_stf_angle_error = deg2rad(0.0);
  max_correspondences_per_point = 0;
  point_correlation_factor = 1.0 / 10.0;

  -- Parameters for Object constraints.
  use_object_constraints = false;
  object_correlation_factor = 1.0 / 10.0;
  object_match_threshold = 0.2;
  min_object_overlap = 300;

  -- Parameters for visibility constraints.
  use_visibility_constraints = true;
  visibility_correlation_factor = 0.02;

  -- Parameters for episode segmentation.
  min_ltf_ratio = 0.9;
  min_episode_length = 10;

  -- MLE Optimization parameters.
  pose_increment = 1;
  max_history = 3; -- 3
  max_solver_iterations = 15; -- 15
  num_repeat_iterations = 1;
  max_repeat_iterations = 2;
  return_jacobian = false;
  num_threads = 6;
  limit_history = true;
};
