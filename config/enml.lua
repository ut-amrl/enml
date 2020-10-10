-- enml_domain = "orebro";
-- enml_domain = "freiburg";
enml_domain = "cobot";

enml = {
  map_name = "EmptyMap";
  starting_loc_x = 0;
  starting_loc_y = 0;
  starting_angle = deg2rad(180.0);

  -- CoBot Sensor parameters.
  laser_std_dev = 0.05;
  min_point_cloud_range = 0.02;
  max_point_cloud_range = 7.9;
  max_normal_point_distance = 0.1;
  robot_sensor_offset = vec3(0.15, 0.0, 0.25);
  num_skip_readings = 1;

  -- Odometry parameters.
  -- CoBot parameters.
  min_rotation = deg2rad(5);
  min_translation = 0.3;
  max_odometry_delta_loc = 0.2;
  max_odometry_delta_angle = deg2rad(15.0);
  odometry_rotation_scale = 1.0;
  odometry_translation_scale = 1.0;

  -- Parameters for Odometry constraints.
  max_update_period = 0.5;
  odometry_radial_stddev_rate = 0.1;
  odometry_tangential_stddev_rate = 0.1;
  odometry_angular_stddev_rate = 0.1;
  odometry_translation_min_stddev = 0.001;
  odometry_translation_max_stddev = 10.5;
  odometry_rotation_min_stddev = deg2rad(1.0);
  odometry_rotation_max_stddev = deg2rad(1500.0);

  -- Parameters for LTF constraints.
  map_huber_loss = 0.1;
  max_point_to_line_distance = 0.25;
  max_angle_error = deg2rad(35.0);
  map_correlation_factor = 1.0 / 50.0;

  -- Parameters for STF constraints.
  point_match_threshold = 0.15;
  max_stf_angle_error = deg2rad(25.0);
  max_correspondences_per_point = 6;
  point_correlation_factor = 1.0 / 40.0;

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
  max_history = 80;
  max_solver_iterations = 30;
  num_repeat_iterations = 1;
  max_repeat_iterations = 4;
  return_jacobian = false;
  num_threads = 8;
  limit_history = false;
};


if RobotConfig.name=="laikago_gazebo"then
  -- ********************************************************
  -- DEFAULT STARTING LOCATION
  -- Doorway of AHG Apartment closest to drone cage facing the drone cage
  enml.map_name = "AHG_Apartment";
  enml.starting_loc_x = 0.0;
  enml.starting_loc_y = 0.0;
  enml.starting_angle = 3.4;

    -- -- Odometry parameters.
  enml.min_rotation = deg2rad(5.0);
  enml.min_translation = 0.25;

  -- VLP16 Sensor parameters.
  enml.laser_std_dev = 0.01;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 100;
  enml.max_normal_point_distance = 0.75;
  enml.robot_sensor_offset = vec3(0.05, 0.0, 0.5);
  enml.num_skip_readings = 1;

  -- Parameters for Odometry constraints.
  enml.max_update_period = 0.5;
  enml.odometry_radial_stddev_rate = 0.1;
  enml.odometry_tangential_stddev_rate = 0.1;
  enml.odometry_angular_stddev_rate = 0.5;
  enml.odometry_translation_min_stddev = 0.001;
  enml.odometry_translation_max_stddev = 10.5;
  enml.odometry_rotation_min_stddev = deg2rad(10.0);
  enml.odometry_rotation_max_stddev = deg2rad(1500.0);

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.25;
  enml.max_stf_angle_error = deg2rad(35.0);
  enml.max_correspondences_per_point = 10;
  enml.point_correlation_factor = 1.0 / 5.0;

  -- Parameters for LTF constraints.
  enml.map_huber_loss = 0.25;
  enml.max_point_to_line_distance = 0.7;
  enml.max_angle_error = deg2rad(35.0);
  enml.map_correlation_factor = 1.0 / 5.0;

  -- Parameters for visibility constraints.
  enml.use_visibility_constraints = false;
  enml.visibility_correlation_factor = 0.02;

  -- MLE Optimization parameters.
  enml.pose_increment = 1;
  enml.max_history = 10;
  enml.max_solver_iterations = 15;
  enml.num_repeat_iterations = 1;
  enml.max_repeat_iterations = 3;
  enml.return_jacobian = false;
  enml.num_threads = 24;
  enml.limit_history = true;
end

if RobotConfig.name=="a1_gazebo" or RobotConfig.name=="a1" then
  -- ********************************************************
  -- DEFAULT STARTING LOCATION
  -- Doorway of AHG Apartment closest to drone cage facing the drone cage
  enml.map_name = "AHG_Apartment";
  enml.starting_loc_x = 0.0;
  enml.starting_loc_y = 0.0;
  enml.starting_angle = 3.4;

    -- -- Odometry parameters.
  enml.min_rotation = deg2rad(5.0);
  enml.min_translation = 0.25;

  -- VLP16 Sensor parameters.
  enml.laser_std_dev = 0.01;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 10;
  enml.max_normal_point_distance = 0.75;
  enml.robot_sensor_offset = vec3(0.05, 0.0, 0.5);
  enml.num_skip_readings = 1;

  -- Parameters for Odometry constraints.
  enml.max_update_period = 0.5;
  enml.odometry_radial_stddev_rate = 0.1;
  enml.odometry_tangential_stddev_rate = 0.1;
  enml.odometry_angular_stddev_rate = 2.;
  enml.odometry_translation_min_stddev = 0.001;
  enml.odometry_translation_max_stddev = 10.5;
  enml.odometry_rotation_min_stddev = deg2rad(25.0);
  enml.odometry_rotation_max_stddev = deg2rad(1500.0);

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.25;
  enml.max_stf_angle_error = deg2rad(35.0);
  enml.max_correspondences_per_point = 10;
  enml.point_correlation_factor = 1.0 / 5.0;

  -- Parameters for LTF constraints.
  enml.map_huber_loss = 0.25;
  enml.max_point_to_line_distance = 0.7;
  enml.max_angle_error = deg2rad(50.0);
  enml.map_correlation_factor = 1.0 / 5.0;

  -- Parameters for visibility constraints.
  enml.use_visibility_constraints = false;
  enml.visibility_correlation_factor = 0.02;

  -- MLE Optimization parameters.
  enml.pose_increment = 1;
  enml.max_history = 5;
  enml.max_solver_iterations = 20;
  enml.num_repeat_iterations = 1;
  enml.max_repeat_iterations = 3;
  enml.return_jacobian = false;
  enml.num_threads = 8;
  enml.limit_history = true;
end

if RobotConfig.name=="ut-jackal" or RobotConfig.name=="smads-ut-jackal" then
  -- On the cobblestone next to the GDC door closest to the lab,
  -- ********************************************************
  -- DEFAULT STARTING LOCATION
  -- On the cobblestone next to the GDC door closest to the lab,
  -- right next to grass, on the side with the lamp post and facing Speedway.
  enml.map_name = "UT_Campus";
  enml.starting_loc_x = 114.067214966;
  enml.starting_loc_y = -251.863693237;
  enml.starting_angle = deg2rad(-175);

  -- VLP16 Sensor parameters.
  enml.laser_std_dev = 0.01;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 100;
  enml.max_normal_point_distance = 0.75;
  enml.robot_sensor_offset = vec3(0.05, 0.0, 0.5);
  enml.num_skip_readings = 1;

  -- MLE Optimization parameters.
  enml.pose_increment = 1;
  enml.max_history = 10;
  enml.max_solver_iterations = 15;
  enml.num_repeat_iterations = 1;
  enml.max_repeat_iterations = 3;
  enml.return_jacobian = false;
  enml.num_threads = 24;
  enml.limit_history = true;
end
  -- ********************************************************

  -- (1) ut-tower-pond_2020-08-07-18-43-49.bag
  -- (2) ut-tower-pond-2_2020-08-07-19-05-01.bag
  -- enml.map_name = "UT_Campus"
  -- enml.starting_loc_x = -116.762;
  -- enml.starting_loc_y = -115.876;
  -- enml.starting_angle = deg2rad(-147)

  -- ut-ahg-tower_2020-07-24-12-49-41.bag
  -- enml.map_name = "UT_Campus"
  -- enml.starting_loc_x = -3;
  -- enml.starting_loc_y = -30;
  -- enml.starting_angle = deg2rad(-122)

  -- 2020-06-03-18-51-39.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 131;
  -- enml.starting_loc_y = -245;
  -- enml.starting_angle = deg2rad(-85);

  -- 2020-06-25-17-34-30.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 4;
  -- enml.starting_loc_y = -29;
  -- enml.starting_angle = deg2rad(-122);

  -- 2020-06-25-17-34-30.bag 160 seconds in
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 67.538;
  -- enml.starting_loc_y = -96.67;
  -- enml.starting_angle = deg2rad(-89);

  -- 2020-06-25-17-34-30.bag 341.3 seconds in
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = -131.66;
  -- enml.starting_loc_y = -133.88;
  -- enml.starting_angle = deg2rad(-172);

  -- 2020-06-25-17-49-13.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = -117.66;
  -- enml.starting_loc_y = -340.72;
  -- enml.starting_angle = deg2rad(-82);

  -- 2020-06-18-17-54-48.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 132;
  -- enml.starting_loc_y = -247.5;
  -- enml.starting_angle = deg2rad(95);

  -- 2020-06-03-18-51-39.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 131;
  -- enml.starting_loc_y = -245;
  -- enml.starting_angle = deg2rad(-85);

  -- 2020-06-03-18-56-16.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = -24.693;
  -- enml.starting_loc_y = -27;
  -- enml.starting_angle = deg2rad(56.41);

  -- 2020-06-03-15-37-53.bag
  -- enml.map_name = "UT_Campus";
  -- enml.starting_loc_x = 0;
  -- enml.starting_loc_y = 0;
  -- enml.starting_angle = deg2rad(0);

  -- 2020-03-12-16-21-47-GDC-Plaza-Loop-4.bag
  -- enml.map_name = "UT_Campus";
  -- enml.map_name = "EmptyMap";
  -- enml.starting_loc_x = 168;
  -- enml.starting_loc_y = -213;
  -- enml.starting_angle = deg2rad(-86);
  -- Skip 222.5 into bag:
  -- enml.starting_loc_x = 157.54;
  -- enml.starting_loc_y = -155.64;
  -- enml.starting_angle = deg2rad(-86.5);

  -- enml.map_name = "EmptyMap";
  -- enml.starting_loc_x = 0;
  -- enml.starting_loc_y = 0;
  -- enml.starting_angle = deg2rad(0);



if RobotConfig.name=="ut-automata" then
  enml.map_name = "Joydeepb-Home";
  -- -- enml.map_name = "EmptyMap";
  -- enml.starting_loc_x = 0;
  -- enml.starting_loc_y = 0;
  -- enml.starting_angle = deg2rad(-33.0);
  -- enml.map_name = "GDC3";
  enml.starting_loc_x = 14.8;
  enml.starting_loc_y = 14.4;
  enml.starting_angle = deg2rad(180.0);

  -- Hokuyo UST-10lx Sensor parameters.
  enml.laser_std_dev = 0.05;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 9.9;
  enml.max_normal_point_distance = 0.05;
  enml.robot_sensor_offset = vec3(0.22, 0.0, 0.15);
  enml.num_skip_readings = 4;

  -- -- Odometry parameters.
  enml.min_rotation = deg2rad(5);
  enml.min_translation = 0.1;

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.15;
  enml.max_stf_angle_error = deg2rad(35.0);
  enml.max_correspondences_per_point = 6;
  enml.point_correlation_factor = 1.0 / 10.0;

  -- Parameters for LTF constraints.
  enml.map_huber_loss = 0.1;
  enml.max_point_to_line_distance = 0.15;
  enml.max_angle_error = deg2rad(35.0);
  enml.map_correlation_factor = 1.0 / 5.0;

  -- Parameters for visibility constraints.
  enml.use_visibility_constraints = true;
  enml.visibility_correlation_factor = 0.02;

  -- MLE Optimization parameters.
  enml.pose_increment = 1;
  enml.max_history = 5;
  enml.max_solver_iterations = 30;
  enml.num_repeat_iterations = 2;
  enml.max_repeat_iterations = 4;
  enml.return_jacobian = false;
  enml.num_threads = 12;
  enml.limit_history = false;
end

if RobotConfig.name=="Cobot-Sim" then
  enml.min_episode_length = 10;
end

if RobotConfig.name=="Cobot1" then
  enml.odometry_translation_scale = 1.02;
end

if RobotConfig.name=="Cobot3" then
  enml.min_rotation = deg2rad(5);
  enml.min_translation = 0.1;

  -- Sensor parameters.
  enml.robot_sensor_offset = vec3(0.14, 0.0, 0.25);
  enml.laser_std_dev = 0.05;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 4.0;
  enml.max_normal_point_distance = 0.15;

  -- Parameters for LTF constraints.
  enml.max_point_to_line_distance = 0.15;
  enml.max_angle_error = deg2rad(35.0);
  enml.map_correlation_factor = 1.0 / 30.0;

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.15;
  enml.point_correlation_factor = 1.0 / 20.0;

  enml.visibility_correlation_factor = 0.01;

  -- Odometry constraints.
  enml.max_update_period = 1.0;

  -- MLE Optimization parameters.
  enml.min_episode_length = 20;
  enml.pose_increment = 1;
  enml.max_history = 80;
  enml.max_solver_iterations = 30;
  enml.num_repeat_iterations = 1;
  enml.max_repeat_iterations = 4;
  enml.num_threads = 4;
end

if RobotConfig.name=="Cobot4" then
  enml.min_rotation = deg2rad(20);
  enml.min_translation = 0.1;

  -- Sensor parameters.
  enml.robot_sensor_offset = vec3(0.165, 0.0, 0.25);
  enml.laser_std_dev = 0.10;
  enml.min_point_cloud_range = 0.02;
  enml.max_point_cloud_range = 12.0;
  enml.max_normal_point_distance = 0.15;

  -- Parameters for LTF constraints.
  enml.max_point_to_line_distance = 0.35;
  enml.max_angle_error = deg2rad(35.0);
  enml.map_correlation_factor = 1.0 / 30.0;

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.15;
  enml.point_correlation_factor = 1.0 / 20.0;

  enml.visibility_correlation_factor = 0.01;

  -- Odometry constraints.
  enml.max_update_period = 1.0;

  -- MLE Optimization parameters.
  enml.min_episode_length = 20;
  enml.pose_increment = 1;
  enml.max_history = 75;
  enml.max_solver_iterations = 10;
  enml.num_repeat_iterations = 1;
  enml.max_repeat_iterations = 4;
  enml.num_threads = 4;
end

if RobotConfig.name=="airsim" then
  enml.map_name = "Airsim-Neighborhood";
  enml.starting_loc_x = 0;
  enml.starting_loc_y = 0;
  enml.starting_angle = deg2rad(0.0);
end

-- Parameters for probabilistic object maps.
ProbabilisticObjectMaps = {
  object_distance_threshold = 0.03;
  min_object_points = 600;
  laser_angular_resolution = deg2rad(360.0 / 1024.0);
  laser_std_dev = 0.05;
  image_resolution = 0.02;
  image_border = 15.0 * 0.02;
  min_sdf_value = -10.0 * 0.02;
  max_sdf_value = 10.0 * 0.02;
  min_sdf_weight = 0.2;
  max_sdf_weight = 0.7;
  min_sdf_inference_weight = 0.4;
  generate_sdf = false;
  sdf_mistrust_weight = 1.0;
  matching_angular_resolution = deg2rad(3.0);
  matching_delta_loc = 0.24;
  matching_loc_resolution = 0.06;
  max_clique_kld_value = 5.0;
  min_clique_overlap_value = 0.80;
  max_num_objects = 60;
  epsilon_occupancy = 0.01;
  occupancy_threshold = 0.3;
  good_match_threshold = 0.7;
};

if enml_domain == "freiburg" then
  -- Univ. Freiburg parameters.

  -- datasets/freiburg_parkinglot/logs/*.bag
  enml.starting_location = vec2(0.0, 0.0);
  enml.starting_angle = deg2rad(0.0);
  enml.map_name = "freiburg_parkinglot";

  -- Freiburg Sensor parameters.
  enml.laser_std_dev = 0.1;
  enml.min_point_cloud_range = 0.1;
  enml.max_point_cloud_range = 70.0;
  enml.max_normal_point_distance = 0.5;
  enml.robot_sensor_offset = vec3(0.0, 0.0, 0.0);


  -- Parameters for LTF constraints.
  enml.map_huber_loss = 0.1;
  enml.max_point_to_line_distance = 0.5;
  enml.max_angle_error = deg2rad(89.0);
  enml.map_correlation_factor = 1; -- 1.0 / 50.0;

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.25;
  enml.max_stf_angle_error = deg2rad(89.0);
  enml.max_correspondences_per_point = 10;
  enml.point_correlation_factor = 1; -- 1.0 / 50.0;

  -- Parameters for Odometry constraints.
  enml.odometry_radial_stddev_rate = 0.2; -- 10.0;
  enml.odometry_tangential_stddev_rate = 0.2; -- 3.0;
  enml.odometry_angular_stddev_rate = 0.2; -- 30.0;
  enml.odometry_translation_min_stddev = 0.01;
  enml.odometry_translation_max_stddev = 1000000.5;
  enml.odometry_rotation_min_stddev = deg2rad(1000.0);
  enml.odometry_rotation_max_stddev = deg2rad(50000000.0);

  -- Odometry parameters.
  enml.min_rotation = deg2rad(10.5);
  enml.min_translation = 0.5;

  enml.use_visibility_constraints = false;

  -- Parameters for episode segmentation.
  enml.min_ltf_ratio = 0.7;
  enml.min_episode_length = 20;

  -- Parameters for object clustering.
  enml.object_distance_threshold = 0.2;
  enml.min_object_points = 500;

  -- MLE Optimization parameters.
  enml.pose_increment = 1;
  enml.max_history = 80;
  enml.max_solver_iterations = 50;
  enml.num_repeat_iterations = 4;
  enml.max_repeat_iterations = 6;

  ProbabilisticObjectMaps.object_distance_threshold = 0.1;
  ProbabilisticObjectMaps.min_object_points = 600;
  ProbabilisticObjectMaps.laser_angular_resolution = deg2rad(1.0);
  ProbabilisticObjectMaps.laser_std_dev = 0.1;
  ProbabilisticObjectMaps.image_resolution = 0.08;
  ProbabilisticObjectMaps.image_border = 15.0 * 0.08;
  ProbabilisticObjectMaps.min_sdf_value = -10.0 * 0.08;
  ProbabilisticObjectMaps.max_sdf_value = 10.0 * 0.08;
  ProbabilisticObjectMaps.min_sdf_weight = 0.2;
  ProbabilisticObjectMaps.max_sdf_weight = 0.7;
  ProbabilisticObjectMaps.min_sdf_inference_weight = 0.4;
  ProbabilisticObjectMaps.generate_sdf = false;
  ProbabilisticObjectMaps.sdf_mistrust_weight = 1.0;
  ProbabilisticObjectMaps.matching_angular_resolution = deg2rad(6.0);
  ProbabilisticObjectMaps.matching_delta_loc = 1.0;
  ProbabilisticObjectMaps.matching_loc_resolution = 0.2;
  ProbabilisticObjectMaps.max_clique_kld_value = 5.0;
  ProbabilisticObjectMaps.min_clique_overlap_value = 0.55;
  ProbabilisticObjectMaps.max_num_objects = 20;
  ProbabilisticObjectMaps.epsilon_occupancy = 0.05;
  ProbabilisticObjectMaps.occupancy_threshold = 0.3;
  ProbabilisticObjectMaps.good_match_threshold = 0.7;

elseif enml_domain == "orebro" then
  -- datasets/orebro/logs/*.bag
  enml.starting_location = vec2(0.0, 0.0);
  enml.starting_angle = deg2rad(0.0);
  enml.map_name = "orebro";


  -- Parameters for LTF constraints.
  enml.map_huber_loss = 0.05;
  enml.max_point_to_line_distance = 0.15;
  enml.max_angle_error = deg2rad(25.0);
  enml.map_correlation_factor = 1.0 / 100.0;

  -- Parameters for STF constraints.
  enml.point_match_threshold = 0.05;
  enml.max_stf_angle_error = deg2rad(89.0);
  enml.max_correspondences_per_point = 2;
  enml.point_correlation_factor = 1.0 / 50.0;

  enml.use_visibility_constraints = false;
  visibility_correlation_factor = 0.002;

  -- Odometry Parameters
  enml.max_odometry_delta_loc = 10.0;
  enml.max_odometry_delta_angle = deg2rad(170.0);
  enml.min_rotation = -1;
  enml.min_translation = -1;
  enml.odometry_radial_stddev_rate = 180.0;
  enml.odometry_tangential_stddev_rate = 180.0;
  enml.odometry_angular_stddev_rate = 180.0;
  enml.odometry_translation_min_stddev = 0.01;
  enml.odometry_translation_max_stddev = 1000000.5;
  enml.odometry_rotation_min_stddev = deg2rad(1000.0);
  enml.odometry_rotation_max_stddev = deg2rad(50000000.0);
  enml.max_odometry_delta_loc = 1.2;
  enml.max_odometry_delta_angle = deg2rad(180.0);

  -- CoBot Sensor parameters.
  enml.laser_std_dev = 0.1;
  enml.min_point_cloud_range = 0.1;
  enml.max_point_cloud_range = 40.0;
  enml.max_normal_point_distance = 0.2;
  enml.robot_sensor_offset = vec3(0.0, 0.0, 0.0);

  -- MLE Optimization parameters.
  enml.pose_increment = 10;
  enml.max_solver_iterations = 20;
  enml.min_episode_length = 20;
  enml.max_history = 120;
  enml.num_repeat_iterations = 4;
  enml.max_repeat_iterations = 20;
end
