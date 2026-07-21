include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7

-- LOCAL DATA AND SCANS
TRAJECTORY_BUILDER_3D.min_range = 0.6
TRAJECTORY_BUILDER_3D.max_range = 30.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- LOCAL SCAN MATCHER STABILITY
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10

-- SUBMAPS & GRAVITY
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 1.0
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 70

-- MOTION FILTER
--TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 10.0
--TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(3.0)

-- GLOBAL OPTIMIZATION GRAPH
POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 100.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 200.0

return options