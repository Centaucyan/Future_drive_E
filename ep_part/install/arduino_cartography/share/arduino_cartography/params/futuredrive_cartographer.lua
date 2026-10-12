include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- TF:
  -- map -> odom -> base_link -> laser_frame
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  -- 기존 브리지에서 odom -> base_link를 발행하므로
  -- Cartographer는 map -> odom만 발행한다.
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 현재 IMU를 사용하지 않음
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- RPLidar C1
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- 소형 이동 로봇과 휠 오도메트리용 설정
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    0.1

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =
    math.rad(20.0)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    10.0

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    0.1

-- 서브맵 설정
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 루프 클로저와 pose graph 최적화
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.huber_scale = 100.0

return options