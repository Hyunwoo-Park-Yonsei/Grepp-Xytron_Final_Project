-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
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
  publish_to_tf = true,
  publish_tracked_pose = true,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4


TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.18 --최소범위
TRAJECTORY_BUILDER_2D.max_range = 12. --라이다 최대범위
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 14. --max_range보다 더 먼 범위
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2 -- 범위 내 데이터 측정 반복수
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025 -- 데이터 처리를 얼마나 빠르게 할건지 클수록 빨라짐
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5 --정한 수보다 큰 길이 제외, 로컬슬램관련
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200 -- 로컬 스램 관련, 최소 타겟 개수
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30. -- 50 디폴트
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.8 --초기 추측을 통해 검색 일치 항목이 하위 맵에 가장 적합한 지점을 찾는 함수 ceres_scan_matcher -- 각 디바이스가 측정한 점 사이에 간격을 얼마나 신뢰하는지에 대한 가중치 값
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e0 --각 디바이스가 측정한 점들의 위치를 얼마만큼 신뢰하는지에 대한 가중치값
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e0 --측정한 점들의 회전정도를 얼만큼 신뢰하는 지에 대한 가중치 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2. -- 정해진 시간 이후 데이터로 서브맵 그리기, default 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1 -- 정해진 거리 이상의 데이터로만 서브맵 그리기
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0) -- 정한 각도 이후 데이터만 서브맵
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10 -- 1개의 서브맵을 만들때 사용할 라이다 개수, default:70~100 
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49

--activate for localization

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}


POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 5
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e7
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e7
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(10.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 1.8
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 1e0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1e0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
--POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
--POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
--POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
--POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
--POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.max_num_final_iterations = 20
POSE_GRAPH.global_sampling_ratio = 0.03
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.global_constraint_search_after_n_seconds = 5

return options


