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

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2-- absolutely    need    to    adapt    to    the    needs    of    yourbag:
TRAJECTORY_BUILDER_2D.min_range = 0.1

TRAJECTORY_BUILDER_2D.max_range = 15.0 --2/5xlaser: 15.0,  og 2xlaser + 3xPC: 4.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 17.0 --2xlaser: 17.0,  2xlaser + 3xPC: 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability=0.58

TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 2.5

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false -- true: initial guess for scan matcher is based on S2M match instead of extrapolated pose.

--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.01)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight= 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight= 60.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight= 60.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 350
POSE_GRAPH = {
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 10., --ceres
      translation_weight = 60., --ceres
      rotation_weight = 60., --ceres
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 10., --satt til ceres
      occupied_space_weight_1 = 60., -- satt til ceres
      translation_weight = 60.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 1e10,  --okt fra 5e
    odometry_rotation_weight = 1e10, -- okt fra e5
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
}
 


-- REAL SENSE --
--[[ POSE_GRAPH.constraint_builder.sampling_ratio = 0.9

POSE_GRAPH.optimization_problem.huber_scale = 1
POSE_GRAPH.constraint_builder.min_score = 0.7

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 300 
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 50 ]]


return options
