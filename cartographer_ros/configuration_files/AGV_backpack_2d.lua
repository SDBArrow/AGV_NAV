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
    map_frame = "map", -- 一般設置為 map 用來發布 submap 的 ros id
    tracking_frame = "laser",  -- slam 算法要跟蹤的 id base_link
    published_frame = "base_link", -- 用來發布 pose 的 id base_link

    odom_frame = "odom", --只有使用里程計的時候會用到 > provide_odem_frame = true
    provide_odom_frame = true,  --算法提供的里程計

    use_odometry = false,  -- 不使用外部里程計
    use_nav_sat = false,
    use_landmarks = false,

    publish_to_tf = true,
    publish_tracked_pose = true, --發布pose
    publish_frame_projected_to_2d = false,

    num_laser_scans = 1, --slam 可以輸入的/scan話題數目的最大值
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,

    rangefinder_sampling_ratio = 1,
    odometry_sampling_ratio = 1,
    fixed_frame_pose_sampling_ratio = 1,
    imu_sampling_ratio = 1,
    landmarks_sampling_ratio = 1,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,

    submap_publish_period_sec = 0.3, --發布子圖的間隔時間，以秒為單位，例如0.3秒。
    pose_publish_period_sec = 5e-3, --發布姿勢的時間間隔（以秒為單位），例如200Hz的頻率為5e-3。
    trajectory_publish_period_sec = 30e-3, --發布軌跡標記的時間間隔（以秒為單位），例如30e-3，持續30毫秒。
}
--自行調整
MAP_BUILDER.use_trajectory_builder_2d = true --使用2D還3D
TRAJECTORY_BUILDER_2D.min_range = 0.15 --lidar最小檢測距離
TRAJECTORY_BUILDER_2D.max_range = 6 --lidar最大檢測距離
--3d雷達用在2d上把一定高度範圍掃描映射到2d
--TRAJECTORY_BUILDER_2D.min_z = -0.05,
--TRAJECTORY_BUILDER_2D.max_z = 0.5,
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --建構一次完整的掃描所需要的訊息量,因為只有一個lidar所以只設置為1,太多會跑到天荒地老
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025 -- 使用voxel濾波時立方體邊長的大小，0.025這個值是固定的
--TRAJECTORY_BUILDER_2D.*adaptive_voxel_filter.max_length --採樣最大間隔
--TRAJECTORY_BUILDER_2D.*adaptive_voxel_filter.min_num_points --採樣後最小雲點數量
TRAJECTORY_BUILDER_2D.use_imu_data = false --沒有使用imu
--TRAJECTORY_BUILDER_nD.imu_gravity_time_constant = 10 --沒有imu所以不需要設置
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --因為沒使用imu要透過閉環檢測計算的方式彌補
--如果這項為false，則掃描匹配使用的是通過前一幀位置，將當前scan與之前做對比，使用高斯牛頓法代求解最小二乘問題求得當前scan的坐標變換。如果這項為true，則使用閉環檢測的方法，將當前scan在一定的搜索範圍內搜索，範圍為設定的平移距離及角度大小，然後在將scan插入到匹配的最優位置處。這種方式建圖的效果非常好，即使建圖有漂移也能夠修正回去，但是這個方法的計算複雜度非常高，非常耗cpu。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =1e-1
--為了避免每個子地圖插入太多的掃描數據，一旦掃描匹配器找到了兩個掃描數據之間的運動關係，就會變成運動過濾器。如果本次掃描的運動被認為不夠重要，那麼本次掃描會被分解。一次掃描能夠被插入現代子地圖的替代是它的運動超過了一定的距離，角度或時間閾值。
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2 --調整2後初始定位變快 偏移量便高？
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 --Submaps越小，內部漂移越小；Submaps越大，越有利於全局定位的回環檢測

POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options

