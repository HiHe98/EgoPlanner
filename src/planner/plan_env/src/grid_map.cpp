#include "plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, 999.0);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);
  // 2. 动态更新参数
  node_.param("grid_map/dynamic_map_enable", dynamic_map_enable_, true);
  node_.param("grid_map/update_threshold", update_threshold_, 8.0);
  node_.param("grid_map/min_move_dist", min_move_dist_, 2.0);
  node_.param("grid_map/update_hysteresis", update_hysteresis_, 2.0);
  node_.param("grid_map/update_freq", update_check_freq_, 1.0);

  // 新增：读取无人机安全半径参数（默认值 0.5m，兼容之前的配置）
  node_.param("grid_map/drone_safe_radius", mp_.drone_safe_radius_, 0.5);
  node_.param("grid_map/enable_drone_self_filter", mp_.enable_drone_self_filter_, true);
  // 新增：读取缓冲栅格参数（默认1个栅格）
  node_.param("grid_map/safe_grid_offset", safe_grid_offset_, 1);

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;
  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  // 初始化地图原点：固定中心在 (0,0,0)，不受无人机初始位置影响
  // 注意：直接使用已声明的 x_size、y_size（无需重新定义 double x_size）
  x_size = mp_.map_size_.x();  // 地图 X 轴尺寸（从参数读取，如30m）
  y_size = mp_.map_size_.y();  // 地图 Y 轴尺寸（如30m）

  // 1. 固定地图中心为 (0,0,0)（核心修改）
  Eigen::Vector3d map_fixed_center(0.0, 0.0, 0.0);  // 固定中心，不依赖无人机位置

  // 2. 计算初始原点：中心 - 地图尺寸/2（确保地图范围对称围绕 (0,0,0)）
  Eigen::Vector3d init_origin;
  init_origin.x() = map_fixed_center.x() - x_size / 2.0;  // 如 0 - 30/2 = -15m
  init_origin.y() = map_fixed_center.y() - y_size / 2.0;  // 如 0 - 30/2 = -15m
  init_origin.z() = mp_.ground_height_;  // Z 轴原点仍用地面高度（保持不变）

  // 3. 打印无人机初始位置（仅调试用，不影响地图初始化）
  // ROS_INFO("[调试] 无人机初始位置 md_.camera_pos_: (%.2f, %.2f, %.2f)",
  //         md_.camera_pos_.x(), md_.camera_pos_.y(), md_.camera_pos_.z());

  // 4. 初始化核心参数（确保地图范围正确）
  mp_.map_origin_ = init_origin;
  last_map_origin_ = init_origin;  // 同步初始化 last_map_origin_（动态更新时用）
  mp_.map_min_boundary_ = init_origin;
  mp_.map_max_boundary_ = init_origin + mp_.map_size_;

  // 5. 输出初始化日志，确认正确性（关键验证）
  ROS_WARN("[GridMap] Init: 地图固定中心=(%.2f,%.2f,%.2f)",
          map_fixed_center.x(), map_fixed_center.y(), map_fixed_center.z());
  ROS_WARN("[GridMap] Init: 地图原点=(%.2f,%.2f,%.2f)",
          init_origin.x(), init_origin.y(), init_origin.z());
  ROS_WARN("[GridMap] Init: 地图范围 X=[%.2f,%.2f], Y=[%.2f,%.2f], Z=[%.2f,%.2f]",
          mp_.map_min_boundary_.x(), mp_.map_max_boundary_.x(),
          mp_.map_min_boundary_.y(), mp_.map_max_boundary_.y(),
          mp_.map_min_boundary_.z(), mp_.map_max_boundary_.z());

  // 6. 初始化定时检查定时器（1Hz 触发，保持不变）
  // update_check_timer_ = node_.createTimer(ros::Duration(1.0 / update_check_freq_),
  //                                         &GridMap::updateCheckCallback, this);

  // ROS_WARN("[GridMap] Update check timer started: %.1f Hz", update_check_freq_);

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

  // 新增：按需重置（保留 keep_boundary 内的区域，重置外部区域）
  // 适配数据迁移场景：重叠区域保留空白，等待迁移数据覆盖
  void GridMap::resetBuffer(const Eigen::AlignedBox3d& keep_boundary)
  {
  // 新地图的全局边界（从配置参数获取）
  Eigen::Vector3d map_min = mp_.map_min_boundary_;
  Eigen::Vector3d map_max = mp_.map_max_boundary_;

  // 将新地图全局边界转换为体素索引范围
  Eigen::Vector3i min_id, max_id;
  posToIndex(map_min, min_id);
  posToIndex(map_max, max_id);
  boundIndex(min_id);  // 确保索引不小于0
  boundIndex(max_id);  // 确保索引不超过地图最大体素数

  // 遍历新地图所有体素，仅重置“不在保留区域内”的体素
  for (int x = min_id(0); x <= max_id(0); ++x)
  {
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        // 1. 计算当前体素的世界坐标
        Eigen::Vector3d voxel_pos;
        indexToPos(Eigen::Vector3i(x, y, z), voxel_pos);

        // 2. 判断是否在保留区域内：在则跳过（不重置），不在则重置
        if (keep_boundary.contains(voxel_pos))
          continue;

        // 3. 重置体素数据（与原有 resetBuffer 逻辑一致）
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;  // 未知状态
        md_.occupancy_buffer_inflate_[idx] = 0;  // 非膨胀障碍
      }
    }
  }

  // 更新局部边界（与原有 resetBuffer 逻辑一致）
  md_.local_bound_min_ = min_id;
  md_.local_bound_max_ = max_id;
  }

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
      // 打印虚拟天花板高度参数
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMap();
  publishMapInflate(true);
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
    // 新增：调用动态地图原点更新
    if (dynamic_map_enable_)
    {
      updateDynamicMapOrigin(md_.camera_pos_);
    }
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;

  // 新增：兜底调用，确保位置更新时触发
  if (dynamic_map_enable_)
  {
    updateDynamicMapOrigin(md_.camera_pos_);
  }
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "No odometry data available!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  const double DRONE_SAFE_RADIUS = mp_.drone_safe_radius_;
  int filtered_self_cnt = 0;  // 统计自身区域过滤的点数

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    // ======================================
    // 核心：按开关控制是否执行自身区域过滤
    // ======================================
    if (mp_.enable_drone_self_filter_)
    {
      // 开启：跳过自身区域的点
      double dist_to_drone = (p3d - md_.camera_pos_).norm();
      if (dist_to_drone <= DRONE_SAFE_RADIUS)
      {
        filtered_self_cnt++;
        continue;
      }
    }
    // 关闭：不执行过滤，所有点正常处理

    /* Point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      /* Inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }

    // ========== 新增：缓冲栅格标记循环（核心） ==========
    int buffer_step = inf_step + safe_grid_offset_;  // 原有膨胀+缓冲栅格数
    int buffer_step_z = inf_step_z + safe_grid_offset_;  // z轴同步缓冲
    for (int x = -buffer_step; x <= buffer_step; ++x)
      for (int y = -buffer_step; y <= buffer_step; ++y)
        for (int z = -buffer_step_z; z <= buffer_step_z; ++z)
        {
          p3d_inf(0) = pt.x + x * mp_.resolution_;
          p3d_inf(1) = pt.y + y * mp_.resolution_;
          p3d_inf(2) = pt.z + z * mp_.resolution_;

          posToIndex(p3d_inf, inf_pt);
          if (!isInMap(inf_pt)) continue;

          int idx_inf = toAddress(inf_pt);
          // 仅标记「非障碍物栅格」为缓冲栅格（值=2，区别于障碍物1）
          if (md_.occupancy_buffer_inflate_[idx_inf] != 1) {
            md_.occupancy_buffer_inflate_[idx_inf] = 2;
          }
        }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // // 优化日志：明确显示开关状态和过滤统计
  // ROS_INFO_THROTTLE(1.0,
  //                   "[GridMap] Raw point cloud processing completed. "
  //                   "Total points: %zu, "
  //                   "Filtered self-region points: %d, "
  //                   "Valid points: %zu "
  //                   "(self-filter: %s, safe radius: %.2fm)",
  //                   latest_cloud.points.size(),
  //                   filtered_self_cnt,
  //                   latest_cloud.points.size() - filtered_self_cnt,
  //                   mp_.enable_drone_self_filter_ ? "ON" : "OFF",
  //                   DRONE_SAFE_RADIUS);
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{
  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  // 关键修改1：点云类型改为 PointXYZI（X/Y/Z + Intensity 强度值）
  pcl::PointXYZI pt;
  pcl::PointCloud<pcl::PointXYZI> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        double grid_val = md_.occupancy_buffer_inflate_[toAddress(x, y, z)];
        if (grid_val == 0)  // 安全栅格，跳过不发布
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        // 关键修改2：用 Intensity 字段标记栅格类型（0~255，值越大概率越高/类型不同）
        if (grid_val == 1.0)  // 原始障碍物栅格 → 强度值设为 255（最高）
        {
          pt.intensity = 255.0;
        }
        else if (grid_val == 2.0)  // 膨胀/缓冲栅格 → 强度值设为 128（中等）
        {
          pt.intensity = 128.0;
        }
        else  // 备用类型（扩展用）→ 强度值设为 64
        {
          pt.intensity = 64.0;
        }

        // 赋值坐标（与原逻辑一致）
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  // 关键修改3：PointXYZI 转 ROS 消息（自动兼容 PointCloud2）
  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // 可选：打印点云数量和强度分布日志
  // ROS_INFO("pub colored map: %ld points (obs=255, buffer=128)", cloud.points.size());
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);    
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();   
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;
  
  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;

  // 新增：调用动态地图原点更新（使用最新的 camera_pos_）
  if (dynamic_map_enable_)
  {
    updateDynamicMapOrigin(md_.camera_pos_);
  }
}

  bool GridMap::updateDynamicMapOrigin(const Eigen::Vector3d& drone_pos)
  {
  if (!dynamic_map_enable_)
    return false;

  // 1. 计算当前地图中心（3D，含 Z 轴）- 复用 mp_.map_size_（已从 launch 读取）
  Eigen::Vector3d map_center = last_map_origin_ + mp_.map_size_ / 2.0;
  Eigen::Vector3d offset = drone_pos - map_center;
  double offset_dist = offset.norm();

  ROS_DEBUG("[GridMap] Drone pos: (%.2f,%.2f,%.2f), Map center: (%.2f,%.2f,%.2f), 3D offset: %.2fm",
           drone_pos.x(), drone_pos.y(), drone_pos.z(),
           map_center.x(), map_center.y(), map_center.z(),
           offset_dist);

  // 2. 阈值过滤（使用从 launch 读取的类成员变量，无硬编码）
  static bool just_updated = false;

  if (just_updated)
  {
    // 用类成员变量：update_threshold_（触发阈值）、update_hysteresis_（滞后量）
    if (offset_dist < (update_threshold_ - update_hysteresis_))
    {
      just_updated = false;
      return false;
    }
    else
    {
      return false;
    }
  }

  // （满足防抖+阈值要求）
  if (offset_dist < update_threshold_ || offset_dist < min_move_dist_)
  {
    return false;
  }

  // 3. 计算新原点（复用 mp_.map_size_，无硬编码）
  ROS_INFO("[GridMap] Trigger map origin update! Old center: (%.2f,%.2f,%.2f) → New center: (%.2f,%.2f,%.2f) (3D offset: %.2fm)",
           map_center.x(), map_center.y(), map_center.z(),
           drone_pos.x(), drone_pos.y(), drone_pos.z(),
           offset_dist);

  Eigen::Vector3d old_origin = last_map_origin_;
  Eigen::Vector3d new_origin;
  // X/Y/Z 轴：复用 mp_.map_size_（已从 launch 读取，支持动态调整地图尺寸）
  new_origin.x() = drone_pos.x() - mp_.map_size_(0) / 2.0;
  new_origin.y() = drone_pos.y() - mp_.map_size_(1) / 2.0;
  new_origin.z() = drone_pos.z() - mp_.map_size_(2) / 2.0;

  // 4. 数据迁移 + 更新核心参数
  resetBuffer();
  migrateValidData(old_origin, new_origin);

  mp_.map_origin_ = new_origin;
  mp_.map_min_boundary_ = new_origin;
  mp_.map_max_boundary_ = new_origin + mp_.map_size_;
  last_map_origin_ = new_origin;

  just_updated = true;

  // 输出更新后边界（验证参数生效）
  ROS_INFO("[GridMap] Updated map boundary: x=[%.2f,%.2f], y=[%.2f,%.2f], z=[%.2f,%.2f] (center Z: %.2f)",
           mp_.map_min_boundary_.x(), mp_.map_max_boundary_.x(),
           mp_.map_min_boundary_.y(), mp_.map_max_boundary_.y(),
           mp_.map_min_boundary_.z(), mp_.map_max_boundary_.z(),
           new_origin.z() + mp_.map_size_(2)/2.0);

  // 新增：直接通过地图边界判断起点是否在地图内（无函数依赖，不会报错）
  bool is_start_in_map = (drone_pos.x() >= mp_.map_min_boundary_.x() && drone_pos.x() <= mp_.map_max_boundary_.x()) &&
                        (drone_pos.y() >= mp_.map_min_boundary_.y() && drone_pos.y() <= mp_.map_max_boundary_.y()) &&
                        (drone_pos.z() >= mp_.map_min_boundary_.z() && drone_pos.z() <= mp_.map_max_boundary_.z());

  // 保留日志输出（用于调试）
  if (!is_start_in_map)
  {
    ROS_ERROR("[GridMap] 无人机起点 (%.2f,%.2f,%.2f) 不在地图范围内！地图边界：x=[%.2f,%.2f], y=[%.2f,%.2f], z=[%.2f,%.2f]",
              drone_pos.x(), drone_pos.y(), drone_pos.z(),
              mp_.map_min_boundary_.x(), mp_.map_max_boundary_.x(),
              mp_.map_min_boundary_.y(), mp_.map_max_boundary_.y(),
              mp_.map_min_boundary_.z(), mp_.map_max_boundary_.z());
  }

  return true;
  }

  // 计算原地图与新地图的重叠边界（世界坐标系）
  Eigen::AlignedBox3d GridMap::calcOverlapBoundary(const Eigen::Vector3d& old_origin, 
                                                 const Eigen::Vector3d& new_origin)
  {
  // 原地图边界
  Eigen::Vector3d old_min = old_origin;
  Eigen::Vector3d old_max = old_origin + mp_.map_size_;
  // 新地图边界
  Eigen::Vector3d new_min = new_origin;
  Eigen::Vector3d new_max = new_origin + mp_.map_size_;

  // 重叠区域边界：取两个地图边界的交集
  Eigen::Vector3d overlap_min, overlap_max;
  overlap_min.x() = max(old_min.x(), new_min.x());
  overlap_min.y() = max(old_min.y(), new_min.y());
  overlap_min.z() = max(old_min.z(), new_min.z());
  overlap_max.x() = min(old_max.x(), new_max.x());
  overlap_max.y() = min(old_max.y(), new_max.y());
  overlap_max.z() = min(old_max.z(), new_max.z());

  // 若无重叠区域（偏移过大），返回空边界
  if (overlap_min.x() >= overlap_max.x() || 
      overlap_min.y() >= overlap_max.y() || 
      overlap_min.z() >= overlap_max.z())
  {
    return Eigen::AlignedBox3d(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  return Eigen::AlignedBox3d(overlap_min, overlap_max);
  }

  void GridMap::migrateValidData(const Eigen::Vector3d& old_origin, const Eigen::Vector3d& new_origin)
  {
  // 1. 计算重叠区域
  Eigen::AlignedBox3d overlap_box = calcOverlapBoundary(old_origin, new_origin);
  if (overlap_box.isEmpty())
  {
    ROS_WARN("[migrateValidData] No overlap, full reset!");
    resetBuffer();
    return;
  }

  ROS_INFO("[migrateValidData] Migrating valid data. Overlap boundary: min(%.2f,%.2f,%.2f) → max(%.2f,%.2f,%.2f)",
           overlap_box.min().x(), overlap_box.min().y(), overlap_box.min().z(),
           overlap_box.max().x(), overlap_box.max().y(), overlap_box.max().z());

  // 2. 仅重置新地图的非重叠区域
  resetBuffer(overlap_box);

  // 3. 补充变量声明：计算旧地图中重叠区域的体素索引范围（关键修复！）
  Eigen::Vector3i old_min_idx, old_max_idx;
  // 用旧地图原点计算重叠区域的体素索引
  posToIndex(overlap_box.min(), old_min_idx, old_origin);
  posToIndex(overlap_box.max(), old_max_idx, old_origin);
  // 确保索引在旧地图范围内（避免越界）
  boundIndex(old_min_idx, old_origin);
  boundIndex(old_max_idx, old_origin);

  // 4. 遍历旧地图重叠区域的所有体素，筛选有效数据并迁移
  int migrate_cnt = 0;
  Eigen::Vector3d old_voxel_pos, new_voxel_pos;
  Eigen::Vector3i new_voxel_idx;
  int old_addr, new_addr;

  // 现在可以正常使用 old_min_idx 和 old_max_idx 了
  for (int x = old_min_idx.x(); x <= old_max_idx.x(); ++x)
  {
    for (int y = old_min_idx.y(); y <= old_max_idx.y(); ++y)
    {
      for (int z = old_min_idx.z(); z <= old_max_idx.z(); ++z)
      {
        // 计算旧地图中当前体素的世界坐标（调用修复后的重载函数）
        indexToPosWithOrigin(Eigen::Vector3i(x, y, z), old_voxel_pos, old_origin);

        // 筛选有效数据：仅迁移占据状态的体素
        old_addr = oldToAddress(x, y, z);
        if (md_.occupancy_buffer_[old_addr] <= mp_.min_occupancy_log_)
          continue;

        // 计算该体素在新地图中的索引
        new_voxel_pos = old_voxel_pos;
        posToIndex(new_voxel_pos, new_voxel_idx);
        if (!isInMap(new_voxel_idx))
          continue;

        // 迁移数据到新地图
        new_addr = toAddress(new_voxel_idx);
        md_.occupancy_buffer_[new_addr] = md_.occupancy_buffer_[old_addr];
        md_.occupancy_buffer_inflate_[new_addr] = md_.occupancy_buffer_inflate_[old_addr];

        migrate_cnt++;
      }
    }
  }

  ROS_INFO("[migrateValidData] Migration finished! Migrated %d valid obstacle voxels.", migrate_cnt);
  }


  // 重载 posToIndex：根据指定原点计算体素索引
  void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, const Eigen::Vector3d& origin)
  {
  idx.x() = floor((pos.x() - origin.x()) * mp_.resolution_inv_);
  idx.y() = floor((pos.y() - origin.y()) * mp_.resolution_inv_);
  idx.z() = floor((pos.z() - origin.z()) * mp_.resolution_inv_);
  }

  // 修复后：带自定义原点的 indexToPos 实现
  void GridMap::indexToPosWithOrigin(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, const Eigen::Vector3d& origin)
  {
  pos.x() = origin.x() + (idx.x() + 0.5) * mp_.resolution_;
  pos.y() = origin.y() + (idx.y() + 0.5) * mp_.resolution_;
  pos.z() = origin.z() + (idx.z() + 0.5) * mp_.resolution_;
  }

  // 重载 boundIndex：根据指定原点限制索引在地图范围内
  void GridMap::boundIndex(Eigen::Vector3i& idx, const Eigen::Vector3d& origin)
  {
  Eigen::Vector3i max_idx;
  max_idx.x() = floor(mp_.map_size_(0) * mp_.resolution_inv_) - 1;
  max_idx.y() = floor(mp_.map_size_(1) * mp_.resolution_inv_) - 1;
  max_idx.z() = floor(mp_.map_size_(2) * mp_.resolution_inv_) - 1;

  idx.x() = max(0, min(idx.x(), max_idx.x()));
  idx.y() = max(0, min(idx.y(), max_idx.y()));
  idx.z() = max(0, min(idx.z(), max_idx.z()));
  }

  // 重载 toAddress：计算指定原点下的体素缓冲区地址
  int GridMap::oldToAddress(int x, int y, int z)
  {
  int max_x = floor(mp_.map_size_(0) * mp_.resolution_inv_);
  int max_y = floor(mp_.map_size_(1) * mp_.resolution_inv_);
  return z * max_x * max_y + y * max_x + x;
  }

  double GridMap::getInflateGridValue(const Eigen::Vector3d& pos) {
  Eigen::Vector3i grid_idx;
  
  // 关键修改：将 mp_.origin_ → mp_.map_origin_（匹配你的 MappingParameters 结构体成员名）
  posToIndex(pos, grid_idx, mp_.map_origin_);
  
  // 检查栅格索引是否在地图有效范围内
  if (!isInMap(grid_idx)) {
    return 0.0;  // 超出地图范围→返回安全值0.0
  }

  // 索引有效：转换为数组地址，返回缓冲栅格原始值
  int idx = toAddress(grid_idx);
  return md_.occupancy_buffer_inflate_[idx];  // 0.0=安全，1.0=障碍物，2.0=缓冲
  }

  // // 新增定时检查回调函数（仅每秒检查1次是否需要更新）
  // void GridMap::updateCheckCallback(const ros::TimerEvent& event)
  // {
  //   if (!dynamic_map_enable_ || md_.camera_pos_.isZero())
  //     return;

  //   // 仅在定时回调中调用更新逻辑，避免高频触发
  //   updateDynamicMapOrigin(md_.camera_pos_);
  // }
// GridMap
