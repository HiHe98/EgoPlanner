
#include <plan_manage/ego_replan_fsm.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/make_shared.hpp>
ros::Publisher pub_finish_event;   // ����

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
    pub_finish_event = nh.advertise<std_msgs::Empty>("/ego_planner/finish_event", 1, true);
    wp_single_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal2", 1, &EGOReplanFSM::singleGoalCallback, this);
    stop_plan_sub_ = nh.subscribe<std_msgs::Empty>("/egoplanner/stopplan", 10, &EGOReplanFSM::stopPlanCallback, this);
    hover_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void ego_planner::EGOReplanFSM::stopPlanCallback(const std_msgs::Empty::ConstPtr& msg) {
      ROS_WARN("[EGOReplanFSM] Received stop plan command: switching to WAIT_TARGET + stopping flight!");
      
      // 1. 切换状态机，暂停规划
      changeFSMExecState(WAIT_TARGET, "Received /egoplanner/stopplan command");
      finish_flag_ = false;

      // 2. 发布悬停速度指令（核心修改：调用新的速度发布函数），有误待修复
      // publishHoverVelocity();
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    ROS_WARN("[waypointCB] enter, last_t=%.3f  global_dur=%.3f  pts=%zu",
        planner_manager_->global_data_.last_progress_time_,
        planner_manager_->global_data_.global_duration_,
        msg->poses.size());
    if (msg->poses[0].pose.position.z < -0.1)
        return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    // 1. 强制目标点Z坐标与无人机当前高度一致
    double fixed_z = odom_pos_.z();
    end_pt_ << msg->poses[0].pose.position.x, 
              msg->poses[0].pose.position.y, 
              fixed_z;

    // 2. 调用 planGlobalTraj（无内部 success 分支，通过返回值判断）
    bool success = planner_manager_->planGlobalTraj(
        odom_pos_, 
        odom_vel_, 
        Eigen::Vector3d::Zero(), 
        end_pt_, 
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
    );

    have_target_ = true;
    ROS_WARN("[planGlobalTraj] success=%d  new_duration=%.3f", 
            success, 
            planner_manager_->global_data_.global_duration_);

    // 3. 可视化目标点
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    // 4. 核心修改：根据返回值 success 触发状态切换（重点适配第一次航点）
    if (success)
    {
        /*** 全局路径可视化 ***/
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            Eigen::Vector3d traj_pt = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
            traj_pt.z() = fixed_z;
            gloabl_traj[i] = traj_pt;
        }

        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        /*** 状态机切换：新增 INIT 分支，确保第一次航点直接进入规划 ***/
        if (exec_state_ == INIT) {
            // 第一次航点（初始状态为 INIT）：切换到 GEN_NEW_TRAJ
            changeFSMExecState(GEN_NEW_TRAJ, "First waypoint trigger (planGlobalTraj success)");
        }
        else if (exec_state_ == WAIT_TARGET)
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "TRIG");

        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
        ROS_INFO_THROTTLE(1, "[FSM] In WAIT_TARGET state, waiting for NEW waypoint. Current have_target_: %d", have_target_);
        // 仅在“无目标”时返回，有目标时也不跳转，保持 WAIT_TARGET 状态
        if (!have_target_)
            return;
        // 关键：删除 else 分支的自动跳转逻辑！
        // 原本的 else { changeFSMExecState(GEN_NEW_TRAJ, "FSM"); } 必须删掉
        break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          ROS_WARN("[EGO] >>>>>  publish finish_event (normal)  <<<<<");
          std_msgs::Empty e;
          pub_finish_event.publish(e);
          ROS_INFO("[EGO] finish_event: traj_complete");
          return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
          if (odom_vel_.norm() < 0.1) {
              ROS_INFO("[EGO]emergency_stop");
              changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 新增：阻断WAIT_TARGET状态下的规划
    if (exec_state_ == WAIT_TARGET) {
        ROS_INFO("[callReboundReplan] Current state is WAIT_TARGET, skip replanning.");
        return false;
    }
    getLocalTarget();

    // 强制局部目标点Z坐标与起点一致
    double fixed_z = start_pt_.z();
    local_target_pt_.z() = fixed_z;
    local_target_vel_.z() = 0.0;

    // 调用局部规划器
    bool close_reason = false;
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_,
        start_vel_,
        start_acc_,
        local_target_pt_,
        local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj,
        close_reason
    );
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;

      /* 修正轨迹控制点的Z坐标 */
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.cols(); ++i) {
        pos_pts(2, i) = fixed_z;
      }

      // 计算时间间隔ts（替代 getTs()）
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      double ts = knots(1) - knots(0); // 均匀B样条的knot间隔相等

      // 重新构造轨迹对象
      info->position_traj_ = UniformBspline(pos_pts, 3, ts);

      /* 发布轨迹 */
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);

      visualization_->displayOptimalList(info->position_traj_.getControlPoint(), 0);
    }
    else if (close_reason && !finish_flag_)  // 仅当未处理过且close_reason为true时执行
    {
        ROS_WARN("[FSM] reboundReplan failed due to CLOSE_TO_GOAL, switching to WAIT_TARGET");
        // 切换至WAIT_TARGET状态
        changeFSMExecState(WAIT_TARGET, "接近目标，无需继续规划");
        // 发布完成事件
        std_msgs::Empty e;
        pub_finish_event.publish(e);
        // 关键：标记为已处理，后续不再进入该分支
        finish_flag_ = true;
    }
    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    // 1. 初始化全局路径时间参数（确保为轨迹时间，而非系统时间）
    double global_duration = planner_manager_->global_data_.global_duration_;
    double last_progress = planner_manager_->global_data_.last_progress_time_;

    // 安全检查：若last_progress未初始化或超出轨迹时长，强制重置为0
    if (last_progress < 0 || last_progress > global_duration + 1e-5) {
      last_progress = 0.0;
      planner_manager_->global_data_.last_progress_time_ = last_progress;
      ROS_WARN("Reset last_progress_time_ to 0 (invalid value: %.3f)", last_progress);
    }

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_; // 采样步长
    double dist_min = 9999.0;
    double dist_min_t = last_progress; // 初始最小距离时间为上次进度

    // 2. 遍历全局路径，寻找局部目标点（从last_progress开始）
    Eigen::Vector3d local_target_pt = end_pt_; // 默认目标为终点
    bool found_valid_target = false;

    for (double t = last_progress; t < global_duration; t += t_step) {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm(); // 距离当前位置的距离

      // 更新最小距离对应的时间（用于进度跟踪）
      if (dist < dist_min) {
        dist_min = dist;
        dist_min_t = t;
      }

      // 找到第一个超出规划视野的点作为局部目标
      if (dist >= planning_horizen_) {
        local_target_pt = pos_t;
        found_valid_target = true;
        break;
      }
    }

    // 3. 若未找到超出视野的点，目标设为全局终点
    if (!found_valid_target) {
      local_target_pt = end_pt_;
    }

    // 4. 更新全局路径进度时间（使用最小距离对应的时间）
    planner_manager_->global_data_.last_progress_time_ = dist_min_t;

    // 5. 计算局部目标速度（确保Z分量为0）
    if ((end_pt_ - local_target_pt).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_)) {
      local_target_vel_ = Eigen::Vector3d::Zero();
    } else {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(dist_min_t);
    }
    local_target_vel_.z() = 0.0; // 强制Z轴速度为0

    // 6. 强制局部目标点Z坐标与起点一致（XY平面约束）
    local_target_pt_.z() = start_pt_.z();
    local_target_pt_ = local_target_pt;
  }
  void EGOReplanFSM::singleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      nav_msgs::Path path;
      path.header = msg->header;
      path.poses.push_back(*msg);
      // 关键：接收新目标时，重置finish_flag_，允许第一次close_reason分支执行
      finish_flag_ = false;
      waypointCallback(boost::make_shared<nav_msgs::Path>(path));
  }

  void ego_planner::EGOReplanFSM::publishHoverVelocity() {
    // 构造悬停速度指令：零线性速度 + 零角速度（停止所有运动，悬停）
    geometry_msgs::Twist hover_vel;
    // 线性速度：x/y/z 方向均为 0（停止平移）
    hover_vel.linear.x = 0.0;
    hover_vel.linear.y = 0.0;
    hover_vel.linear.z = 0.0;
    // 角速度：x/y/z 方向均为 0（停止旋转）
    hover_vel.angular.x = 0.0;
    hover_vel.angular.y = 0.0;
    hover_vel.angular.z = 0.0;

    // 持续发布悬停指令（发布 5 次，确保 PX4 收到并响应）
    for (int i = 0; i < 5; ++i) {
        hover_vel_pub_.publish(hover_vel);
        ros::Duration(0.05).sleep();  // 间隔 50ms，避免消息丢失
    }

    ROS_INFO("[publishHoverVelocity] Published hover command (zero velocity) to PX4!");
  }
} // namespace ego_planner
