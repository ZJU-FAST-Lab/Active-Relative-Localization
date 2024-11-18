
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    have_trigger_ = !flag_realworld_experiment_;

    int formation_num = -1;
    nh.param("formation/num", formation_num, -1);
    if (formation_num < planner_manager_->pp_.drone_id + 1)
    {
      ROS_ERROR("formation_num is smaller than the drone number, illegal!");
      return;
    }
    std::vector<double> pos;
    nh.getParam("formation/drone" + to_string(planner_manager_->pp_.drone_id), pos);
    formation_pos_ << pos[0], pos[1], pos[2];
    nh.getParam("formation/start", pos);
    formation_start_ << pos[0], pos[1], pos[2];

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);

      // if ( i==0 )
      // {
      //   Eigen::Vector3d dir(waypoints_[0][0]-formation_start_(0), waypoints_[0][1]-formation_start_(1), waypoints_[0][2]-formation_start_(2));
      //   dir.normalize();

      // }
      // else
      // {
      //   Eigen::Vector3d dir(waypoints_[i][0]-waypoints_[i-1][0], waypoints_[i][1]-waypoints_[i-1][1], waypoints_[i][2]-waypoints_[i-1][2]);
      //   dir.normalize();

      // }
    }

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    /*--------------------------------new code start---------------------------------*/
    
    nh.param("drone_id", drone_id_, -1);
    nh.param("drones_num", drones_num_, 3);
    nh.param("fsm/eskf_update", eskf_update_, true);
    nh.param("fsm/drone_busy_duaration", drone_busy_duaration_, 4.0);
    nh.param("fsm/x_noise_step", x_noise_step_, 2.0);
    nh.param("fsm/y_noise_step", y_noise_step_, 2.0);
    nh.param("fsm/z_noise_step", z_noise_step_, 0.5);
    nh.param("fsm/obs_cov_thresh", obs_cov_thresh_, 2.5);
    nh.param("fsm/master_no_noise", master_no_noise_, false);
    nh.param("fsm/active_obs", active_obs_, true);

    // ESKF init
    X_error_ = Eigen::VectorXd::Zero(drones_num_*3);
    P_cov_ = Eigen::MatrixXd::Zero(drones_num_*3, drones_num_*3);
    F_sys_ = Eigen::MatrixXd::Identity(drones_num_*3, drones_num_*3);
    cur_offset.setZero();

    // ros init
    if (drone_id_ == 0) { // master
      obs_share_sub_ = nh.subscribe("obs_share", 5, &EGOReplanFSM::obsShareCallback, this);
      offset_pub_  = nh.advertise<quadrotor_msgs::Offset>("offset_share", 10);
      do_obs_pub_ = nh.advertise<quadrotor_msgs::DoObs>("do_obs", 10);
      do_obs_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::decideObsCallback, this);
    }
    for(int i = 0; i < drones_num_; i ++)
    {
      std::string real_pos_topic =
        "/drone_" + std::to_string(i) + "_visual_slam/real_pos";
      ros::Subscriber real_pos_sub = nh.subscribe(real_pos_topic, 1, &EGOReplanFSM::realPosCallback, this);
      all_real_pos_sub_.push_back(real_pos_sub);
    }
    for(int i = 0; i < drones_num_; i ++)
    {
      std::string odom_topic =
        "/drone_" + std::to_string(i) + "_visual_slam/odom";
      ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, &EGOReplanFSM::allOdomCallback, this);
      all_odom_sub_.push_back(odom_sub);
    }
    
    offset_sub_ = nh.subscribe("offset_share", 5, &EGOReplanFSM::offsetShareCallback, this);
    obs_share_pub_ = nh.advertise<quadrotor_msgs::YoloObs>("obs_share", 10);
    observe_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::observeCallback, this);
    offset_timer_ = nh.createTimer(ros::Duration(0.02), &EGOReplanFSM::correctOffsetCallback, this);
    odom_corrected_pub_ = nh.advertise<nav_msgs::Odometry>("odom_corrected", 10);
    do_obs_sub_ = nh.subscribe("do_obs", 5, &EGOReplanFSM::doObsCallback, this);
    drone_is_busy_[0] = drone_is_busy_[1] = drone_is_busy_[2] = false;

    rela_vis_pub_ = nh.advertise<visualization_msgs::Marker>("planning/relation_vis", 10);
    fov_vis_pub = nh.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
    // for FOV vis
    double vis_dist = 4.5;
    double hor = vis_dist * tan(0.689);  // 40 degree
    double vert = vis_dist * tan(0.524); // 30 degree
    Eigen::Vector3d origin(0, 0, 0);
    Eigen::Vector3d left_up(vis_dist, hor, vert);
    Eigen::Vector3d left_down(vis_dist, hor, -vert);
    Eigen::Vector3d right_up(vis_dist, -hor, vert);
    Eigen::Vector3d right_down(vis_dist, -hor, -vert);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(left_up);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(left_down);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(right_up);
    cam_vertices1_.push_back(origin);
    cam_vertices2_.push_back(right_down);
    cam_vertices1_.push_back(left_up);
    cam_vertices2_.push_back(right_up);
    cam_vertices1_.push_back(right_up);
    cam_vertices2_.push_back(right_down);
    cam_vertices1_.push_back(right_down);
    cam_vertices2_.push_back(left_down);
    cam_vertices1_.push_back(left_down);
    cam_vertices2_.push_back(left_up);

    rpe_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::rpeCallback, this);

    /*--------------------------------new code end-------------------------------------*/

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for odometry and trigger");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_ERROR("Failed to generate the first trajectory!!!");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
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
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wpt_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        formation_start_ = wps_[wpt_id_];
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_], formation_start_);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // local target close to the global target
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          formation_start_ = wps_[wpt_id_];
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_], formation_start_);
          // have_trigger_ = false; // must have trigger
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
        goto force_return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
        {
          have_target_ = false;
          have_trigger_ = false;
          ROS_ERROR("The goal is in obstacles, finish the planning.");
          callEmergencyStop(odom_pos_);

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else
        {
          // pass;
        }
      }
      else if (t_cur > replan_thresh_ ||
               (!touch_the_goal && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
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
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    // static int last_printed_state = -1, dot_nums = 0;

    // if (exec_state_ != last_printed_state)
    //   dot_nums = 0;
    // else
    //   dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    // last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << endl;

    // cout << string(dot_nums, '.');

    // fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    double height;
    measureGroundHeight(height);

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    // bool close_to_the_end_of_safe_segment = (pts_chk.back().back().first - t_cur) < emergency_time_;
    // // bool close_to_goal = (info->traj.getPos(info->duration) - end_pt_).norm() < 1e-5;
    // if (close_to_the_end_of_safe_segment)
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   return;

    //   // if (!close_to_goal)
    //   // {
    //   //   // ROS_INFO("current position is close to the safe segment end.");
    //   //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //   return;
    //   // }
    //   // else
    //   // {
    //   //   double t_step = map->getResolution() / planner_manager_->pp_.max_vel_;
    //   //   for (double t = pts_chk.back().back().first; t < info->duration; t += t_step)
    //   //   {
    //   //     if (map->getInflateOccupancy(info->traj.getPos(t)))
    //   //     {
    //   //       if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
    //   //       {
    //   //         ROS_ERROR("Dense obstacles close to the goal, stop planning.");
    //   //         callEmergencyStop(odom_pos_);
    //   //         have_target_ = false;
    //   //         changeFSMExecState(WAIT_TARGET, "SAFETY");
    //   //         return;
    //   //       }
    //   //       else
    //   //       {
    //   //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //         return;
    //   //       }
    //   //     }
    //   //   }
    //   // }
    // }

    /* ---------- check trajectory ---------- */
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);

    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;

    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());
    if (i_start >= pts_chk.size())
    {
      // ROS_ERROR("i_start >= pts_chk.size()");
      return;
    }
    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    // Eigen::Vector3d last_pt = pts_chk[0][0].second;
    // for (size_t i = 0; i < pts_chk.size(); ++i)
    // {
    //   cout << "--------------------" << endl;
    //   for (size_t j = 0; j < pts_chk[i].size(); ++j)
    //   {
    //     cout << pts_chk[i][j].first << " @ " << pts_chk[i][j].second.transpose() << " @ " << (pts_chk[i][j].second - last_pt).transpose() << " @ " << map->getInflateOccupancy(pts_chk[i][j].second) << endl;
    //     last_pt = pts_chk[i][j].second;
    //   }
    // }

    // cout << "pts_chk[i_start][j_start].first - t_cur = " << pts_chk[i_start][j_start].first - t_cur << endl;
    // cout << "devi = " << (pts_chk[i_start][j_start].second - info->traj.getPos(t_cur)).transpose() << endl;

    // cout << "pts_chk.size()=" << pts_chk.size() << " i_start=" << i_start << endl;
    // Eigen::Vector3d p_last = pts_chk[i_start][j_start].second;
    const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;
        // if ( (p - p_last).cwiseAbs().maxCoeff() > planner_manager_->grid_map_->getResolution() * 1.05 )
        // {
        //   ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
        //   cout << "p=" << p.transpose() << " p_last=" << p_last.transpose() << " dist=" << (p - p_last).cwiseAbs().maxCoeff() << endl;
        // }
        // p_last = p;

        // cout << "t=" << t << " @ "
        //      << "p=" << p.transpose() << endl;
        // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        // if (t_cur < t_2_3 && t >= t_2_3)
        //   break;

        bool occ = false;
        occ |= map->getInflateOccupancy(p);

        // cout << "p=" << p.transpose() << endl;

        // if (occ)
        // {
        //   ROS_WARN("AAAAAAAAAAAAAAAAAAA");
        //   cout << "pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
        //   cout << "i=" << i << " j=" << j << " i_start=" << i_start << " j_start=" << j_start << endl;
        //   cout << "pts_chk.size()=" << pts_chk.size() << endl;
        //   cout << "t=" << t << endl;
        //   cout << "from t=" << info->traj.getPos(t).transpose() << endl;
        //   cout << "from rec=" << p.transpose() << endl;
        // }

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();

            if (dist < CLEARANCE)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              occ = true;
              break;
            }
          }
        }

        if (occ)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    computeYawVel(poly_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        formation_start_, wps_[wpt_id_],
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {

      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);

      /*-----------------------------new code start-----------------------------*/ 
      if (to_turn_) {
        computeYawTurn(poly_msg);
        to_turn_ = false;
        // cout << drone_id_ << " : yaw turn complete." << endl;
      } else {
        computeYawVel(poly_msg);
      }
      /*-----------------------------new code end-----------------------------*/ 

      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp)
  {
    Eigen::Vector3d dir = (next_wp - previous_wp).normalized();
    end_pt_ = next_wp + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
                                        dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
                                        formation_pos_(2));

    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;
      // have_trigger_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory! Undefined actions!");
    }
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    Eigen::Vector3d next_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
    wps_.push_back(next_wp);
    wpt_id_ = wps_.size() - 1;
    if (wpt_id_ >= 1)
    {
      formation_start_ = wps_[wpt_id_ - 1];
    }
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  // void EGOReplanFSM::planGlobalTrajbyGivenWps()
  // {
  //   std::vector<Eigen::Vector3d> wps(waypoint_num_);
  //   for (int i = 0; i < waypoint_num_; i++)
  //   {
  //     wps[i](0) = waypoints_[i][0];
  //     wps[i](1) = waypoints_[i][1];
  //     wps[i](2) = waypoints_[i][2];

  //     end_pt_ = wps.back();
  //   }
  //   bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero(), wps,
  //                                                            Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero());

  //   for (size_t i = 0; i < (size_t)waypoint_num_; i++)
  //   {
  //     visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //     ros::Duration(0.001).sleep();
  //   }

  //   if (success)
  //   {

  //     /*** display ***/
  //     constexpr double step_size_t = 0.1;
  //     int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
  //     std::vector<Eigen::Vector3d> gloabl_traj(i_end);
  //     for (int i = 0; i < i_end; i++)
  //     {
  //       gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
  //     }

  //     have_target_ = true;
  //     have_new_target_ = true;

  //     // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  //     ros::Duration(0.001).sleep();
  //     visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  //     ros::Duration(0.001).sleep();
  //   }
  //   else
  //   {
  //     ROS_ERROR("Unable to generate global trajectory!");
  //   }
  // }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    if (!have_odom_)
    {
      ROS_ERROR("Reject formation flight!");
      return;
    }
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    /*--------------------------------new code start---------------------------------*/
    static nav_msgs::Odometry last_msg = *msg;
    static nav_msgs::Odometry first_odom_msg = *msg;
    odom_pos_(0) = msg->pose.pose.position.x + cur_offset(0); // error state kf
    odom_pos_(1) = msg->pose.pose.position.y + cur_offset(1); 
    odom_pos_(2) = msg->pose.pose.position.z + cur_offset(2);
    /*--------------------------------new code end---------------------------------*/

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;

    /*--------------------------------new code start---------------------------------*/
    nav_msgs::Odometry odom_corrected = *msg;
    odom_corrected.pose.pose.position.x = odom_pos_(0);
    odom_corrected.pose.pose.position.y = odom_pos_(1);
    odom_corrected.pose.pose.position.z = odom_pos_(2);

    // propagate
    if (drone_id_ == 0) {
      double delta_dist = sqrt(pow(fabs(msg->pose.pose.position.x - last_msg.pose.pose.position.x), 2) +
                            pow(fabs(msg->pose.pose.position.y - last_msg.pose.pose.position.y), 2) +
                            pow(fabs(msg->pose.pose.position.z - last_msg.pose.pose.position.z), 2));
      // double dist = sqrt(pow(fabs(msg->pose.pose.position.x - first_odom_msg.pose.pose.position.x), 2) +
      //                     pow(fabs(msg->pose.pose.position.y - first_odom_msg.pose.pose.position.y), 2) +
      //                     pow(fabs(msg->pose.pose.position.z - first_odom_msg.pose.pose.position.z), 2));
      for (int id = 0; id < drones_num_; id ++) { // assume propagate process of all drone approximately similar
        // P_cov_(0 + id*3, 0 + id*3) += 0.02 * 0.02 * 2 * delta_dist * dist;
        // P_cov_(1 + id*3, 1 + id*3) += 0.02 * 0.02 * 2 * delta_dist * dist;
        // P_cov_(2 + id*3, 2 + id*3) += 0.01 * 0.01 * 2 * delta_dist * dist;
        P_cov_(0 + id*3, 0 + id*3) += x_noise_step_ * x_noise_step_ * delta_dist * delta_dist;
        P_cov_(1 + id*3, 1 + id*3) += y_noise_step_ * y_noise_step_ * delta_dist * delta_dist;
        P_cov_(2 + id*3, 2 + id*3) += z_noise_step_ * z_noise_step_ * delta_dist * delta_dist;
      }
    }

    if (drone_id_ == 0 && master_no_noise_) {
      P_cov_(0, 0) = P_cov_(1, 1) = P_cov_(2, 2) = 0;
    }

    odom_corrected_pub_.publish(odom_corrected);
    last_msg = *msg;

    // FOV vis
    vector<Eigen::Vector3d> l1, l2;
    Eigen::Vector3d pos;
    double yaw, x, y, z, w;
    pos << all_real_pos_[drone_id_].pose.pose.position.x, all_real_pos_[drone_id_].pose.pose.position.y,
                                    all_real_pos_[drone_id_].pose.pose.position.z;
    x = all_real_pos_[drone_id_].pose.pose.orientation.x;
    y = all_real_pos_[drone_id_].pose.pose.orientation.y;
    z = all_real_pos_[drone_id_].pose.pose.orientation.z;
    w = all_real_pos_[drone_id_].pose.pose.orientation.w;
    yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
    getFOV(l1, l2, pos, yaw);
    drawFOV(l1, l2);
    /*---------------------------new code end------------------------------*/
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }

          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }

  /*------------------------------new code start-------------------------------*/

  void EGOReplanFSM::realPosCallback(const nav_msgs::Odometry& msg)
  {
    std::string frame_id = msg.child_frame_id;
    int id = frame_id[frame_id.size()-1] - '0';
    all_real_pos_[id] = msg;
  }

  void EGOReplanFSM::obsShareCallback(const quadrotor_msgs::YoloObs& msg) 
  {
    int from_id = msg.from_id;
    int target_id = msg.target_id;
    cout << "drone" << from_id << " detect drone" << target_id << "!" << endl;
    if (eskf_update_) {
      Eigen::Vector3d rela_pos(msg.rela_pos.x, msg.rela_pos.y, msg.rela_pos.z);
      Eigen::Matrix3d R_cov;
      // R_cov.setZero(); // obs no noise
      R_cov << msg.ekfP[0], msg.ekfP[1], msg.ekfP[2],
              msg.ekfP[3], msg.ekfP[4], msg.ekfP[5],
              msg.ekfP[6], msg.ekfP[7], msg.ekfP[8];
      
      Eigen::Matrix<double, 3, Eigen::Dynamic> H_obs(3, drones_num_ * 3); // obs matrix
      H_obs.setZero();
      H_obs.block(0, from_id*3, 3, 3) = -Eigen::Matrix3d::Identity();
      H_obs.block(0, target_id*3, 3, 3) = Eigen::Matrix3d::Identity();

      Eigen::Matrix3d S_res = P_cov_.block(from_id*3, from_id*3, 3, 3) + // res cov matrix
                              P_cov_.block(target_id*3, target_id*3, 3, 3) -
                              P_cov_.block(from_id*3, target_id*3, 3, 3) -
                              P_cov_.block(target_id*3, from_id*3, 3, 3) + R_cov;
      
      Eigen::Matrix<double, Eigen::Dynamic, 3> K(drones_num_ * 3, 3); // Kalman gain
      K = P_cov_ * H_obs.transpose() * S_res.inverse();
      
      Eigen::Vector3d predicted_rela_pos;
      predicted_rela_pos[0] = all_odom_[target_id].pose.pose.position.x + X_error_[target_id*3 + 0]
                                - all_odom_[from_id].pose.pose.position.x - X_error_[from_id*3 + 0];
      predicted_rela_pos[1] = all_odom_[target_id].pose.pose.position.y + X_error_[target_id*3 + 1]
                                - all_odom_[from_id].pose.pose.position.y - X_error_[from_id*3 + 1];
      predicted_rela_pos[2] = all_odom_[target_id].pose.pose.position.z + X_error_[target_id*3 + 2]
                                - all_odom_[from_id].pose.pose.position.z - X_error_[from_id*3 + 2];
      Eigen::Vector3d res = rela_pos - predicted_rela_pos; // rela_pos - predicted_rela_pos，res

      X_error_ = X_error_ + K * res;
      P_cov_ = P_cov_ - K * H_obs * P_cov_;

      // X_error：odom correct target
      quadrotor_msgs::Offset offset_msg;
      offset_msg.from_id = drone_id_;
      for (int id = 1; id < drones_num_; id ++) {
        offset_msg.target_id = id;
        offset_msg.offset[0] = X_error_[id*3 + 0];
        offset_msg.offset[1] = X_error_[id*3 + 1];
        offset_msg.offset[2] = X_error_[id*3 + 2];
        offset_pub_.publish(offset_msg);
      }
    }

  }

  void EGOReplanFSM::offsetShareCallback(const quadrotor_msgs::Offset& msg)
  {
    if(drone_id_ != 0 && msg.from_id == 0 && msg.target_id == drone_id_) {
      X_error_[drone_id_*3 + 0] = msg.offset[0];
      X_error_[drone_id_*3 + 1] = msg.offset[1];
      X_error_[drone_id_*3 + 2] = msg.offset[2];
    }
  }

  void EGOReplanFSM::allOdomCallback(const nav_msgs::Odometry& msg)
  {
    std::string frame_id = msg.child_frame_id;
    int id = frame_id[frame_id.size()-1] - '0';
    all_odom_[id] = msg;
  }

  void EGOReplanFSM::correctOffsetCallback(const ros::TimerEvent &e)
  {
    Eigen::Vector3d target_offset = X_error_.block(drone_id_*3, 0, 3, 1);
    if((target_offset - cur_offset).norm() > 0.02)
    {
      cur_offset += (target_offset - cur_offset).normalized() * 0.008; // 0.8cm / 0.02s
    } else {
      cur_offset = target_offset;
    }
  }

  void EGOReplanFSM::computeYawVel(traj_utils::PolyTraj& poly_traj)
  {
    double yaw_dt = 0.1; 
    double time_forward = 1.5; 
    double yaw;
    double t_cur, t_next, traj_duration;
    int yaw_num = 0;
    auto traj = &planner_manager_->traj_.local_traj.traj;
    poly_traj.yaw_dt = yaw_dt;
    traj_duration = traj->getTotalDuration();
    yaw_num = floor(traj_duration / yaw_dt);
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), next_pos(Eigen::Vector3d::Zero()), dir;
    for(int i = 0; i < yaw_num + 1; ++i) {
        t_cur = i * yaw_dt;
        pos = traj->getPos(t_cur);
        t_next = t_cur + time_forward;
        if (t_next > traj_duration) t_next = traj_duration-0.01;
        next_pos = traj->getPos(t_next);
        dir = next_pos - pos;
        yaw = atan2(dir(1), dir(0));
        poly_traj.yaw_pts.push_back(yaw);
    }
  }

  void EGOReplanFSM::computeYawTurn(traj_utils::PolyTraj& poly_traj) 
  {
    double yaw_dt = 0.1; 
    double time_forward = 1.5; 
    double observe_time = 1.5; // obs time
    constexpr double YAW_DOT_PER_SEC = M_PI / 3; // turn vel
    auto traj = &planner_manager_->traj_.local_traj.traj;
    poly_traj.yaw_dt = yaw_dt;
    double traj_duration = traj->getTotalDuration();
    int yaw_num = floor(traj_duration / yaw_dt);
    // turn
    Eigen::Vector3d origin_dir = traj->getPos(time_forward) - traj->getPos(0);
    double origin_yaw = atan2(origin_dir(1), origin_dir(0));
    double delta_yaw = to_turn_yaw_ - origin_yaw;
    if (delta_yaw > M_PI) delta_yaw -= 2 * M_PI;
    if (delta_yaw < -M_PI) delta_yaw += 2 * M_PI;
    int yaw_turn_num = floor(abs(delta_yaw) / (YAW_DOT_PER_SEC * 0.1));
    double yaw;
    for (int i = 0; i < yaw_turn_num+1; i ++) {
      yaw = origin_yaw +  delta_yaw / abs(delta_yaw) * YAW_DOT_PER_SEC * 0.1 * i;
      if (yaw > M_PI) yaw -= 2 * M_PI;
      if (yaw < -M_PI) yaw += 2 * M_PI;
      poly_traj.yaw_pts.push_back(yaw);
    }
    // obs
    for (int i = 0; i < int(observe_time/0.1); i ++) {
      yaw = to_turn_yaw_;
      poly_traj.yaw_pts.push_back(yaw);
    }
    // back
    for (int i = 0; i < yaw_turn_num+1; i ++) {
      yaw = origin_yaw +  delta_yaw / abs(delta_yaw) * YAW_DOT_PER_SEC * 0.1 * (yaw_turn_num-i);
      if (yaw > M_PI) yaw -= 2 * M_PI;
      if (yaw < -M_PI) yaw += 2 * M_PI;
      poly_traj.yaw_pts.push_back(yaw);
    }
    // follow traj
    double t_cur, t_next;
    Eigen::Vector3d pos, next_pos, dir;
    for (int i = 2*yaw_turn_num+2+int(observe_time/0.1); i < yaw_num+1; i ++) {
      t_cur = i * yaw_dt;
      pos = traj->getPos(t_cur);
      t_next = t_cur + time_forward;
      if (t_next > traj_duration) t_next = traj_duration-0.01;
      next_pos = traj->getPos(t_next);
      dir = next_pos - pos;
      yaw = atan2(dir(1), dir(0));
      poly_traj.yaw_pts.push_back(yaw);
    }
  }

  bool EGOReplanFSM::insideFov(const Eigen::Vector3d& pc, const double& yaw, 
                  const Eigen::Vector3d& pw) {

    Eigen::Vector3d dir = pw - pc;
    double up_down_angle = M_PI * 30 / 180; // FOV updown 60 degree
    double left_right_angle = M_PI * 40 / 180; // FOV leftright 80 degree

    Eigen::Vector3d n_top, n_bottom, n_left, n_right;
    n_top << cos(M_PI_2 - up_down_angle), 0.0, -sin(M_PI_2 - up_down_angle);
    n_bottom << cos(M_PI_2 - up_down_angle), 0.0, sin(M_PI_2 - up_down_angle);
    n_left << cos(M_PI_2 - left_right_angle), -sin(M_PI_2 - left_right_angle), 0.0;
    n_right << cos(M_PI_2 - left_right_angle), sin(M_PI_2 - left_right_angle), 0.0;
    vector<Eigen::Vector3d> normals = { n_top, n_bottom, n_left, n_right };

    Eigen::Matrix3d Rwb;
    Rwb << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    for (auto& n : normals) {
      n = Rwb * n; 
    }
    
    if (dir.norm() > 4.5) { // fov range
      // cout << "out of range!" << endl;
      return false;
    }
    for (auto n : normals) { // fov range
      if (dir.dot(n) < 0.1) {
        // cout << "out of fov!" << endl;
        return false;
      }
    }
    return true;
  }

  void EGOReplanFSM::observeCallback(const ros::TimerEvent &e) 
  {
    // // debug
    // if (drone_id_ == 0) {
    //   cout << drone_id_ << " : P_cov_: " << P_cov_(0,0) << ", " << P_cov_(1,1) << ", " << P_cov_(2,2) << ", "
    //                   << P_cov_(3,3) << ", " << P_cov_(4,4) << ", " << P_cov_(5,5) << ", "
    //                   << P_cov_(6,6) << ", " << P_cov_(7,7) << ", " << P_cov_(8,8) << endl;
    //   Eigen::Matrix3d rela_P01_cov = P_cov_.block(0,0,3,3) + P_cov_.block(3,3,3,3) - 
    //                                   P_cov_.block(0,3,3,3) - P_cov_.block(3,0,3,3);
    //   cout << drone_id_ << " : P01_cov_: " << rela_P01_cov(0,0) << ", " << rela_P01_cov(1,1) << ", " 
    //                                             << rela_P01_cov(2,2) << endl;
    //   Eigen::Matrix3d rela_P02_cov = P_cov_.block(0,0,3,3) + P_cov_.block(6,6,3,3) - 
    //                                   P_cov_.block(0,6,3,3) - P_cov_.block(6,0,3,3);
    //   cout << drone_id_ << " : P02_cov_: " << rela_P02_cov(0,0) << ", " << rela_P02_cov(1,1) << ", " 
    //                                             << rela_P02_cov(2,2) << endl;
    //   Eigen::Matrix3d rela_P12_cov = P_cov_.block(3,3,3,3) + P_cov_.block(6,6,3,3) - 
    //                                   P_cov_.block(3,6,3,3) - P_cov_.block(6,3,3,3);
    //   cout << drone_id_ << " : P12_cov_: " << rela_P12_cov(0,0) << ", " << rela_P12_cov(1,1) << ", " 
    //                                             << rela_P12_cov(2,2) << endl;
    //   cout << drone_id_ << " : X_error_: " << X_error_[0] << ", " << X_error_[1] << ", " << 
    //           X_error_[2] << ", " << X_error_[3] << ", " << X_error_[4] << ", " << X_error_[5] << 
    //           ", " << X_error_[6] << ", " << X_error_[7] << ", " << X_error_[8] << endl;
    // }

    static std::random_device rd;
    static std::default_random_engine re(rd());
    static std::normal_distribution<double> x_d(0, 0.3);
    static std::normal_distribution<double> y_d(0, 0.3);
    static std::normal_distribution<double> z_d(0, 0.3);
    Eigen::Vector3d pw, pc;
    double yaw, x, y, z, w;
    pc << all_real_pos_[drone_id_].pose.pose.position.x, all_real_pos_[drone_id_].pose.pose.position.y,
                                    all_real_pos_[drone_id_].pose.pose.position.z;
    x = all_real_pos_[drone_id_].pose.pose.orientation.x;
    y = all_real_pos_[drone_id_].pose.pose.orientation.y;
    z = all_real_pos_[drone_id_].pose.pose.orientation.z;
    w = all_real_pos_[drone_id_].pose.pose.orientation.w;
    yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));

    static std::map<int, int> obs_count;
    static std::map<int, bool> last_view;
    for (int id = 0; id < drones_num_; id++) {
      if (id == drone_id_) continue;
      pw << all_real_pos_[id].pose.pose.position.x, all_real_pos_[id].pose.pose.position.y,
                                    all_real_pos_[id].pose.pose.position.z;
      if (insideFov(pc, yaw, pw)) {
        //  if the same drone has been within the FOV for six consecutive frames, it is considered detectable by YOLO, 
        //  and the observations are fed to the EKF for update
        if (last_view[id]) {
          if (obs_count[id] > 5) { 
            
            quadrotor_msgs::YoloObs msg;
            msg.from_id = drone_id_;
            msg.target_id = id;
            msg.rela_pos.x = all_real_pos_[id].pose.pose.position.x -
                      all_real_pos_[drone_id_].pose.pose.position.x + x_d(re);
            msg.rela_pos.y = all_real_pos_[id].pose.pose.position.y -
                      all_real_pos_[drone_id_].pose.pose.position.y + y_d(re);
            msg.rela_pos.z = all_real_pos_[id].pose.pose.position.z -
                      all_real_pos_[drone_id_].pose.pose.position.z + z_d(re);
            msg.ekfP[0] = 0.2; msg.ekfP[1] = 0; msg.ekfP[2] = 0;
            msg.ekfP[3] = 0; msg.ekfP[4] = 0.2; msg.ekfP[5] = 0;
            msg.ekfP[6] = 0; msg.ekfP[7] = 0; msg.ekfP[8] = 0.2;
            obs_share_pub_.publish(msg);

            if (obs_count[id] < 30) { 
              obs_count[id] ++;
            }

          } else {
            obs_count[id] ++;
          }
        }

        last_view[id] = true;
      } else {
        last_view[id] = false;
        obs_count[id] = 0;
      }
    }
  }

  void EGOReplanFSM::getFOV(vector<Eigen::Vector3d>& list1, 
              vector<Eigen::Vector3d>& list2, Eigen::Vector3d pos, double yaw) {
    list1.clear();
    list2.clear();

    // Get info for visualizing FOV at (pos, yaw)
    Eigen::Matrix3d Rwb;
    Rwb << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    for (long unsigned int i = 0; i < cam_vertices1_.size(); ++i) {
      auto p1 = Rwb * cam_vertices1_[i] + pos;
      auto p2 = Rwb * cam_vertices2_[i] + pos;
      list1.push_back(p1);
      list2.push_back(p2);
    }
  }

  void EGOReplanFSM::drawFOV(const vector<Eigen::Vector3d>& list1, 
                              const vector<Eigen::Vector3d>& list2) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.04;
    mk.scale.y = 0.04;
    mk.scale.z = 0.04;

    // Clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    fov_vis_pub.publish(mk);

    if (list1.size() == 0) return;

    // Pub new marker
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
    mk.action = visualization_msgs::Marker::ADD;
    fov_vis_pub.publish(mk);
  }

  void EGOReplanFSM::getRela(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2, 
                                                int id1, int id2, double cov_trace)
  {
    if (cov_trace < obs_cov_thresh_) return ;
    Eigen::Vector3d pos1, pos2;
    pos1 << all_real_pos_[id1].pose.pose.position.x, all_real_pos_[id1].pose.pose.position.y,
                                                  all_real_pos_[id1].pose.pose.position.z;
    pos2 << all_real_pos_[id2].pose.pose.position.x, all_real_pos_[id2].pose.pose.position.y,
                                                  all_real_pos_[id2].pose.pose.position.z;
    if ((pos1-pos2).norm() > 4.5) return ;
    list1.push_back(pos1);
    list2.push_back(pos2);
  }

  void EGOReplanFSM::drawRela(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.ns = "relative_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;
    mk.color.a = 0.6;
    mk.scale.x = 0.04;
    mk.scale.y = 0.04;
    mk.scale.z = 0.04;

    // Clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    rela_vis_pub_.publish(mk);

    if (list1.size() == 0) return;

    // Pub new marker
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1.size()); ++i) {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
    mk.action = visualization_msgs::Marker::ADD;
    fov_vis_pub.publish(mk);
  }

  void EGOReplanFSM::decideObsCallback(const ros::TimerEvent &e) 
  {
    vector<Eigen::Vector3d> l1, l2;
    cout << "trace : ";
    for (int i = 0; i < drones_num_; i ++) {
      for (int j = i + 1; j < drones_num_; j ++) {
        Eigen::Matrix3d rela_Pij_cov = P_cov_.block(i*3,i*3,3,3) + P_cov_.block(j*3,j*3,3,3) - 
                                        P_cov_.block(i*3,j*3,3,3) - P_cov_.block(j*3,i*3,3,3);
        double Pij_trace = rela_Pij_cov.trace();
        if (i == drones_num_-2 && j == drones_num_-1)  {
          cout << Pij_trace << endl;
        } else {
          cout << Pij_trace << ", ";
        }
        if (active_obs_) {
          decideDoObs(i, j, Pij_trace);

          if (doobs_is_published_[i*10 + j] && 
              (ros::Time::now() - last_obs_time_[i*10 + j]).toSec() >= drone_busy_duaration_) {
            doobs_is_published_[i*10 + j] = false;
          }

          getRela(l1, l2, i, j, Pij_trace);
        }
      }
    } 

    drawRela(l1, l2);
    
    if (active_obs_) {
      for (int id = 0; id < drones_num_; id ++) {
        if (drone_is_busy_[id] && 
            (ros::Time::now() - last_busy_time_[id]).toSec() >= drone_busy_duaration_) {
          drone_is_busy_[id] = false;
          // cout << "drone" << id << " is reset to false!" << endl;
        }
      }
    }

  }

  void EGOReplanFSM::decideDoObs(int id1, int id2, double cov_trace) {
    if (cov_trace < obs_cov_thresh_) return ;
    Eigen::Vector3d dir_id1_id2, id1_odom, id2_odom;
    double id1_yaw, id2_yaw, x, y, z, w;
    x = all_odom_[id1].pose.pose.orientation.x;
    y = all_odom_[id1].pose.pose.orientation.y;
    z = all_odom_[id1].pose.pose.orientation.z;
    w = all_odom_[id1].pose.pose.orientation.w;
    id1_yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
    x = all_odom_[id2].pose.pose.orientation.x;
    y = all_odom_[id2].pose.pose.orientation.y;
    z = all_odom_[id2].pose.pose.orientation.z;
    w = all_odom_[id2].pose.pose.orientation.w;
    id2_yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z));
    id1_odom << all_odom_[id1].pose.pose.position.x + X_error_[id1*3 + 0],
                all_odom_[id1].pose.pose.position.y + X_error_[id1*3 + 1],
                all_odom_[id1].pose.pose.position.z + X_error_[id1*3 + 2];
    id2_odom << all_odom_[id2].pose.pose.position.x + X_error_[id2*3 + 0],
                all_odom_[id2].pose.pose.position.y + X_error_[id2*3 + 1],
                all_odom_[id2].pose.pose.position.z + X_error_[id2*3 + 2];

    dir_id1_id2 = id2_odom - id1_odom; 
    // cout << "dir_" << id1 << id2 << " : " << dir_id1_id2[0] << ", " << dir_id1_id2[1] << ", "
    //                                       << dir_id1_id2[2] << endl;
    if (dir_id1_id2.norm() > 4.5) return ;
    double dir_id1_id2_yaw = atan2(dir_id1_id2[1], dir_id1_id2[0]);
    double dir_id2_id1_yaw = atan2(-dir_id1_id2[1], -dir_id1_id2[0]);
    // cout << "dir_id1_id2_yaw : " << dir_id1_id2_yaw << endl;
    // cout << "dir_id2_id1_yaw : " << dir_id2_id1_yaw << endl;
    // cout << "id1_yaw : " << id1_yaw << endl;
    // cout << "id2_yaw : " << id2_yaw << endl;
    if (doobs_is_published_[id1*10 + id2]) return ;
    if (abs(id1_yaw - dir_id1_id2_yaw) < M_PI_2*1.2 && !drone_is_busy_[id1]) {
      quadrotor_msgs::DoObs msg;
      msg.from_id = id1;
      msg.target_id = id2;
      msg.yaw = dir_id1_id2_yaw;
      do_obs_pub_.publish(msg);
      drone_is_busy_[id1] = true;
      doobs_is_published_[id1*10 + id2] = true;
      last_busy_time_[id1] = ros::Time::now();
      last_obs_time_[id1*10 + id2] = ros::Time::now();
      return ;
    } else if (abs(id2_yaw - dir_id2_id1_yaw) < M_PI_2*1.2 && !drone_is_busy_[id2]) {
      quadrotor_msgs::DoObs msg;
      msg.from_id = id2;
      msg.target_id = id1;
      msg.yaw = dir_id2_id1_yaw;
      do_obs_pub_.publish(msg);
      drone_is_busy_[id2] = true;
      doobs_is_published_[id1*10 + id2] = true;
      last_busy_time_[id2] = ros::Time::now();
      last_obs_time_[id1*10 + id2] = ros::Time::now();
      return ;
    }
  }

  void EGOReplanFSM::doObsCallback(const quadrotor_msgs::DoObs& msg) {
    if (drone_id_ != msg.from_id) return ;
    while (exec_state_ != EXEC_TRAJ)
    {
      ros::spinOnce();
      ros::Duration(0.001).sleep();
    }
    to_turn_yaw_ = msg.yaw;
    to_turn_ = true;
    cout << drone_id_ << " : to observe " << msg.target_id << ", yaw turn " 
                                          << msg.yaw / M_PI * 180 << endl; 
    changeFSMExecState(REPLAN_TRAJ, "TURN_TRIG");
  }

  void EGOReplanFSM::rpeCallback(const ros::TimerEvent &e)
  {
    if (drone_id_ != 0) return ;
    double rpe = 0.0;
    Eigen::Vector3d predicted_rela_pos;
    Eigen::Vector3d real_rela_pos;
    for (int i = 0; i < drones_num_; i ++) {
      for (int j = i + 1; j < drones_num_; j ++) {
        predicted_rela_pos[0] = all_odom_[j].pose.pose.position.x + X_error_[j*3 + 0]
                              - all_odom_[i].pose.pose.position.x - X_error_[i*3 + 0];
        predicted_rela_pos[1] = all_odom_[j].pose.pose.position.y + X_error_[j*3 + 1]
                              - all_odom_[i].pose.pose.position.y - X_error_[i*3 + 1];
        predicted_rela_pos[2] = all_odom_[j].pose.pose.position.z + X_error_[j*3 + 2]
                              - all_odom_[i].pose.pose.position.z - X_error_[i*3 + 2];
        real_rela_pos[0] = all_real_pos_[j].pose.pose.position.x - all_real_pos_[i].pose.pose.position.x;
        real_rela_pos[1] = all_real_pos_[j].pose.pose.position.y - all_real_pos_[i].pose.pose.position.y;
        real_rela_pos[2] = all_real_pos_[j].pose.pose.position.z - all_real_pos_[i].pose.pose.position.z;
        rpe += (predicted_rela_pos - real_rela_pos).norm();
      }
    }
    rpe = rpe / (drones_num_*(drones_num_-1)/2);
    cout << "rpe : " << rpe << endl;
  }
  /*------------------------------new code end-------------------------------*/
} // namespace ego_planner
