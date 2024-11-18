#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

/*--------------------------new code start------------------------*/
#include <quadrotor_msgs/YoloObs.h>
#include <quadrotor_msgs/Covariance.h>
#include <quadrotor_msgs/Offset.h>
#include <quadrotor_msgs/DoObs.h>
#include <random>
/*---------------------------new code end--------------------------*/

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int form_num_;
    Eigen::MatrixXd formation_;
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool flag_escape_emergency_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d end_pt_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;

    Eigen::Vector3d formation_start_;
    // int formation_num_;
    Eigen::Vector3d formation_pos_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

    /*--------------------------new code start------------------------*/
    int drone_id_; // drone id
    int drones_num_; // drone num

    // ESKF
    bool eskf_update_; // enable ESKF update
    Eigen::VectorXd X_error_; // ESKF error state 1*(drones_num_*3)
    Eigen::MatrixXd P_cov_; // ESKF cov (drones_num_*3)*(drones_num_*3)
    Eigen::MatrixXd F_sys_; // system matrix (drones_num_*3)*(drones_num_*3)
    std::vector<ros::Subscriber> all_odom_sub_; // sub all odometry not corrected by ESKF
    std::map<int, nav_msgs::Odometry> all_odom_; // save all odometry information not corrected by ESKF
    ros::Publisher offset_pub_; // master drone send updated X_error information to slave drone
    ros::Subscriber offset_sub_; // slave drone sub updated X_error information
    Eigen::Vector3d cur_offset; // current correction offset on odom
    ros::Timer offset_timer_; //  gradually correct odom

    // fake yolo相关
    ros::Timer observe_timer_; // fake_yolo
    std::vector<ros::Subscriber> all_real_pos_sub_; // real pos of all drone
    std::map<int, nav_msgs::Odometry> all_real_pos_; 
    ros::Publisher obs_share_pub_; // pub observation message
    ros::Subscriber obs_share_sub_; // sub observation message

    // 观测相关
    ros::Timer do_obs_timer_; // check uncertainty, trigger obs
    ros::Publisher do_obs_pub_; // pub doobs event
    ros::Subscriber do_obs_sub_; // do obs
    double drone_busy_duaration_; // busy time
    std::map<int, bool> drone_is_busy_; 
    std::map<int, bool> doobs_is_published_; 
    std::map<int, ros::Time> last_busy_time_; 
    std::map<int, ros::Time> last_obs_time_;
    bool to_turn_ = false; // to obs
    double to_turn_yaw_;
    double obs_cov_thresh_; // thresh of cov to do obs
    bool active_obs_; // if active obs

    // 其他
    double x_noise_step_, y_noise_step_, z_noise_step_; // noise of odom
    ros::Publisher odom_corrected_pub_; // pub odom corrected by ESKF
    ros::Publisher fov_vis_pub; // FOV vis
    vector<Eigen::Vector3d> cam_vertices1_, cam_vertices2_; // for vis
    bool master_no_noise_; // if odom of master drone no noise 
    ros::Publisher rela_vis_pub_; // pub relative cov
    ros::Timer rpe_timer_; // caculate relative pos error
    /*--------------------------new code end------------------------*/

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    // void planGlobalTrajbyGivenWps();
    void readGivenWpsAndPlan();
    void planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp);

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);

    /*------------------------new code start--------------------------*/
    void realPosCallback(const nav_msgs::Odometry& msg); 
    void observeCallback(const ros::TimerEvent &e); 
    void obsShareCallback(const quadrotor_msgs::YoloObs& msg);
    void allOdomCallback(const nav_msgs::Odometry& msg); 
    void offsetShareCallback(const quadrotor_msgs::Offset& msg);
    void correctOffsetCallback(const ros::TimerEvent &e);
    void decideObsCallback(const ros::TimerEvent &e);
    void doObsCallback(const quadrotor_msgs::DoObs& msg);

    void computeYawVel(traj_utils::PolyTraj& poly_traj); // yaw planning
    void computeYawTurn(traj_utils::PolyTraj& poly_traj); 
    // if other drone inside FOV
    bool insideFov(const Eigen::Vector3d& pw, const double& yaw, const Eigen::Vector3d& pc); 
    // for FOV vis
    void getFOV(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2, 
                                              Eigen::Vector3d pos, double yaw);
    void drawFOV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2);
    // for relative cov vis
    void getRela(vector<Eigen::Vector3d>& list1, vector<Eigen::Vector3d>& list2, 
                                            int id1, int id2, double cov_trace);
    void drawRela(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2);
    // decide if do obs
    void decideDoObs(int id1, int id2, double cov_trace);
    void rpeCallback(const ros::TimerEvent &e);
    /*------------------------new code end--------------------------*/
  };

} // namespace ego_planner

#endif