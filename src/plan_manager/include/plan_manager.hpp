#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include <ros/ros.h>
#include "plan_env/sdf_map.h"
#include "utils/NURBS.hpp"
#include "utils/shape.hpp"
#include "utils/visualization.hpp"
#include <geometry_msgs/PoseStamped.h>
#include "robotstatemsgs/robotState.h"
#include "robotstatemsgs/robotTrajectory.h"
#include "swept_volume/swept_volume_manager.hpp"
#include "planner_algorithm/front_end_astar.hpp"
#include "planner_algorithm/BSpline_optimizer.hpp"
#include <thread>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <utils/debug_publisher.hpp>

enum StateMachine{
  INIT,
  IDLE,
  PLANNING,
  REPLAN,
  GOINGTOGOAL,
  EMERGENCY_STOP,
};

class PlanManager
{
  public:
    ros::NodeHandle nh_;

    int shape_number_;
    std::shared_ptr<SDFmap> sdfmap_;
    std::shared_ptr<BasicShape> shape_;
    std::shared_ptr<sv_manager> sv_manager_;
    std::shared_ptr<vis::Visualization> visualizer_;

    std::shared_ptr<AstarPathSearcher> astar_searcher_;
    std::shared_ptr<TrajOptimizer> traj_optimizer_;

    // Timer
    ros::Timer main_thread_timer_;
    // Publisher
    ros::Publisher emergency_stop_pub_;
    ros::Publisher trajectory_pub_;
    // Subsriber
    ros::Subscriber goal_sub_;
    ros::Subscriber current_state_sub;

    ros::Time current_time_;
    Eigen::Vector3d current_state_XYTheta_P_;
    Eigen::Vector3d current_state_XYTheta_V_;
    Eigen::Vector3d current_state_XYTheta_A_;

    double plan_start_time_;
    Eigen::Vector3d plan_start_state_XYTheta_P_;
    Eigen::Vector3d plan_start_state_XYTheta_V_;
    Eigen::Vector3d plan_start_state_XYTheta_A_;


    std::vector<Eigen::Vector3d> front_end_path_;
    Eigen::Vector3d goal_state_;

    bool have_goal_ = false;
    bool have_geometry_ = false;
    StateMachine state_machine_ = StateMachine::IDLE;

    double replan_time_;
    double max_replan_time_;

    ros::Time Traj_start_time_;
    double Traj_total_time_;

    ros::Time loop_start_time_;

    double predicted_traj_start_time_;

    // Timer启动的Thread
    void MainThread(const ros::TimerEvent& event);

    bool is_debug_grad_ = false;
    bool method = false;
    bool fixed = true;
    bool mode = true;
    
  public:
    PlanManager(ros::NodeHandle &nh);
    ~PlanManager(); 

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void geometry_callback(const robotstatemsgs::robotState::ConstPtr &msg);

    // 实用工具
    void cout_plan_information();
    void set_start_and_end_derivative(std::vector<Eigen::Vector3d> &start_and_end_derivative);
};


#endif