#include "plan_manager.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include <tf/transform_datatypes.h> 
#include "robotstatemsgs/robotTrajectory.h"

PlanManager::PlanManager(ros::NodeHandle &nh): nh_(nh)
{
    nh_.param<int>("shape",shape_number_,0);
    switch (shape_number_)
    {
    case 1:
      shape_ = std::make_shared<FShape>(nh_);
      break;
    case 2:
      shape_ = std::make_shared<AShape>(nh_);
      break;
    case 3:
      shape_ = std::make_shared<SShape>(nh_);
      break;
    case 4:
      shape_ = std::make_shared<TShape>(nh_);
      break;
    default:
      shape_ = std::make_shared<ArcShape>(nh_);
      break;
    }
    sdfmap_ = std::make_shared<SDFmap>(nh);
    sv_manager_ = std::make_shared<sv_manager>(sdfmap_ , shape_);
    visualizer_ = std::make_shared<vis::Visualization>(nh);

    astar_searcher_ = std::make_shared<AstarPathSearcher>(nh_, sdfmap_, sv_manager_);
    traj_optimizer_ = std::make_shared<TrajOptimizer>(nh, sv_manager_, visualizer_);
    
    // Timer
    int thread_number = 0;
    nh_.param<int>("thread_number",thread_number,0);
    main_thread_timer_ = nh_.createTimer(ros::Duration(0.01),&PlanManager::MainThread, this);

    // Publisher
    emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/planner/emergency_stop",1);
    trajectory_pub_ = nh_.advertise<robotstatemsgs::robotTrajectory>("/planner/trajectory",1);

    // Subsriber
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PlanManager::goal_callback, this);
    current_state_sub = nh_.subscribe("/simulation/pose",1,&PlanManager::geometry_callback,this);
    
    nh_.param<double>("replan_time",replan_time_,0.08);
    nh_.param<double>("max_replan_time", max_replan_time_, 0.05);
    nh_.param<bool>("fixed", fixed, true);

    loop_start_time_ = ros::Time::now();
    goal_state_ << 0,0,0;
}

PlanManager::~PlanManager(){}


///////////////////// thread    //////////////////////////
void PlanManager::MainThread(const ros::TimerEvent& event)
{
    if(!have_geometry_ || !have_goal_)
    {
        return;
    }

    if(state_machine_ == StateMachine::IDLE ||
      (  (state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN)
       &&(ros::Time::now() - loop_start_time_).toSec()>replan_time_                         ))
    {
        loop_start_time_ = ros::Time::now();
        double current = loop_start_time_.toSec();
        // start new plan
        if(state_machine_ == StateMachine::IDLE)
        {
            state_machine_ = StateMachine::PLANNING;
            // initialize
            plan_start_time_ = -1;
            predicted_traj_start_time_ = -1;
            plan_start_state_XYTheta_P_ = current_state_XYTheta_P_;
            plan_start_state_XYTheta_V_ = current_state_XYTheta_V_;
            plan_start_state_XYTheta_A_ = current_state_XYTheta_A_;
        }
        else if(state_machine_ == StateMachine::PLANNING || 
                state_machine_ == StateMachine::REPLAN)
        {
            // 接近目标时，不需要再重规划，直接前进，return
            if((current_state_XYTheta_P_ - goal_state_).head(2).squaredNorm() < 1.0 ||
                traj_optimizer_->trajectory_->getTotalDuration() < max_replan_time_)
            {
                state_machine_ = StateMachine::GOINGTOGOAL;
                return;
            }
            // 需要重规划了
            state_machine_ = StateMachine::REPLAN;
            predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
            
            traj_optimizer_->get_the_predicted_state(predicted_traj_start_time_, 
                                                    plan_start_state_XYTheta_P_,
                                                    plan_start_state_XYTheta_V_,
                                                    plan_start_state_XYTheta_A_);
            // 这里没有return，会接着执行
        }
        ROS_INFO("\033[0;30;43m start new plan \033[0m");
        // cout_plan_information();

        //front end
        ros::Time astar_start_time = ros::Time::now();
        astar_searcher_->AstarPathSearch(plan_start_state_XYTheta_P_.head(2), 
                                        goal_state_.head(2),
                                        plan_start_state_XYTheta_P_[2]);
        if(!astar_searcher_->success_flag)
        {
            state_machine_ = REPLAN;
            ROS_ERROR("EMERGENCY_STOP!!! can not find astar road !!!");
            return;
        }
        std::vector<Eigen::Vector3d> waypoints;
        std::vector<Eigen::Vector2d> collision_points;
        
        astar_searcher_->getPathAndCollisionPoints(waypoints, collision_points);
        ROS_INFO("\033[0;30;46m all of front end time:%f \033[0m", 
                  (ros::Time::now()-astar_start_time).toSec());
        visualizer_->front_end_visualization("Robot", waypoints);
        visualizer_->visualize_points("Collision_points",collision_points);
    
        // mid end
        std::vector<Eigen::Vector3d> start_and_end_derivative;
        set_start_and_end_derivative(start_and_end_derivative);
        traj_optimizer_->InitializeBspline(waypoints, start_and_end_derivative, collision_points);
        traj_optimizer_->optimize_lbfgs();
        traj_optimizer_->Time_ReAllocate();
        traj_optimizer_->visTrajectory();
        
        ROS_INFO("\033[0;30;46m all of plan time:%f \033[0m", 
                  (ros::Time::now().toSec()-current));
        
        // 发布轨迹
        if(plan_start_time_ < 0){
          Traj_start_time_ = ros::Time::now();
          plan_start_time_ = Traj_start_time_.toSec();
        }
        else{
          plan_start_time_ = current + max_replan_time_;
          Traj_start_time_ = ros::Time(plan_start_time_);
        }
        robotstatemsgs::robotTrajectory trajectory_msgs = traj_optimizer_->trajectory_->getRobotTrajectoryMsg(Traj_start_time_);
        trajectory_pub_.publish(trajectory_msgs);
        Traj_total_time_ = traj_optimizer_->trajectory_->getTotalDuration();
    }

    if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_)
    {
        state_machine_ = StateMachine::IDLE;
        have_goal_ = false;
    }
}


///////////////////// callback  //////////////////////////
void PlanManager::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::cout<<"get goal"<<std::endl;
    // 记录目标点
    goal_state_(0) = msg->pose.position.x; 
    goal_state_(1) = msg->pose.position.y; 
    goal_state_(2) = tf::getYaw(msg->pose.orientation);
    if(fixed)
    {
      if(mode)
      {
      goal_state_(0) = -2.26;
      goal_state_(1) = 7.76;
      goal_state_(2) = 0.0;
      mode = false;
      }
      else{
      goal_state_(0) = -6.8;
      goal_state_(1) = -6.1;
      goal_state_(2) = 0.0;
      mode = true;
      }
    }

    have_goal_ = true;
}

void PlanManager::geometry_callback(const robotstatemsgs::robotState::ConstPtr &msg)
{
    // ROS_INFO("geometry");
    have_geometry_ = true;
    current_state_XYTheta_P_ << msg->x, msg->y, msg->yaw;
    current_state_XYTheta_V_ << msg->vx, msg->vy, msg->omega;
    current_state_XYTheta_A_ << msg->ax, msg->ay, msg->alpha;
    current_time_ = msg->Header.stamp;
}

///////////////////// 实用工具  ////////////////////////////
void PlanManager::cout_plan_information()
{
    ROS_INFO("\033[1;3;33m plan_start_state_P_: \033[0m %.10f %.10f %.10f ", 
                  plan_start_state_XYTheta_P_(0), 
                  plan_start_state_XYTheta_P_(1), 
                  plan_start_state_XYTheta_P_(2));
    ROS_INFO("\033[1;3;33m plan_start_state_V_: \033[0m %.10f %.10f %.10f ", 
                  plan_start_state_XYTheta_V_(0), 
                  plan_start_state_XYTheta_V_(1), 
                  plan_start_state_XYTheta_V_(2));
    ROS_INFO("\033[1;3;33m plan_start_state_A_: \033[0m %.10f %.10f %.10f ", 
                  plan_start_state_XYTheta_A_(0), 
                  plan_start_state_XYTheta_A_(1), 
                  plan_start_state_XYTheta_A_(2));
    ROS_INFO("\033[1;3;33m goal_state_: \033[0m %.10f %.10f %.10f ", 
                  goal_state_(0), 
                  goal_state_(1), 
                  goal_state_(2));
}

void PlanManager::set_start_and_end_derivative(std::vector<Eigen::Vector3d> &start_and_end_derivative)
{
    start_and_end_derivative.reserve(4);
    start_and_end_derivative.push_back(plan_start_state_XYTheta_V_);
    start_and_end_derivative.emplace_back(Eigen::Vector3d::Zero());
    start_and_end_derivative.push_back(plan_start_state_XYTheta_A_);
    start_and_end_derivative.emplace_back(Eigen::Vector3d::Zero());
}
