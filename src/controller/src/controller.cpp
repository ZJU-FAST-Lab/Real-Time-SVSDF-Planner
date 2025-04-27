// controller.cpp

#include "controller/controller.hpp"
#include <iostream>

// 假设 Bspline 类已经实现，包含 fromMessage 和 evaluateDeBoor 等方法

Controller::Controller(ros::NodeHandle& nh) : nh_(nh), emergency_stop_(false) {

    std::shared_ptr<Bspline> trajectory_ = nullptr;
    std::shared_ptr<Bspline> new_trajectory_ = nullptr;

    // 初始化订阅者
    traj_sub_ = nh_.subscribe("/planner/trajectory", 10, &Controller::trajectoryCallback, this);
    emergency_stop_sub_ = nh_.subscribe("/planner/emergency_stop", 10, &Controller::emergencyStopCallback, this);

    // 初始化发布者
    pose_pub_ = nh_.advertise<robotstatemsgs::robotState>("/controller/pose", 1);

    // 初始化定时器（200 Hz）
    timer_ = nh_.createTimer(ros::Duration(0.005), &Controller::timerCallback, this);

    nh_.param<double>("start_x",start_x_,-6.8);
    nh_.param<double>("start_y",start_y_,-6.1);
    nh_.param<double>("start_yaw",start_yaw_,0);

    ROS_INFO("Controller node initialized.");
}

Controller::~Controller() {}

void Controller::trajectoryCallback(const robotstatemsgs::robotTrajectory::ConstPtr& msg) {
    ROS_INFO("Received new BSplineTrajectory message.");

    // 创建一个新的 Bspline 对象并从消息中填充
    std::shared_ptr<Bspline> bspline_ptr = std::make_shared<Bspline>(msg);

    if (!trajectory_) {
        // 如果当前轨迹未赋值，直接赋值
        trajectory_ = bspline_ptr;
        start_time_ = msg->traj_start_time;
        // ROS_INFO("Set initial trajectory.");
    }
    else {
        // 否则，赋值给 new_trajectory_
        new_trajectory_ = bspline_ptr;
        new_trajectory_start_time_ = msg->traj_start_time;
        // ROS_INFO("Set new trajectory.");
    }
}

void Controller::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        if (!emergency_stop_) {
            ROS_WARN("Emergency stop activated. Stopping timer.");
            emergency_stop_ = true;
            timer_.stop();
        }
    }
}

void Controller::timerCallback(const ros::TimerEvent& event) {
    if (emergency_stop_) {
        // 如果紧急停止激活，不进行任何操作
        return;
    }

    ros::Time current_time = ros::Time::now();

    // 检查是否需要切换到新的轨迹
    if (new_trajectory_ && current_time >= new_trajectory_start_time_) {
        trajectory_ = new_trajectory_;
        start_time_ = new_trajectory_start_time_;
        new_trajectory_.reset();
        ROS_INFO("Switched to new trajectory.");
    }

    if (trajectory_) {
        // 计算经过的时间
        double t = (current_time - start_time_).toSec();
        double duration = trajectory_->getTotalDuration();
        if(t>duration)
            return;
        // 评估当前轨迹的位置
        Eigen::Vector3d state = trajectory_->getPos(t); // 假设 evaluateDeBoor 方法已实现
        Eigen::Vector3d velocity = trajectory_->getVel(t);
        Eigen::Vector3d acceleration = trajectory_->getAcc(t); 

        // 创建 robotState 消息
        robotstatemsgs::robotState pose_msg;
        pose_msg.Header.stamp = current_time;
        pose_msg.Header.frame_id = "world"; // 根据需要设置框架

        // 假设 state 包含 (x, y, yaw)
        pose_msg.x = state[0];
        pose_msg.y = state[1];
        pose_msg.yaw = state[2];
        
        pose_msg.vx = velocity[0];
        pose_msg.vy = velocity[1];
        pose_msg.omega = velocity[2];
        
        pose_msg.ax = acceleration[0];
        pose_msg.ay = acceleration[1];
        pose_msg.alpha = acceleration[2];

        // 发布消息
        pose_pub_.publish(pose_msg);
    }
    else
    {   // 初始化时候没有轨迹，处在原点
        robotstatemsgs::robotState pose_msg;
        pose_msg.Header.stamp = current_time;
        pose_msg.Header.frame_id = "world"; // 根据需要设置框架

        // 假设 state 包含 (x, y, yaw)
        pose_msg.x = start_x_;
        pose_msg.y = start_y_;
        pose_msg.yaw = start_yaw_;
        
        pose_msg.vx = 0.0;
        pose_msg.vy = 0.0;
        pose_msg.omega = 0.0;
        
        pose_msg.ax = 0.0;
        pose_msg.ay = 0.0;
        pose_msg.alpha = 0.0;

        // 发布消息
        pose_pub_.publish(pose_msg);
    }
}
