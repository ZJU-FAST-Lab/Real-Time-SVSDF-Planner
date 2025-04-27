// controller.hpp

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ros/ros.h>
#include <robotstatemsgs/robotTrajectory.h>
#include <robotstatemsgs/robotState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "utils/NURBS.hpp" // 假设 Bspline 类已实现

class Controller {
public:
    Controller(ros::NodeHandle& nh);
    double start_x_;
    double start_y_;
    double start_yaw_;
    ~Controller();

private:
    void trajectoryCallback(const robotstatemsgs::robotTrajectory::ConstPtr& msg);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;

    ros::Subscriber traj_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Publisher pose_pub_;

    ros::Timer timer_;

    std::shared_ptr<Bspline> trajectory_;
    std::shared_ptr<Bspline> new_trajectory_;

    ros::Time start_time_;
    ros::Time new_trajectory_start_time_;

    bool emergency_stop_;
};

#endif // CONTROLLER_HPP
