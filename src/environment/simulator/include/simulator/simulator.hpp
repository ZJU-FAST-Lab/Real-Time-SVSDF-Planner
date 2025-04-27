#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <robotstatemsgs/robotState.h>
#include <tf/tf.h>
#include <cmath>
#include <Eigen/Dense> 
#include <string>




class Simulator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;   //订阅一个根据轨迹发布state的话题
  ros::Publisher marker_pub_;
  ros::Publisher state_pub_;

  visualization_msgs::Marker robot_marker_;

  
  
  Eigen::Matrix2d R;
  int shape_ = 0;
  // 需要对以下三个参数重置
  Eigen::Vector2d offset_;
  std::string mesh_resource_ = "package://simulator/urdf/Arc.dae";
  double scale_ = 0.0005*0.8;

public:
  // 构造函数
  Simulator(ros::NodeHandle &nh);
  void PoseCallback(const robotstatemsgs::robotState::ConstPtr &msg); // 回调函数：接收 x, y, yaw 后发布 Marker 可视化机器人
  void robot_marker_initialize(visualization_msgs::Marker &robot_marker);

};


///////////////////////////////////////////////////////////////////////////////////////////////

Simulator::Simulator(ros::NodeHandle &nh)
{
  nh_ = nh;

  state_sub_ = nh_.subscribe("/controller/pose", 1, &Simulator::PoseCallback, this);
  state_pub_ = nh_.advertise<robotstatemsgs::robotState>("/simulation/pose", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/vis/robot_pose", 1);
  
  nh_.param<int>("shape",shape_,0);
  std::cout<<"shape_:"<<shape_<<std::endl;
  switch (shape_)
  {
  case 1:// F
    offset_<<-0.001*940.37, -0.001*1255;
    mesh_resource_ = "package://simulator/urdf/F.dae";
    scale_ = 0.001;
    break;
  case 2:// A
    offset_<<-0.001*800, -0.001*1155;
    mesh_resource_ = "package://simulator/urdf/A.dae";
    scale_ = 0.001;
    break;
  case 3:// S
    offset_<<-0.001*1050, -0.001*1245;
    mesh_resource_ = "package://simulator/urdf/S.dae";
    scale_ = 0.001;
    break;
  case 4:// T
    offset_<<-0.001*600, -0.001*1000;
    mesh_resource_ = "package://simulator/urdf/T.dae";
    scale_ = 0.001;
    break;
  default:
    offset_ << -0.5260360*2.5*0.8,-0.3762240*2.5*0.8;
    mesh_resource_ = "package://simulator/urdf/Arc.dae";
    scale_ = 0.0005*0.8;
    break;
  }
  robot_marker_initialize(robot_marker_);
  ROS_INFO("Simulator initialized.");
}

void Simulator::PoseCallback(const robotstatemsgs::robotState::ConstPtr &msg)
{
  // 位置
  double yaw = msg->yaw;
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  robot_marker_.pose.position.x = (R*offset_).x() + msg->x;
  robot_marker_.pose.position.y = (R*offset_).y() + msg->y;
  robot_marker_.pose.position.z = 4.5;

  // 姿态（根据 yaw 生成四元数）
  tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
  robot_marker_.pose.orientation.x = q.x();
  robot_marker_.pose.orientation.y = q.y();
  robot_marker_.pose.orientation.z = q.z();
  robot_marker_.pose.orientation.w = q.w();
  // 更新时间戳
  robot_marker_.header.stamp = ros::Time::now();

  marker_pub_.publish(robot_marker_);
  state_pub_.publish(msg);
}

void Simulator::robot_marker_initialize(visualization_msgs::Marker &robot_marker)
{
  // 基本属性
  robot_marker.header.frame_id = "world";          
  robot_marker.header.stamp = ros::Time::now();                    
  robot_marker.id = 0;                          
  robot_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  robot_marker.action = visualization_msgs::Marker::ADD;
  robot_marker.mesh_resource = mesh_resource_; 

  // 缩放
  robot_marker.scale.x = scale_;
  robot_marker.scale.y = scale_;
  robot_marker.scale.z = scale_;

  // 颜色
  robot_marker.color.r = 0.2;
  robot_marker.color.g = 0.86;
  robot_marker.color.b = 0.77;
  robot_marker.color.a = 0.8;

  robot_marker.lifetime = ros::Duration();
}