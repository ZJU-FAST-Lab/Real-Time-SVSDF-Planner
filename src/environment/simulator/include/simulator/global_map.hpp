#ifndef GLOBAL_MAP_H_
#define GLOBAL_MAP_H_

#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>         // std::function 和 Lambda 表达式
#include <ros/timer.h>
#include <opencv4/opencv2/opencv.hpp>

#define Unoccupied 0
#define Occupied 1
#define Unknown 2

class GlobalMap{
    private:
        ros::NodeHandle nh_;

        ros::Publisher pub_gridmap_;
        ros::Publisher pub_laser_gridmap_;
        ros::Publisher pub_surf_;

        ros::Timer timer_pub_gridmap_;

        uint8_t *gridmap_;

        bool get_grid_map_;
        double grid_interval_, inv_grid_interval_;

        double x_upper = -DBL_MAX, y_upper = -DBL_MAX;
        double x_lower = DBL_MAX, y_lower = DBL_MAX;
        int GLX_SIZE, GLY_SIZE;
        int GLXY_SIZE;
        Eigen::Vector2i EIXY_SIZE;

        uint8_t *laser_gridmap_;
        double laser_grid_interval_, inv_laser_grid_interval_;
        double laser_x_upper = -DBL_MAX, laser_y_upper = -DBL_MAX;
        double laser_x_lower = DBL_MAX, laser_y_lower = DBL_MAX;
        int laser_GLX_SIZE, laser_GLY_SIZE;
        int laser_GLXY_SIZE;
        Eigen::Vector2i laser_EIXY_SIZE;

        bool if_boundary_wall_;

        bool get_grid_from_yaml();
        bool get_grid_from_random();
        bool get_grid_from_png();
        
        bool get_laser_grid_from_yaml();

        //for gridmap
        Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index);
        Eigen::Vector2d gridIndex2coordd(const int &x, const int &y);
        Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt);
        void setObs(const Eigen::Vector3d coord);
        void setObs(const Eigen::Vector2d coord);
        Eigen::Vector2i vectornum2gridIndex(const int &num);
        inline int Index2Vectornum(const int &x, const int &y);
        inline void grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size);
        inline void laser_grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size);
        uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt);
        uint8_t CheckCollisionBycoord(const double ptx,const double pty);

        void publish_gridmap(const ros::TimerEvent& event);


    public:
        
        GlobalMap(const ros::NodeHandle &nh): nh_(nh){
        
            get_grid_map_ = false;
            nh_.param<double>(ros::this_node::getName()+"/gridmap_interval",grid_interval_,0.1);
            inv_grid_interval_ = 1/grid_interval_;
        
            pub_gridmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map/gridmap", 1);
            pub_laser_gridmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map/laser_gridmap", 1);
            pub_surf_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map/surf", 1);

            timer_pub_gridmap_ = nh_.createTimer(ros::Duration(1.0), &GlobalMap::publish_gridmap, this);

            nh_.param<double>(ros::this_node::getName()+"/laser_gridmap_interval",laser_grid_interval_,0.01);
            inv_laser_grid_interval_ = 1/laser_grid_interval_;
            
            nh_.param<bool>(ros::this_node::getName()+"/if_boundary_wall_",if_boundary_wall_,true);

            int map_input_method;
            nh_.param<int>(ros::this_node::getName()+"/Map_input_method",map_input_method, -1);
            switch(map_input_method){
                case 1:{
                if(get_grid_from_yaml()){
                    get_laser_grid_from_yaml();
                    ROS_INFO("Reading map parameters from yaml file. SUCCESS!!!");
                }

                else
                    ROS_ERROR("Reading map parameters from yaml file. ERROR!!!");
                break;
                }
                case 2:{
                if(get_grid_from_random())
                    ROS_INFO("Randomly read map parameters. SUCCESS!!!");
                else
                    ROS_ERROR("Randomly read map parameters. ERROR!!!");
                break;
                }
                case 3:{
                if(get_grid_from_png()){
                    ROS_INFO("Reading map parameters from png file. SUCCESS!!!");
                }
                else
                    ROS_ERROR("Reading map parameters from png file. ERROR!!!");
                break;
                }
                default:
                ROS_ERROR("input method ERROR!!! please check model_config.yaml Map_input_method");
            }
        }

        
};



#endif