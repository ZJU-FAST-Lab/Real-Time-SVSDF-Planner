#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

#define Delayforvis(pub,topic) \
    do { \
        double wait_time = 0.0; \
        while (pub.getNumSubscribers() < 1) { \
            ros::Duration(0.1).sleep(); \
            wait_time += 0.1; \
            if (wait_time > 0.5) { \
                std::cout << "Looks like Topic " << topic << " is not subscribed by any subscriber. Check rviz config."<<std::endl; \
                break; \
            } \
        } \
    } while(0);

using namespace Eigen;
using namespace std;
namespace vis
{
    using PublisherMap = std::unordered_map<std::string, ros::Publisher>; 
                                                                        
    enum Color
    {
        white,
        red,
        green,
        blue,
        light_blue,
        yellow,
        chartreuse,
        black,
        gray,
        orange,
        purple,
        pink,
        steelblue,
        lightyellow,
        lightred,
        darkred
    };


    class Visualization
    {
    private:
        ros::NodeHandle nh_;
        PublisherMap publisher_map_; 
        

        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   Color color = blue,
                                   double a = 1)
        {
            marker.color.a = a;
            switch (color)
            {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case light_blue:
                marker.color.r = 0.4;
                marker.color.g = 0.6;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            case lightyellow:
                marker.color.r = 0.9490196;
                marker.color.g = 0.8862745;
                marker.color.b = 0.5664062;
                break;
            case lightred:
                marker.color.r = 0.9490196;
                marker.color.g = 0.6549019;
                marker.color.b = 0.5019607;
                break;
            case darkred:
                marker.color.r = 0.8470588;
                marker.color.g = 0.4784313;
                marker.color.b = 0.2901960;
                break;
            }
        }

   
        inline void setMarkerColor(visualization_msgs::Marker &marker,
                                   double a,
                                   double r,
                                   double g,
                                   double b)
        {
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        }

        inline void setMarkerScale(visualization_msgs::Marker &marker,
                                   const double &x,
                                   const double &y,
                                   const double &z)
        {
            marker.scale.x = x;
            marker.scale.y = y;
            marker.scale.z = z;
        }
    
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
       
        template <class ROTATION>
        inline void setMarkerPose(visualization_msgs::Marker &marker,
                                  const double &x,
                                  const double &y,
                                  const double &z,
                                  const ROTATION &R)
        {
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            Eigen::Quaterniond r(R);
            marker.pose.orientation.w = r.w();
            marker.pose.orientation.x = r.x();
            marker.pose.orientation.y = r.y();
            marker.pose.orientation.z = r.z();
        }

    public:
        int shape_ = 0;
        Eigen::Vector2d offset_;
        std::string mesh_resource_ = "package://simulator/urdf/Arc.dae";
        double scale_ = 0.0005*0.8;

        Visualization() = delete;
        Visualization(ros::NodeHandle &nh) : nh_(nh) 
        {
            ros::param::get("/global_planning/shape", shape_);
            std::cout<<"vis shape"<< shape_<<std::endl;
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
        }
        typedef std::shared_ptr<Visualization> Ptr;

        void front_end_visualization(const std::string &robot_id, const std::vector<Eigen::Vector3d> &robot_path, Color color = steelblue, double height = 4.4)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(0);  // 永不过期

            marker.mesh_resource = mesh_resource_;
            setMarkerColor(marker, color, 0.2);
            setMarkerScale(marker, scale_, scale_, scale_);

            visualization_msgs::MarkerArray marker_array;
            for (size_t i = 0; i < robot_path.size(); ++i)
            {
                const Eigen::Vector3d &state = robot_path[i];
                double x = state.x();
                double y = state.y();
                double yaw = state.z();
                Eigen::AngleAxisd rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
                Eigen::Matrix2d R;
                R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
                x = (R*offset_).x() + x;
                y = (R*offset_).y() + y;
                setMarkerPose(marker, x, y, height, rotation);
                marker.id ++;

                // 更新Publisher
                if (publisher_map_.find(robot_id) == publisher_map_.end())
                {
                    ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>("vis/" + robot_id, 1);
                    publisher_map_[robot_id] = pub;
                    Delayforvis(pub,robot_id);
                }
                // 发布Marker
                marker_array.markers.push_back(marker);
            }
            publisher_map_[robot_id].publish(marker_array);
        }

        inline void visBsplineTraj(const std::string &topic, 
                                   const std::vector<Eigen::Vector3d> &route, 
                                   int traj_id = 1552999, 
                                   bool ontop = false)
        {
            // 查找是否已有对应的Publisher
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                // 如果没有，则创建一个新的Publisher
                ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(topic, 10000); 
                publisher_map_[topic] = pub1;

                // 等待订阅者连接
                double wait_time = 0.0;
                while (pub1.getNumSubscribers() < 1) {
                    ros::Duration(0.1).sleep();
                    wait_time += 0.1;
                    if(wait_time > 0.5){
                        std::cout << "[Livz] 话题 " << topic 
                                  << " 似乎没有订阅者,请检查rviz配置。"
                                  << "[Livz] Looks like Topic " << topic 
                                  << " is not subscribed by any subscriber. Check rviz config." 
                                  << std::endl;
                        break;
                    }
                } 
            }

            // 创建Marker消息
            visualization_msgs::Marker traj_vis;
            std::string st("step_traj");
            if(topic == st)
            {
                traj_vis.lifetime = ros::Duration(0.5);
            }
            else
            {
                traj_vis.lifetime = ros::Duration(0); // 永不过期
            }

            traj_vis.header.stamp       = ros::Time::now();
            traj_vis.header.frame_id    = "world";
            traj_vis.id                 = traj_id;
            traj_vis.type               = visualization_msgs::Marker::LINE_STRIP;
            traj_vis.scale.x            = 0.05; // 线宽
            traj_vis.scale.y            = 0.05;
            traj_vis.scale.z            = 0.05;
            traj_vis.pose.orientation.x = 0.0;
            traj_vis.pose.orientation.y = 0.0;
            traj_vis.pose.orientation.z = 0.0;
            traj_vis.pose.orientation.w = 1.0;

            // 设置颜色
            setMarkerColor(traj_vis, vis::Color::darkred, 1.0);

            // 添加轨迹点
            geometry_msgs::Point pt;
            if (!route.empty()) {
                for (const auto& it : route) {
                    pt.x = it(0);
                    pt.y = it(1);
                    pt.z = ontop ? 6 : 0; // 根据参数设置高度
                    traj_vis.points.push_back(pt);
                }
                // 发布Marker
                publisher_map_[topic].publish(traj_vis);
            }
        }

        void visualize_points(const std::string &robot_id, 
                             const std::vector<Eigen::Vector2d> &points,
                             Color color = blue,
                             double scale = 0.1)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = robot_id + "_points";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(0); // 永不过期

            // 设置点的颜色和大小
            setMarkerColor(marker, color, 1.0);
            marker.scale.x = scale;
            marker.scale.y = scale;

            // 设置点的坐标
            for(const auto &pt : points)
            {
                geometry_msgs::Point ros_pt;
                ros_pt.x = pt.x();
                ros_pt.y = pt.y();
                ros_pt.z = 0; // 可以根据需要设置高度
                marker.points.push_back(ros_pt);
            }

            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.push_back(marker);

            // 更新Publisher
            if (publisher_map_.find(robot_id) == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>("vis/" + robot_id + "/points", 1);
                publisher_map_[robot_id] = pub;
            }

            // 发布Marker
            publisher_map_[robot_id].publish(marker_array);
        }

        template <class PC>
        inline void visPointcloudXYZI(const PC &pc, const std::string &topic)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>("vis/" + topic + "/SDF", 10000);
                publisher_map_[topic] = pub;
                Delayforvis(pub,topic);
            }
            sensor_msgs::PointCloud2 point_cloud_msg;
            pcl::toROSMsg(pc, point_cloud_msg);
            point_cloud_msg.header.frame_id = "world";
            point_cloud_msg.header.stamp = ros::Time::now();
            publisher_map_[topic].publish(point_cloud_msg);
        }

        void visControlPoints(const std::string &topic, 
                          const Eigen::MatrixXd &control_points_, 
                          Color color = red, 
                          double scale = 0.05)
    {
        visualization_msgs::MarkerArray marker_array;
        ROS_INFO("vis COntrol POints");
        // 创建控制点的 POINTS 标记
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "world";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = topic + "_control_points";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.lifetime = ros::Duration(0); // 永不过期

        // 设置颜色和大小
        setMarkerColor(points_marker, color, 1.0);
        points_marker.scale.x = scale;
        points_marker.scale.y = scale;

        // 添加点
        for (int i = 0; i < control_points_.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = control_points_(0, i);
            pt.y = control_points_(1, i);
            pt.z = 7.0; // 固定高度，可以根据需要调整
            points_marker.points.push_back(pt);
        }

        marker_array.markers.push_back(points_marker);

        // 创建文本标记，用于显示每个控制点的编号
        for (int i = 0; i < control_points_.cols(); ++i)
        {
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "world";
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = topic + "_control_points_text";
            text_marker.id = i + 1; // 确保每个文本标记有唯一的 ID，避免与点的 ID 冲突
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.lifetime = ros::Duration(0); // 永不过期

            // 设置文本内容为控制点的索引
            text_marker.text = std::to_string(i);

            // 设置文本的缩放（大小）
            text_marker.scale.z = 0.2; // 根据需要调整文本大小

            // 设置文本颜色（例如黑色）
            text_marker.color.a = 1.0;
            text_marker.color.r = 0.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 0.0;

            // 设置文本的位置，稍微偏移以避免与点重叠
            text_marker.pose.position.x = control_points_(0, i) + 0.1; // x 方向偏移
            text_marker.pose.position.y = control_points_(1, i) + 0.1; // y 方向偏移
            text_marker.pose.position.z = 7.2; // 高度略高于点

            // 方向保持默认
            text_marker.pose.orientation.w = 1.0;

            marker_array.markers.push_back(text_marker);
        }

        // 创建或获取 Publisher
        std::string marker_array_topic = "vis/" + topic + "/control_points_with_text";
        if (publisher_map_.find(marker_array_topic) == publisher_map_.end())
        {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(marker_array_topic, 1);
            publisher_map_[marker_array_topic] = pub;
            ROS_INFO("Created publisher for topic: %s", marker_array_topic.c_str());
        }

        // 发布 MarkerArray
        publisher_map_[marker_array_topic].publish(marker_array);
    }
        
        inline void visVector(const Eigen::Vector3d &vec_pos, 
                              Eigen::Vector3d vec_dir,  
                              const std::string &topic, 
                              Color color = black, 
                              const int id = 1, 
                              bool clear_old = false)
        {
            auto got = publisher_map_.find(topic);
            if (got == publisher_map_.end())
            {
                ros::Publisher pub =
                    nh_.advertise<visualization_msgs::Marker>("vis/" + topic + "/vector", 10000);
                publisher_map_[topic] = pub;
            }

            if(clear_old){
                visualization_msgs::Marker clc;
                clc.header.frame_id = "world";
                clc.type = visualization_msgs::Marker::DELETEALL;
                publisher_map_[topic].publish(clc);
            }

            visualization_msgs::Marker vec;
            vec.type = visualization_msgs::Marker::ARROW;
            vec.action = visualization_msgs::Marker::ADD;
            vec.header.frame_id = "world";
            vec.id = id;

            double length = vec_dir.norm();  // 向量的长度
            vec.scale.x = length;  // 根据向量的长度设置箭头的长度
            vec.scale.y = 0.1;
            vec.scale.z = 0.1;
            vec.color.a = 1;
            vec.color.r = 1.0;
            vec.color.g = 1.0;
            vec.color.b = 0.0;
            vec.pose.position.x = vec_pos(0);
            vec.pose.position.y = vec_pos(1);
            vec.pose.position.z = vec_pos(2);
            vec_dir.normalize();
            Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + vec_dir);
            // final_pose.normalize();
            Eigen::AngleAxisd t_V(M_PI, final_pose);
            Eigen::Quaterniond q(t_V);
            q.normalize();
            vec.pose.orientation.w = q.w();
            vec.pose.orientation.x = q.x();
            vec.pose.orientation.y = q.y();
            vec.pose.orientation.z = q.z();
            publisher_map_[topic].publish(vec);
        }

        template <class TOPIC>
        inline void pubFloat64(const TOPIC &topic, const double msg)
        {
            auto got = publisher_map_.find(topic); // 如果unordered_map中没有存储这个键值对，即插入这个对
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            std_msgs::Float64 datamsg;
            datamsg.data = msg;
            publisher_map_[topic].publish(datamsg);
        }
}; // namespace vis
}
#endif
