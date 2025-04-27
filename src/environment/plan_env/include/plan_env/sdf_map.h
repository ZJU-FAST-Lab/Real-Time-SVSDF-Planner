#ifndef _SDF_MAP_H_
#define _SDF_MAP_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <queue>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <robotstatemsgs/robotState.h>

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

class SDFmap;
class PathSDFmap;
class UnknownMap;
class MapKernel;

class SDFmap
{
  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_gridmap_;
    ros::Publisher pub_ESDF_;
    ros::Publisher pub_gradESDF_;
    ros::Timer occ_timer_;
    ros::Timer esdf_timer_;
    ros::Timer vis_timer_;

    // receive map && Odom
    ros::Subscriber cloud_sub_;
    bool has_cloud_ = false;
    bool if_have_map_ = false;
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    robotstatemsgs::robotState odom_;
    ros::Subscriber odom_sub_;
    Eigen::Vector2d update_odom_;
    bool has_odom_ = false;

    bool occ_need_update_ = false;
    bool has_map_ = false;
    bool has_esdf_ = false;
    bool esdf_need_update_ = false;
    
    // for ESDF
    std::vector<double> distance_buffer_all_;

    ros::Subscriber Pose_sub_;

    uint8_t *gridmap_ = nullptr;

    std::vector<double> occupancy_map_;
    std::vector<short> count_hit_, count_hit_and_miss_;
    std::queue<Eigen::Vector2i> cache_voxel_;
    double prob_hit_log_, prob_miss_log_;
    double clamp_max_log_, clamp_min_log_;
    double unknown_flag_;
    double min_occupancy_log_;

    double detection_range_;

    bool if_perspective_;

    bool if_cirSupRaycast_;

    // debug
    ros::Publisher debug_maker;

    // FOV
    bool hrz_limited_;
    double hrz_laser_range_dgr_;

  public:

    enum {Unknown, Unoccupied, Occupied};

    double grid_interval_;
    double inv_grid_interval_;

    double x_upper_ = -DBL_MAX, y_upper_ = -DBL_MAX;
	  double x_lower_ = DBL_MAX, y_lower_ = DBL_MAX;
    int X_SIZE_, Y_SIZE_, XY_SIZE_;
    
    
    double global_x_upper_ = -DBL_MAX, global_y_upper_ = -DBL_MAX;
	  double global_x_lower_ = DBL_MAX, global_y_lower_ = DBL_MAX;
    int GLX_SIZE_, GLY_SIZE_;
	  int GLXY_SIZE_;
    Eigen::Vector2i EIXY_SIZE_;

    std::shared_ptr<MapKernel> map_kernel_;

    SDFmap(const ros::NodeHandle &nh){
      nh_ = nh;

      // debug
      debug_maker = nh_.advertise<visualization_msgs::MarkerArray>("/SDFmap/debug",1);

      nh_.param<double>(ros::this_node::getName()+"/gridmap_interval",grid_interval_,0.1);
      inv_grid_interval_ = 1/grid_interval_;
  
      pub_gridmap_ = nh_.advertise<sensor_msgs::PointCloud2>("/SDFmap/gridmap",1);
      pub_ESDF_ = nh_.advertise<sensor_msgs::PointCloud2>("/SDFmap/ESDF",1);
      pub_gradESDF_ = nh_.advertise<visualization_msgs::MarkerArray>("/SDFmap/ESDFGrad",1);

      occ_timer_ = nh_.createTimer(ros::Duration(0.05), &SDFmap::updateOccupancyCallback, this);
      esdf_timer_ = nh_.createTimer(ros::Duration(0.05), &SDFmap::updateESDFCallback, this);
      vis_timer_ = nh_.createTimer(ros::Duration(0.5), &SDFmap::visCallback, this); //更新地图

      // cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/laser_simulator/local_pointcloud", 1, &SDFmap::pointCloudCallback, this);
      cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/local_pointcloud", 1, &SDFmap::pointCloudCallback, this);
      odom_sub_ = nh_.subscribe<robotstatemsgs::robotState>("/odom", 1, &SDFmap::odomCallback, this);

      nh_.param<double>(ros::this_node::getName()+"/detection_range",detection_range_,5.0);
      
      nh_.param<double>(ros::this_node::getName()+"/global_x_lower",global_x_lower_, -10);
      nh_.param<double>(ros::this_node::getName()+"/global_x_upper",global_x_upper_, 10);
      nh_.param<double>(ros::this_node::getName()+"/global_y_lower",global_y_lower_, -10);
      nh_.param<double>(ros::this_node::getName()+"/global_y_upper",global_y_upper_, 10);

      nh_.param<bool>(ros::this_node::getName()+"/if_perspective",if_perspective_,false);
      // std::cout<<"if_perspective: "<<if_perspective_<<std::endl;

      nh_.param<bool>(ros::this_node::getName()+"/if_cirSupRaycast", if_cirSupRaycast_, false);

      nh_.param<bool>(ros::this_node::getName()+"/hrz_limited",hrz_limited_,false);
      nh_.param<double>(ros::this_node::getName()+"/hrz_laser_range_dgr",hrz_laser_range_dgr_,360.0);
      hrz_laser_range_dgr_ = hrz_laser_range_dgr_ / 180.0 * M_PI ;
    
      
      // init map
      GLX_SIZE_ = ceil((global_x_upper_ - global_x_lower_) / grid_interval_);
      GLY_SIZE_ = ceil((global_y_upper_ - global_y_lower_) / grid_interval_);
      GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;
      EIXY_SIZE_ << GLX_SIZE_, GLY_SIZE_;
      gridmap_ = new uint8_t[GLXY_SIZE_];
      memset(gridmap_, Unknown, GLXY_SIZE_ * sizeof(uint8_t));

      distance_buffer_all_ = std::vector<double>(GLXY_SIZE_, std::numeric_limits<double>::max());

      X_SIZE_ = ceil(detection_range_ / grid_interval_) * 2;
      Y_SIZE_ = ceil(detection_range_ / grid_interval_) * 2;
      XY_SIZE_ = X_SIZE_ * Y_SIZE_;

      // Occupancy grid map
      double p_hit, p_miss, p_min, p_max, p_occ;
      nh_.getParam(ros::this_node::getName()+"/p_hit", p_hit);
      nh_.getParam(ros::this_node::getName()+"/p_miss", p_miss);
      nh_.getParam(ros::this_node::getName()+"/p_min", p_min);
      nh_.getParam(ros::this_node::getName()+"/p_max", p_max);
      nh_.getParam(ros::this_node::getName()+"/p_occ", p_occ);

      prob_hit_log_ = logit(p_hit);
      prob_miss_log_ = logit(p_miss);
      clamp_min_log_ = logit(p_min);
      clamp_max_log_ = logit(p_max);
      min_occupancy_log_ = logit(p_occ);
      unknown_flag_ = 0.01;
      occupancy_map_ = std::vector<double>(GLXY_SIZE_, clamp_min_log_ - unknown_flag_);

      count_hit_ = std::vector<short>(GLXY_SIZE_, 0);
      count_hit_and_miss_ = std::vector<short>(GLXY_SIZE_, 0);

      // map_kernel
      int rows = GLY_SIZE_;   //有多少行
      int cols = (GLX_SIZE_ + 31) / 32;    //有多少列
      map_kernel_ = std::make_shared<MapKernel>(rows, cols);  // 用于前端搜索
    }

    ~SDFmap(){
      delete[] gridmap_;
      gridmap_ = nullptr;
      map_kernel_ = nullptr;
    };

    bool get_grid_map_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void odomCallback(const robotstatemsgs::robotState::ConstPtr &msg);

    void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
    void raycastProcess();
    void cirSupRaycastProcess();
    void updateOccupancyMap();
    void RemoveOutliers();
    void updateESDFCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);

    

    //for gridmap
    Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index);
    Eigen::Vector2d gridIndex2coordd(const int &x, const int &y);
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt);
    void setObs(const Eigen::Vector3d coord);
    void setObs(const Eigen::Vector2d coord);
    Eigen::Vector2i vectornum2gridIndex(const int &num);
    int Index2Vectornum(const int &x, const int &y);
    int Index2Vectornum(const Eigen::Vector2i &index);
    inline void grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size);
    uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt);
    uint8_t CheckCollisionBycoord(const double ptx,const double pty);
    bool isInGloMap(const Eigen::Vector2d &pt);
    Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos);
    
    int setCacheOccupancy(Eigen::Vector2d pos, int occ);
    int setCacheOccupancy(Eigen::Vector2i idx, int occ);
    std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
    
    bool isOccupied(const Eigen::Vector2i &index);
    bool isOccupied(const int &idx, const int &idy);
    bool isUnOccupied(const int &idx, const int &idy);
    bool isUnOccupied(const Eigen::Vector2i &index);
    bool isUnknown(const Eigen::Vector2i &index);
    bool isUnknown(const int &idx, const int &idy);
    bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis);
    bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis);

    // visualization
    void publish_gridmap();

    // for ESDF
    void updateESDF2d();
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
    void publish_ESDF();
    void publish_ESDFGrad();
    inline double getDistance(const Eigen::Vector2i& id);
    inline double getDistance(const int& idx, const int& idy);
    double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);
    double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis);
    double getDistWithGradBilinear(const Eigen::Vector2d &pos);

    double getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad);
    
    double getDistanceReal(const Eigen::Vector2d& pos);
    // Check for collisions at half the height of the map
    inline Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt);

    Eigen::Vector2d get_update_odom();

    inline double normalize_angle(double angle);

};


// uint_32 的kernel
class MapKernel
{
public:
  int rows;
  int cols;
public:
  std::array<uint32_t, 32> mask_array_ = {
      0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80,
      0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000,
      0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000,
      0x1000000, 0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000
  };
  std::vector<std::vector<uint32_t>> map_kernel_;

  MapKernel(int rows, int cols) : rows(rows), cols(cols) 
  {
      // 初始化 map_data，每一行包含 cols 个 uint32_t
      map_kernel_.resize(rows, std::vector<uint32_t>(cols, 0));
  }
};

#endif
