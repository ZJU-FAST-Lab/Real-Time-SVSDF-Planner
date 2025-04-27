#include "simulator/global_map.hpp"

bool GlobalMap::get_grid_from_yaml(){
  std::vector<double> boxes;
  nh_.getParam(ros::this_node::getName()+"/map_box", boxes);
  std::vector<double> boundary;
  nh_.getParam(ros::this_node::getName()+"/map_size", boundary);
  x_lower = boundary[0];
  x_upper = boundary[1];
  y_lower = boundary[2];
  y_upper = boundary[3];
  
  std::vector<Eigen::Vector3d> state_locations;
  std::vector<Eigen::Matrix3d> euler_matrixs;
  std::vector<Eigen::Vector3d> link_sizes;

  for(u_int i=0; i<boxes.size()/5; i++){
    Eigen::Vector3d location(boxes[5*i], boxes[5*i+1], 0.0);
    Eigen::Vector3d size(boxes[5*i+2], boxes[5*i+3], 0.0);
    Eigen::Matrix3d euler(Eigen::AngleAxisd(boxes[5*i+4],Eigen::Vector3d::UnitZ()));
    state_locations.push_back(location);
    link_sizes.emplace_back(size);
    euler_matrixs.emplace_back(euler);
  }

  x_lower = floor(x_lower * inv_grid_interval_) * grid_interval_;
  y_lower = floor(y_lower * inv_grid_interval_) * grid_interval_;
  x_upper = ceil(x_upper * inv_grid_interval_) * grid_interval_;
  y_upper = ceil(y_upper * inv_grid_interval_) * grid_interval_;
  GLX_SIZE = static_cast<int>(round((x_upper - x_lower) * inv_grid_interval_));
  GLY_SIZE = static_cast<int>(round((y_upper - y_lower) * inv_grid_interval_));
  GLXY_SIZE = GLX_SIZE*GLY_SIZE;
  EIXY_SIZE << GLX_SIZE, GLY_SIZE;
  
  gridmap_ = new uint8_t[GLXY_SIZE];
  memset(gridmap_, Unoccupied, GLXY_SIZE * sizeof(uint8_t));
  
  if(if_boundary_wall_){
    state_locations.emplace_back(x_lower, (y_lower+y_upper)/2.0, 0);
    state_locations.emplace_back(x_upper, (y_lower+y_upper)/2.0, 0);
    state_locations.emplace_back((x_lower+x_upper)/2.0, y_lower, 0);
    state_locations.emplace_back((x_lower+x_upper)/2.0, y_upper, 0);
    link_sizes.emplace_back(grid_interval_, x_upper-x_lower, 0);
    link_sizes.emplace_back(grid_interval_, x_upper-x_lower, 0);
    link_sizes.emplace_back(y_upper-y_lower, grid_interval_, 0);
    link_sizes.emplace_back(y_upper-y_lower, grid_interval_, 0);
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
  }

  for(u_int i=0;i<link_sizes.size();i++){
    grid_insertbox(state_locations[i],euler_matrixs[i],link_sizes[i]);
  }
  get_grid_map_ = true;
  return true;
}

bool GlobalMap::get_laser_grid_from_yaml(){
  std::vector<double> boxes;
  nh_.getParam(ros::this_node::getName()+"/map_box", boxes);
  std::vector<double> boundary;
  nh_.getParam(ros::this_node::getName()+"/map_size", boundary);
  laser_x_lower = boundary[0];
  laser_x_upper = boundary[1];
  laser_y_lower = boundary[2];
  laser_y_upper = boundary[3];
  
  std::vector<Eigen::Vector3d> state_locations;
  std::vector<Eigen::Matrix3d> euler_matrixs;
  std::vector<Eigen::Vector3d> link_sizes;

  for(u_int i=0; i<boxes.size()/5; i++){
    Eigen::Vector3d location(boxes[5*i], boxes[5*i+1], 0.0);
    Eigen::Vector3d size(boxes[5*i+2], boxes[5*i+3], 0.0);
    Eigen::Matrix3d euler(Eigen::AngleAxisd(boxes[5*i+4],Eigen::Vector3d::UnitZ()));
    state_locations.push_back(location);
    link_sizes.emplace_back(size);
    euler_matrixs.emplace_back(euler);
  }

  laser_x_lower = floor(laser_x_lower * inv_laser_grid_interval_) * laser_grid_interval_;
  laser_y_lower = floor(laser_y_lower * inv_laser_grid_interval_) * laser_grid_interval_;
  laser_x_upper = ceil(laser_x_upper * inv_laser_grid_interval_) * laser_grid_interval_;
  laser_y_upper = ceil(laser_y_upper * inv_laser_grid_interval_) * laser_grid_interval_;
  laser_GLX_SIZE = static_cast<int>(round((laser_x_upper - laser_x_lower) * inv_laser_grid_interval_));
  laser_GLY_SIZE = static_cast<int>(round((laser_y_upper - laser_y_lower) * inv_laser_grid_interval_));
  laser_GLXY_SIZE = laser_GLX_SIZE*laser_GLY_SIZE;
  laser_EIXY_SIZE << laser_GLX_SIZE, laser_GLY_SIZE;
  
  laser_gridmap_ = new uint8_t[laser_GLXY_SIZE];
  memset(laser_gridmap_, Unoccupied, laser_GLXY_SIZE * sizeof(uint8_t));
  
  if(if_boundary_wall_){
    state_locations.emplace_back(laser_x_lower, (laser_y_lower+laser_y_upper)/2.0, 0);
    state_locations.emplace_back(laser_x_upper, (laser_y_lower+laser_y_upper)/2.0, 0);
    state_locations.emplace_back((laser_x_lower+laser_x_upper)/2.0, laser_y_lower, 0);
    state_locations.emplace_back((laser_x_lower+laser_x_upper)/2.0, laser_y_upper, 0);
    link_sizes.emplace_back(laser_grid_interval_, laser_x_upper-laser_x_lower, 0);
    link_sizes.emplace_back(laser_grid_interval_, laser_x_upper-laser_x_lower, 0);
    link_sizes.emplace_back(laser_y_upper-laser_y_lower, laser_grid_interval_, 0);
    link_sizes.emplace_back(laser_y_upper-laser_y_lower, laser_grid_interval_, 0);
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    euler_matrixs.emplace_back(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
  }


  for(u_int ei=0;ei<link_sizes.size();ei++){
    laser_grid_insertbox(state_locations[ei],euler_matrixs[ei],link_sizes[ei]);
  }
  return true;
}

bool GlobalMap::get_grid_from_random(){
  nh_.getParam(ros::this_node::getName()+"/Random/map_range/x_min", x_lower);
  nh_.getParam(ros::this_node::getName()+"/Random/map_range/x_max", x_upper);
  nh_.getParam(ros::this_node::getName()+"/Random/map_range/y_min", y_lower);
  nh_.getParam(ros::this_node::getName()+"/Random/map_range/y_max", y_upper);

  x_lower = floor(x_lower * inv_grid_interval_) * grid_interval_;
  y_lower = floor(y_lower * inv_grid_interval_) * grid_interval_;
  x_upper = ceil(x_upper * inv_grid_interval_) * grid_interval_;
  y_upper = ceil(y_upper * inv_grid_interval_) * grid_interval_;

  GLX_SIZE = static_cast<int>(round((x_upper - x_lower) * inv_grid_interval_));
  GLY_SIZE = static_cast<int>(round((y_upper - y_lower) * inv_grid_interval_));
  GLXY_SIZE = GLX_SIZE*GLY_SIZE;
  EIXY_SIZE << GLX_SIZE, GLY_SIZE;

  gridmap_ = new uint8_t[GLXY_SIZE];
  memset(gridmap_, Unoccupied, GLXY_SIZE * sizeof(uint8_t));

  {
    laser_x_lower = floor(x_lower * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_y_lower = floor(y_lower * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_x_upper = ceil(x_upper * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_y_upper = ceil(y_upper * inv_laser_grid_interval_) * laser_grid_interval_;

    laser_GLX_SIZE = static_cast<int>(round((laser_x_upper - laser_x_lower) * inv_laser_grid_interval_));
    laser_GLY_SIZE = static_cast<int>(round((laser_y_upper - laser_y_lower) * inv_laser_grid_interval_));
    laser_GLXY_SIZE = laser_GLX_SIZE*laser_GLY_SIZE;
    laser_EIXY_SIZE << laser_GLX_SIZE, laser_GLY_SIZE;

    laser_gridmap_ = new uint8_t[laser_GLXY_SIZE];
    memset(laser_gridmap_, Unoccupied, laser_GLXY_SIZE * sizeof(uint8_t));
  }

  get_grid_map_ = true;

  int box_num;
  nh_.getParam(ros::this_node::getName()+"/Random/obstacle_box/num", box_num);
  double length;
  nh_.getParam(ros::this_node::getName()+"/Random/obstacle_box/length", length);
  double safe_dis;
  nh_.getParam(ros::this_node::getName()+"/Random/obstacle_box/safe_dis", safe_dis);

  double start_x, start_y;
  nh_.param<double>(ros::this_node::getName()+"/start_x",start_x,0.0);
  nh_.param<double>(ros::this_node::getName()+"/start_y",start_y,0.0);

  int rand_num;
  nh_.param<int>(ros::this_node::getName()+"/srand_num", rand_num, -1);
  if(rand_num == -1){
     rand_num = time(nullptr);
  }
  srand(rand_num);
  ROS_INFO_STREAM("    get_grid_from_random rand num for srand: " << rand_num);

  for(int i=0; i<box_num; i++){
    double box_x = (double)rand() / RAND_MAX * (x_upper - x_lower) + x_lower;
    double box_y = (double)rand() / RAND_MAX * (y_upper - y_lower) + y_lower;
    double box_yaw = (double)rand() / RAND_MAX * 2 * M_PI;

    if((Eigen::Vector2d(box_x,box_y)-Eigen::Vector2d(start_x,start_y)).norm()<safe_dis){
      --i;
      continue;
    }
    
    Eigen::Matrix2d R;
    R << cos(box_yaw), -sin(box_yaw), sin(box_yaw), cos(box_yaw);

    Eigen::Vector2d x(1,0), y(0,1);
    x = R * x; y = R * y;

    float insert_interval = 0.5;
    for(float i=-length/2;i<=length/2;i+=grid_interval_*insert_interval)
      for(float j=-length/2;j<=length/2;j+=grid_interval_*insert_interval){
        Eigen::Vector2d point = Eigen::Vector2d(box_x, box_y) + i*x + j*y;
        setObs(point);
      }

    for(float i=-length/2;i<=length/2;i+=laser_grid_interval_*insert_interval)
      for(float j=-length/2;j<=length/2;j+=laser_grid_interval_*insert_interval){
        Eigen::Vector2d point = Eigen::Vector2d(box_x, box_y) + i*x + j*y;
        float coord_x = point.x();
        float coord_y = point.y();
        if (coord_x < x_lower || coord_y < y_lower ||
          coord_x >= x_upper || coord_y >= y_upper )
          continue;
        int idx_x = static_cast<int>((coord_x - x_lower) * inv_laser_grid_interval_);
        int idx_y = static_cast<int>((coord_y - y_lower) * inv_laser_grid_interval_);
        laser_gridmap_[idx_x * laser_GLY_SIZE + idx_y] = Occupied;
      }
  }

  if(if_boundary_wall_){
    Eigen::Vector3d location(x_lower, (y_lower+y_upper)/2.0, 0);
    Eigen::Vector3d size(grid_interval_, x_upper-x_lower, 0);
    Eigen::Matrix3d euler(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ()));
    grid_insertbox(location,euler,size);
    laser_grid_insertbox(location,euler,size);
    

    location = Eigen::Vector3d(x_upper, (y_lower+y_upper)/2.0, 0);
    grid_insertbox(location,euler,size);
    laser_grid_insertbox(location,euler,size);

    location = Eigen::Vector3d((x_lower+x_upper)/2.0, y_lower, 0);
    size = Eigen::Vector3d(y_upper-y_lower, grid_interval_, 0);
    grid_insertbox(location,euler,size);
    laser_grid_insertbox(location,euler,size);

    location = Eigen::Vector3d((x_lower+x_upper)/2.0, y_upper, 0);
    grid_insertbox(location,euler,size);
    laser_grid_insertbox(location,euler,size);
  }
  

  return true;
}


bool GlobalMap::get_grid_from_png(){
  double input_x_lower, input_y_lower;

  nh_.getParam(ros::this_node::getName()+"/Png/x_lower", input_x_lower);
  nh_.getParam(ros::this_node::getName()+"/Png/y_lower", input_y_lower);

  std::string png_path;
  nh_.getParam(ros::this_node::getName()+"/Png/file_path", png_path);

  cv::Mat img = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
  cv::flip(img, img, -1);
  cv::flip(img, img, 1);
  if(img.empty()){
    ROS_ERROR("Failed to load image: %s", png_path.c_str());
    return false;
  }

  GLX_SIZE = img.cols;
  GLY_SIZE = img.rows;
  GLXY_SIZE = GLX_SIZE*GLY_SIZE;
  EIXY_SIZE << GLX_SIZE, GLY_SIZE;

  x_lower = floor(input_x_lower * inv_grid_interval_) * grid_interval_;
  y_lower = floor(input_y_lower * inv_grid_interval_) * grid_interval_;
  
  x_upper = x_lower + GLX_SIZE * grid_interval_;
  y_upper = y_lower + GLY_SIZE * grid_interval_;

  gridmap_ = new uint8_t[GLXY_SIZE];
  memset(gridmap_, Unoccupied, GLXY_SIZE * sizeof(uint8_t));

  {
    laser_x_lower = floor(x_lower * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_y_lower = floor(y_lower * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_x_upper = ceil(x_upper * inv_laser_grid_interval_) * laser_grid_interval_;
    laser_y_upper = ceil(y_upper * inv_laser_grid_interval_) * laser_grid_interval_;

    laser_GLX_SIZE = static_cast<int>(round((laser_x_upper - laser_x_lower) * inv_laser_grid_interval_));
    laser_GLY_SIZE = static_cast<int>(round((laser_y_upper - laser_y_lower) * inv_laser_grid_interval_));
    laser_GLXY_SIZE = laser_GLX_SIZE*laser_GLY_SIZE;
    laser_EIXY_SIZE << laser_GLX_SIZE, laser_GLY_SIZE;

    laser_gridmap_ = new uint8_t[laser_GLXY_SIZE];
    memset(laser_gridmap_, Unoccupied, laser_GLXY_SIZE * sizeof(uint8_t));
  }

  get_grid_map_ = true;

  for(int i=0; i<GLX_SIZE; i++){
    for(int j=0; j<GLY_SIZE; j++){
      if(img.at<uchar>(j, i) < 128){
        Eigen::Vector2d point = gridIndex2coordd(i, j);
        setObs(point);
        laser_gridmap_[i * laser_GLY_SIZE + j] = Occupied;
      }
    }
  }  



  return true;
}

Eigen::Vector2d GlobalMap::gridIndex2coordd(const Eigen::Vector2i &index){
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + x_lower;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + y_lower;
  return pt;
}

Eigen::Vector2d GlobalMap::gridIndex2coordd(const int &x, const int &y){
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + x_lower;
  pt(1) = ((double)y + 0.5) * grid_interval_ + y_lower;
  return pt;
}

Eigen::Vector2i GlobalMap::coord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - x_lower) * inv_grid_interval_), 0), GLX_SIZE - 1),
      std::min(std::max(int((pt(1) - y_lower) * inv_grid_interval_), 0), GLY_SIZE - 1);
  return idx;
}

void GlobalMap::setObs(const Eigen::Vector3d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < x_lower || coord_y < y_lower ||
    coord_x >= x_upper || coord_y >= y_upper )
    return;
  int idx_x = static_cast<int>((coord_x - x_lower) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - y_lower) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE + idx_y] = Occupied;
}

void GlobalMap::setObs(const Eigen::Vector2d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < x_lower || coord_y < y_lower ||
    coord_x >= x_upper || coord_y >= y_upper )
    return;
  int idx_x = static_cast<int>((coord_x - x_lower) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - y_lower) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE + idx_y] = Occupied;
}

Eigen::Vector2i GlobalMap::vectornum2gridIndex(const int &num){
  Eigen::Vector2i index;
  index(0) = num / GLY_SIZE;
  index(1) = num % GLY_SIZE;
  return index;
}

inline int GlobalMap::Index2Vectornum(const int &x, const int &y){
  return x * GLY_SIZE + y;
}

inline void GlobalMap::grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.3;
  for(float i=-size.x()/2;i<=size.x()/2;i+=grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=grid_interval_*insert_interval){
        Eigen::Vector3d point = location+i*x+j*y+k*z;
        setObs(point);
      }
}

inline void GlobalMap::laser_grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.3;
  for(float i=-size.x()/2;i<=size.x()/2;i+=laser_grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=laser_grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=laser_grid_interval_*insert_interval){
        Eigen::Vector3d point = location+i*x+j*y+k*z;
        if(point.x()<laser_x_lower||point.x()>laser_x_upper||point.y()<laser_y_lower||point.y()>laser_y_upper)
          continue;
        int idx_x = static_cast<int>((point.x() - x_lower) * inv_laser_grid_interval_);
        int idx_y = static_cast<int>((point.y() - y_lower) * inv_laser_grid_interval_);
        laser_gridmap_[idx_x * laser_GLY_SIZE + idx_y] = Occupied;
      }
}

uint8_t GlobalMap::CheckCollisionBycoord(const Eigen::Vector2d &pt){
  if(pt.x()>x_upper||pt.x()<x_lower||pt.y()>y_upper||pt.y()<y_lower){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f",pt.x(),pt.y());
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLY_SIZE + index.y()];
}

uint8_t GlobalMap::CheckCollisionBycoord(const double ptx,const double pty){
  if(ptx>x_upper||ptx<x_lower||pty>y_upper||pty<y_lower){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f %f",ptx,pty);
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(Eigen::Vector2d(ptx,pty));
  return gridmap_[index.x() * GLY_SIZE + index.y()];
}

void GlobalMap::publish_gridmap(const ros::TimerEvent& event){
  if(!get_grid_map_) return;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;
  for(int idx = 1;idx < GLXY_SIZE;idx++){
    if(gridmap_[idx]==Occupied){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZ pt(corrd.x(),corrd.y(),-0.1);
      cloud_vis.points.push_back(pt);
    }
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "world";
  map_vis.header.stamp = ros::Time::now();
  pub_gridmap_.publish(map_vis);

  pcl::PointCloud<pcl::PointXYZ> laser_cloud_vis;
  sensor_msgs::PointCloud2 laser_map_vis;
  for(int idx = 1;idx < laser_GLXY_SIZE;idx++){
    if(laser_gridmap_[idx]==Occupied){
      for(int z_low = -5, z_high = 5; z_low < z_high; z_low++){
        Eigen::Vector2d corrd;
        corrd.x() = laser_x_lower + (double(idx / laser_GLY_SIZE) + 0.5) * laser_grid_interval_;
        corrd.y() = laser_y_lower + (double(idx % laser_GLY_SIZE) + 0.5) * laser_grid_interval_;
        pcl::PointXYZ pt(corrd.x(),corrd.y(),z_low*laser_grid_interval_);
        laser_cloud_vis.points.push_back(pt);
      }
    }
  }
  laser_cloud_vis.width = laser_cloud_vis.points.size();
  laser_cloud_vis.height = 1;
  laser_cloud_vis.is_dense = true;
  pcl::toROSMsg(laser_cloud_vis, laser_map_vis);
  laser_map_vis.header.frame_id = "world";
  laser_map_vis.header.stamp = ros::Time::now();
  pub_laser_gridmap_.publish(laser_map_vis);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "global_map_node");
  ros::NodeHandle nh;
  
  GlobalMap global_map(nh);

  ros::spin();

  return 0;
}
