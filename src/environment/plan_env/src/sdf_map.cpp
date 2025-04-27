#include "plan_env/sdf_map.h"

void SDFmap::odomCallback(const robotstatemsgs::robotState::ConstPtr &msg){
  odom_ = *msg;
  has_odom_ = true;
}

void SDFmap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  static tf2_ros::Buffer tf_buffer(ros::Duration(10.0));
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  // Get transformation information
  geometry_msgs::TransformStamped transformStamped;
  try {
      transformStamped = tf_buffer.lookupTransform("world", msg->header.frame_id,
                                                  msg->header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
  }

  // Perform coordinate transformation
  sensor_msgs::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transformStamped);

  // Convert to PCL format
  pcl::fromROSMsg(transformed_cloud, cloud_);
  
  occ_need_update_ = true;
}

void SDFmap::updateOccupancyCallback(const ros::TimerEvent& /*event*/){
  if(!occ_need_update_) return;

  if(!if_perspective_){
    ros::Time t1, t2;
    t1 = ros::Time::now();

    x_lower_ = std::max(odom_.x - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_.x + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_.y - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_.y + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    raycastProcess();

    if(if_cirSupRaycast_){
      static int cirSup = 1;
      cirSup ++;
      if(cirSup%3==0){
        cirSupRaycastProcess();
        cirSup = 1;
      }
    }

    RemoveOutliers();
    
    // Update the map based on occupancy_map_
    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    // // Will treat obstacles as free space
    // for (int x=min_id.x(); x<=max_id.x(); x++) {
    //   for (int y=min_id.y(); y<=max_id.y(); y++){
    //     if(occupancy_map_[Index2Vectornum(x,y)] >= clamp_min_log_ && occupancy_map_[Index2Vectornum(x,y)] <= min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Unoccupied;
    //     }
    //     else if(occupancy_map_[Index2Vectornum(x,y)] > min_occupancy_log_){
    //       gridmap_[Index2Vectornum(x,y)] = Occupied;
    //     }
    //   }
    // }
    // Will not treat obstacles as free space!!
    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        int vecIndex = Index2Vectornum(x,y);
        if(gridmap_[vecIndex] == Unknown && occupancy_map_[vecIndex] >= clamp_min_log_ && occupancy_map_[vecIndex] <= min_occupancy_log_){
          gridmap_[vecIndex] = Unoccupied;
        }
        else if(occupancy_map_[vecIndex] > min_occupancy_log_){
          gridmap_[vecIndex] = Occupied;
          //同步更新map_kernel
          map_kernel_-> map_kernel_[y][x/32] |= map_kernel_ -> mask_array_[x%32];
        }
      }
    }
  }

  
  else{
    x_lower_ = std::max(odom_.x - ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_.x + ceil(detection_range_ / grid_interval_)*grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_.y - ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_.y + ceil(detection_range_ / grid_interval_)*grid_interval_, global_y_upper_);

    X_SIZE_ = ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    for (int x=min_id.x(); x<=max_id.x(); x++) {
      for (int y=min_id.y(); y<=max_id.y(); y++){
        if(gridmap_[Index2Vectornum(x,y)] == Unknown)
          gridmap_[Index2Vectornum(x,y)] = Unoccupied;
      }
    }

    for(auto point:cloud_.points){
      Eigen::Vector2d coord = Eigen::Vector2d(point.x, point.y);
      if(!isInGloMap(coord)){
        continue;
      }
      Eigen::Vector2i idx = coord2gridIndex(coord);
      gridmap_[Index2Vectornum(idx)] = Occupied;
    }
  }


  has_map_ = true;
  esdf_need_update_ = true;
  occ_need_update_ = false;
}

void SDFmap::raycastProcess(){
  int points_cnt = cloud_.points.size();
  
  update_odom_ = Eigen::Vector2d(odom_.x, odom_.y);
  Eigen::Vector2d cur_point;
  int vox_idx;
  double length;

  Eigen::Vector3d ray_pt;
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);

  RayCaster raycaster;

  for (int i = 0; i < points_cnt; ++i) {
    cur_point << cloud_.points[i].x, cloud_.points[i].y;
    if(!isInGloMap(cur_point)){
      cur_point = closetPointInMap(cur_point, update_odom_);
      length = (cur_point - update_odom_).norm();
      if(length > detection_range_){
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
      }
      vox_idx = setCacheOccupancy(cur_point, 0);
    }
    else {
      length = (cur_point - update_odom_).norm();
      if (length > detection_range_) {
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
        vox_idx = setCacheOccupancy(cur_point, 0);
      } else {
        vox_idx = setCacheOccupancy(cur_point, 1);
      }
    }
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(coord2gridIndex(update_odom_), coord2gridIndex(cur_point));
    
    int size = line.size() - 1;

    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }

  }

  updateOccupancyMap();
}

// Add a circle to prevent no points, resulting in no safe position points
void SDFmap::cirSupRaycastProcess(){
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
  double length;
  int vox_idx;

  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_.x - detection_range_; x < odom_.x + detection_range_+1e-10;  x += 2*detection_range_){
    for(double y = odom_.y - detection_range_; y < odom_.y + detection_range_+1e-10;  y += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }
  for(double y = odom_.y - detection_range_; y < odom_.y + detection_range_+1e-10;  y += 2*detection_range_){
    for(double x = odom_.x - detection_range_; x < odom_.x + detection_range_+1e-10;  x += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }

  RayCaster raycaster;

  for(auto cir_point:cir_points){
   
    if(hrz_limited_){
      double angle = atan2(cir_point.y() - odom_.y, cir_point.x() - odom_.x);
      angle = normalize_angle(angle - odom_.yaw);
      if(angle < -hrz_laser_range_dgr_/2.2 || angle >hrz_laser_range_dgr_/2.2){
        continue;
      }
    }

    if(!isInGloMap(cir_point)){
      cir_point = closetPointInMap(cir_point, update_odom_);
    }

    length = (cir_point - update_odom_).norm();
    if(length > detection_range_){
      cir_point = (cir_point - update_odom_) / length * detection_range_ + update_odom_;
    }

    std::vector<Eigen::Vector2i> line;
    raycaster.setInput(Eigen::Vector3d(cir_point.x(), cir_point.y(), 0.1) / grid_interval_, Eigen::Vector3d(update_odom_.x(), update_odom_.y(), 0.1) / grid_interval_);

    bool occ = false;
    Eigen::Vector3d ray_pt;
    while (raycaster.step(ray_pt)) {
      Eigen::Vector2d tmp = (ray_pt.head(2) + half) * grid_interval_;
      Eigen::Vector2i tmp_idx = coord2gridIndex(tmp);
      line.emplace_back(tmp_idx);
      if(gridmap_[Index2Vectornum(coord2gridIndex(tmp))] == Occupied || 
         (tmp_idx.y() < GLY_SIZE_ && gridmap_[Index2Vectornum(coord2gridIndex(tmp))+1] == Occupied) || 
         (tmp_idx.y() > 0 && gridmap_[Index2Vectornum(coord2gridIndex(tmp))-1] == Occupied) || 
         (tmp_idx.x() < GLX_SIZE_ && gridmap_[Index2Vectornum(coord2gridIndex(tmp))+GLY_SIZE_] == Occupied) || 
         (tmp_idx.x() > 0 && gridmap_[Index2Vectornum(coord2gridIndex(tmp))-GLY_SIZE_] == Occupied)){
        occ = true;
        break;
      }
    }
    if(occ) continue;
    
    int size = line.size()-1;
    for (int i=0; i<size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }
  }

  // updateOccupancyMap();

  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 4*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    log_odds_update = 0.0;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }

}

void SDFmap::updateOccupancyMap(){
  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 3*count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
  }
}

void SDFmap::RemoveOutliers(){
  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_.x - detection_range_; x < odom_.x + detection_range_+1e-10;  x += grid_interval_){
    for(double y = odom_.y - detection_range_; y < odom_.y + detection_range_+1e-10;  y += grid_interval_){
      cir_points.emplace_back(x,y);
    }
  }

  double xlow = global_x_lower_ + grid_interval_;
  double xup = global_x_upper_ - grid_interval_;
  double ylow = global_y_lower_ + grid_interval_;
  double yup = global_y_upper_ - grid_interval_;
  for(auto cir_point:cir_points){
    if(cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow && cir_point.y() < yup){
      if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] == Unknown){
        if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+1] == Unoccupied && 
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-1] == Unoccupied && 
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))+GLY_SIZE_] == Unoccupied && 
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point))-GLY_SIZE_] == Unoccupied){
          gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] = Unoccupied;
        }
      }
    }
    
  }
  Eigen::Vector2i idx = coord2gridIndex(Eigen::Vector2d(odom_.x, odom_.y));
  for(int i=-1; i<=1; i++){
    for(int j=-1; j<=1; j++){
      if(gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] == Unknown){
        gridmap_[Index2Vectornum(idx.x()+i, idx.y()+j)] = Unoccupied;
      }
    }
  }

}

int SDFmap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
  if (occ != 1 && occ != 0) return -1;

  Eigen::Vector2i idx = coord2gridIndex(pos);
  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


int SDFmap::setCacheOccupancy(Eigen::Vector2i idx, int occ) {
  if (occ != 1 && occ != 0) return -1;

  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}


std::vector<Eigen::Vector2i> SDFmap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
    std::vector<Eigen::Vector2i> line;
    
    int dx = abs(end.x() - start.x());
    int dy = abs(end.y() - start.y());
    int sx = (start.x() < end.x()) ? 1 : -1;
    int sy = (start.y() < end.y()) ? 1 : -1;
    int err = dx - dy;

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
        line.emplace_back(x0, y0);
        if (x0 == end.x() && y0 == end.y()) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return line;
}

void SDFmap::updateESDFCallback(const ros::TimerEvent& /*event*/){
  if(!esdf_need_update_) return;
  updateESDF2d();

  has_esdf_ = true;
  esdf_need_update_ = false;
  
}

void SDFmap::visCallback(const ros::TimerEvent& /*event*/){
  if(has_map_){
    //ROS_WARN("publish_gridmap()");
    publish_gridmap();
    // exit(0);
  }
  if(has_esdf_){
    publish_ESDF();
    // publish_ESDFGrad();
  }
}

inline void SDFmap::grid_insertbox(Eigen::Vector3d location,Eigen::Matrix3d euler,Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.5;
  for(float i=-size.x()/2;i<=size.x()/2;i+=grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=grid_interval_*insert_interval){
        Eigen::Vector3d point = location+i*x+j*y+k*z;
        setObs(point);
      }
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const Eigen::Vector2i &index){
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const int &x, const int &y){
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)y + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2i SDFmap::coord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_), 0), GLX_SIZE_ - 1),
      std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_), 0), GLY_SIZE_ - 1);
  return idx;
}

void SDFmap::setObs(const Eigen::Vector3d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_ )
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

void SDFmap::setObs(const Eigen::Vector2d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_ )
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

Eigen::Vector2i SDFmap::vectornum2gridIndex(const int &num){
  Eigen::Vector2i index;
  index(0) = num / GLY_SIZE_;
  index(1) = num % GLY_SIZE_;
  return index;
}

int SDFmap::Index2Vectornum(const int &x, const int &y){
  return x * GLY_SIZE_ + y;
}

int SDFmap::Index2Vectornum(const Eigen::Vector2i &index){
  return index.x() * GLY_SIZE_ + index.y();
}

void SDFmap::publish_gridmap(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;
  for(int idx = 1;idx < GLXY_SIZE_;idx++){
    // if(gridmap_[idx]==Unoccupied){
    //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
    //   pcl::PointXYZI pt;
    //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
    //   pt.intensity = 8.0;
    //   cloud_vis.points.push_back(pt);
    // }
    // if(gridmap_[idx]==Occupied){
    //   Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
    //   pcl::PointXYZI pt;
    //   pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
    //   pt.intensity = 0.0;
    //   cloud_vis.points.push_back(pt);
    // } // 雷达探测到的障碍物的边界
    if(gridmap_[idx]==Unknown){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      pt.intensity = 8.0;
      cloud_vis.points.push_back(pt);
    } // 未知区域的环境
  }

  pcl::PointXYZI pt;
  pt.x = 100.0; pt.y = 100.0; pt.z = 0.1;
  pt.intensity = 10.0;
  cloud_vis.points.push_back(pt);

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "world";
  pub_gridmap_.publish(map_vis);
}

uint8_t SDFmap::CheckCollisionBycoord(const Eigen::Vector2d &pt){
  if(pt.x()>global_x_upper_||pt.x()<global_x_lower_||pt.y()>global_y_upper_||pt.y()<global_y_lower_){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f",pt.x(),pt.y());
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

uint8_t SDFmap::CheckCollisionBycoord(const double ptx,const double pty){
  if(ptx>global_x_upper_||ptx<global_x_lower_||pty>global_y_upper_||pty<global_y_lower_){
    // ROS_ERROR("[CheckCollisionBycoord], coord out of map!!! %f %f %f",ptx,pty);
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(Eigen::Vector2d(ptx,pty));
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

bool SDFmap::isInGloMap(const Eigen::Vector2d &pt){
  return pt.x() < global_x_upper_ && pt.x() > global_x_lower_ && pt.y() < global_y_upper_ && pt.y() > global_y_lower_;
}

Eigen::Vector2d SDFmap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos){
  Eigen::Vector2d diff = pt - pos;
  Eigen::Vector2d max_tc = Eigen::Vector2d(global_x_upper_, global_y_upper_) - pos;
  Eigen::Vector2d min_tc = Eigen::Vector2d(global_x_lower_, global_y_lower_) - pos;

  double min_t = 1000000;

  for (int i = 0; i < 2; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return pos + (min_t - 1e-3) * diff;
}



void SDFmap::updateESDF2d(){
  Eigen::Vector2i min_esdf(floor(std::max(0.0, odom_.x - detection_range_ - global_x_lower_)*inv_grid_interval_), floor(std::max(0.0, odom_.y - detection_range_ - global_y_lower_)*inv_grid_interval_));
  Eigen::Vector2i max_esdf(ceil(std::min(global_x_upper_ - global_x_lower_, odom_.x + detection_range_ - global_x_lower_)*inv_grid_interval_) - 1,
                           ceil(std::min(global_y_upper_ - global_y_lower_, odom_.y + detection_range_ - global_y_lower_)*inv_grid_interval_) - 1);

  int update_X_SIZE = max_esdf.x() - min_esdf.x();
  int update_Y_SIZE = max_esdf.y() - min_esdf.y();
  int update_XY_SIZE = (update_X_SIZE + 1) * (update_Y_SIZE + 1);

  std::vector<double> tmp_buffer1_ = std::vector<double>(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer_ = std::vector<double>(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer_neg_ = std::vector<double>(update_XY_SIZE, 0.0);
  // distance_buffer_all_ = std::vector<double>(GLXY_SIZE, 0.0);
  /* ========== compute positive DT (distance transform outside the obstacles) ========== */
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
        [&](int y) {
          return gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())] == Occupied ?
              0.0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
        update_Y_SIZE, update_Y_SIZE+1);
  }
  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF(
      [&](int x) { 
        return tmp_buffer1_[x * update_Y_SIZE + y]; },
              [&](int x, double val) {
                distance_buffer_[x * update_Y_SIZE + y] = grid_interval_ * std::sqrt(val);
              },
              0, update_X_SIZE, update_X_SIZE+1);
  }
  /* ========== compute negative distance inside the obstacles ========== */
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
        [&](int y) {
          int state = gridmap_[(x+min_esdf.x()) * GLY_SIZE_ + (y+min_esdf.y())];
          return (state == Unoccupied || state == Unknown) ?
              0.0 :
              std::numeric_limits<double>::max();
        },
        [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; }, 0,
        update_Y_SIZE, update_Y_SIZE+1);
  }
  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF([&](int x) { return tmp_buffer1_[x * update_Y_SIZE + y]; },
              [&](int x, double val) {
                distance_buffer_neg_[x * update_Y_SIZE + y] = grid_interval_ * std::sqrt(val);
              },
              0, update_X_SIZE, update_X_SIZE+1);
  }
  /* ========== combine pos and neg DT ========== */
  for (int x = 0; x < update_X_SIZE; x++)
    for (int y = 0; y < update_Y_SIZE; y++){
        int global_idx = (x + min_esdf.x()) * GLY_SIZE_ + y + min_esdf.y();
        int idx =  x * update_Y_SIZE + y;
        distance_buffer_all_[global_idx] = distance_buffer_[idx];

        if (distance_buffer_neg_[idx] > 0.0)
          distance_buffer_all_[global_idx] += (-distance_buffer_neg_[idx] + grid_interval_);
      }
}

template <typename F_get_val, typename F_set_val>
void SDFmap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  int v[dim_size];
  double z[dim_size + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}


void SDFmap::publish_ESDF(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  const double min_dist = 0.0;
  const double max_dist = 5.0;
  int size = distance_buffer_all_.size();
  for(int i = 1; i< size;i++){
    Eigen::Vector2d coord = gridIndex2coordd(vectornum2gridIndex(i));
    pcl::PointXYZI pt;
    pt.x = coord.x();pt.y = coord.y();pt.z = 0.0;
    pt.intensity = std::max(min_dist, std::min(distance_buffer_all_[i],max_dist));
    cloud_vis.points.push_back(pt);
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "world";
  pub_ESDF_.publish(surf_vis);
}

inline double SDFmap::getDistance(const Eigen::Vector2i& id){
  // if(distance_buffer_all_[Index2Vectornum(id.x(), id.y())]>5){
  //   ROS_ERROR("out of map!!  %d  %d   distance:%f",id.x(),id.y(),distance_buffer_all_[Index2Vectornum(id[0],id[1])]);
  // }
  return distance_buffer_all_[Index2Vectornum(id[0],id[1])];
}

inline double SDFmap::getDistance(const int& idx, const int& idy){
  // if(distance_buffer_all_[Index2Vectornum(idx, idy)]>5){
  //   ROS_ERROR("out of map!!  %d  %d  distance:%f",idx,idy,distance_buffer_all_[Index2Vectornum(idx, idy)]);
  // }
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
}

inline Eigen::Vector2i SDFmap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_ - 0.5), 0), GLX_SIZE_ - 1),
         std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_ - 0.5), 0), GLY_SIZE_ - 1);
  return idx;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double& mindis){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  if(dist > mindis){
    return dist;
  }

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;


  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  return dist; 
}

double SDFmap::getDistanceReal(const Eigen::Vector2d& pos){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    return 10000;
  }
  Eigen::Vector2i idx = coord2gridIndex(pos);
  return distance_buffer_all_[idx.x() * GLY_SIZE_ + idx.y()];
}

double SDFmap::getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x()<global_x_lower_||pos.y()<global_y_lower_||pos.x()>global_x_upper_||pos.y()>global_y_upper_){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! pos:%f  %f  %f",pos.x(),pos.y(),pos.z());
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = coord2gridIndex(pos_m);
  if(idx.x()>=GLX_SIZE_-1||idx.y()>=GLY_SIZE_-1){
    grad.setZero();
    // ROS_ERROR("[getDistWithGradBilinear], coord out of map!!! idx:%d  %d  %d",idx.x(),idx.y(),idx.z());
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = gridmap_[Index2Vectornum(current_idx)]==Unknown?1:0;
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

Eigen::Vector2d SDFmap::get_update_odom(){
  return update_odom_;
}

inline double SDFmap::normalize_angle(double angle){
  if(angle>M_PI) angle -= 2*M_PI;
  if(angle<-M_PI) angle += 2*M_PI;
  return angle;
}

bool SDFmap::isOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Occupied;
}

bool SDFmap::isOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
}

bool SDFmap::isUnOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
}

bool SDFmap::isUnOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
}

bool SDFmap::isUnknown(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unknown;
}

bool SDFmap::isUnknown(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
}

bool SDFmap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
}

bool SDFmap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(idx, idy)] < safe_dis;
}

void SDFmap::publish_ESDFGrad(){
    visualization_msgs::MarkerArray grad_all;
    
    Eigen::Vector2i min_cut(0,0);
    Eigen::Vector2i max_cut(GLX_SIZE_-1, GLY_SIZE_-1);

    for (int x = min_cut(0)+2; x < max_cut(0); x+=5)
      for (int y = min_cut(1)+2; y < max_cut(1); y+=5) {
        Eigen::Vector2d pos = gridIndex2coordd(x,y);        
        Eigen::Vector2d d(0.025,0.025);
        pos = pos + d;
        Eigen::Vector2d grad;
        getDistWithGradBilinear(pos,grad);
        visualization_msgs::Marker grad_Marker;
        grad_Marker.header.frame_id = "world";
        grad_Marker.header.stamp = ros::Time::now();
        grad_Marker.ns = "world";
        grad_Marker.type = visualization_msgs::Marker::ARROW;
        grad_Marker.action = visualization_msgs::Marker::ADD;
        grad_Marker.id = (x+1) + (y*100000);
        grad_Marker.pose.position.x = pos[0];
        grad_Marker.pose.position.y = pos[1];
        grad_Marker.pose.position.z = 0.0;
        
        Eigen::Quaterniond Quat;
        Eigen::Vector3d vectorBefore(1, 0, 0);
        Eigen::Vector3d vectorAfter(grad.x(), grad.y(), 0.0);
        Quat = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

        grad_Marker.pose.orientation.w = Quat.w();
        grad_Marker.pose.orientation.x = Quat.x();
        grad_Marker.pose.orientation.y = Quat.y();
        grad_Marker.pose.orientation.z = Quat.z();

        grad_Marker.scale.x = (grad.norm()+0.01)/10.0;
        if(grad.norm()>1000){
          ROS_INFO("grad error!!!  position: %f  %f    grad:%f  %f   grad.norm(): %f",pos.x(),pos.y(),grad.x(),grad.y(),grad.norm());
        }
        grad_Marker.scale.y = 0.01;
        grad_Marker.scale.z = 0.01;

        grad_Marker.color.r = 0.0f;
        grad_Marker.color.g = 1.0f;
        grad_Marker.color.b = 0.0f;
        grad_Marker.color.a = 1.0;
        grad_Marker.lifetime = ros::Duration(100.0);
        grad_Marker.frame_locked = true;

        grad_all.markers.push_back(grad_Marker);
      }

    pub_gradESDF_.publish(grad_all);

}
