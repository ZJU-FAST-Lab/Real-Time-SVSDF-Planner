
#include "planner_algorithm/front_end_astar.hpp"
#include <unordered_set>
#include "Eigen/Dense"

//对index的处理没有问题
AstarPathSearcher::AstarPathSearcher(ros::NodeHandle &nh,
                                     std::shared_ptr<SDFmap> sdfmap, 
                                     std::shared_ptr<sv_manager> sv_manager)
{
    nh_ = nh;
    sdfmap_ = sdfmap;
    sv_manager_ = sv_manager;

    terminatePtr = nullptr;

    GLX_SIZE_ = sdfmap_->GLX_SIZE_ / sample_resolution_; //gridmap地图分辨率过高，降低一点用来搜索
    GLY_SIZE_ = sdfmap_->GLY_SIZE_ / sample_resolution_;
    GLZ_SIZE_ = sv_manager_->shape_->degree_divisions_;

    ROS_INFO("\033[0;30;47m    A* initialize  \033[0m");
    std::cout<<"map size:" << GLX_SIZE_ <<","<<GLY_SIZE_ <<","<<GLZ_SIZE_<<std::endl;

    GridNodeMap.resize(GLX_SIZE_ * GLY_SIZE_ * GLZ_SIZE_);
    for(int i = 0; i < GLX_SIZE_; i++)
    {
        for(int j = 0; j < GLY_SIZE_; j++)
        {
            for(int k = 0; k < GLZ_SIZE_; k++)
            {
                Eigen::Vector3i temp_index(i, j, k);
                Eigen::Vector2d temp_index_double(i, j);
                Eigen::Vector2d temp_position = temp_index_double*static_cast<double>(sample_resolution_)*static_cast<double>(sdfmap_->grid_interval_) + Eigen::Vector2d(sdfmap_->global_x_lower_,sdfmap_->global_y_lower_);
                bool is_occ = sv_manager_->is_shape_kernel_valiable(i, j, k);
                GridNodeMap[get1DIndex(i,j,k)] = std::make_unique<GridNode>(temp_index, temp_position, !is_occ);
            }
        }
    }
    ROS_INFO("\033[0;30;47m    A* initialize completed \033[0m");

    pieces_ = sv_manager_->shape_->degree_divisions_;       
    resolution_ = 2.0*M_PI/pieces_;

    grid_interval_ = sdfmap_->grid_interval_;
    global_x_lower_ = sdfmap_->global_x_lower_;
    global_y_lower_ = sdfmap_->global_y_lower_;

    neighbor_offsets_ = {
        Eigen::Vector2i(0, 1),   // 上
        Eigen::Vector2i(0, -1),  // 下
        Eigen::Vector2i(-1, 0),  // 左
        Eigen::Vector2i(1, 0)    // 右
    };

    nh_.param<int>("horizen", horizen_, 15);
}

void AstarPathSearcher::reset()
{
    for(int i = 0; i < GLX_SIZE_; i++)
    {
        for(int j = 0; j < GLY_SIZE_; j++)
        {
            for(int k = 0; k < GLZ_SIZE_; k++)
            {
                GridNodeMap[get1DIndex(i,j,k)]->reset();
            }
        }
    }
    openSet.clear();
    success_flag = false;
}

double AstarPathSearcher::getHeu(GridNode* node1, GridNode* node2) const
{
    double cost;
    const double p = 0.001;

    Eigen::Vector2i d = node1->index.head<2>()-node2->index.head<2>();
    int dx = abs(d(0)), dy = abs(d(1));
    int dmin = std::min(dx, dy); 
    int dmax = std::max(dx, dy); 
    double h = sqrt(2) * dmin + (dmax - dmin);

    cost = h*(1+p);
    return cost;
}

void AstarPathSearcher::AstarGetSucc(GridNode* currentPtr, std::vector<GridNode*> & neighborPtrSets, std::vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();           
    bool cond = false;

    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            for (int k = -1; k < 2; k ++)
            {
                Eigen::Vector3i vi(i,j,k);
                vi = vi + currentPtr -> index;
                if (vi(0) < 0 || vi(0) >= GLX_SIZE_ || vi(1) < 0 || vi(1) >= GLY_SIZE_) 
                {
                    continue;
                }
                if (vi(2)<0)  
                {
                    vi(2) = GLZ_SIZE_ - 1;
                }                                                          
                else if (vi(2)>=GLZ_SIZE_)
                {
                    vi(2) = 0;
                }                                                  
                    
                GridNode* p = GridNodeMap[get1DIndex(vi(0),vi(1),vi(2))].get();
                if(p->is_occupied)
                {
                    continue;
                }
                Eigen::Vector3i VI(vi(0)*sample_resolution_, vi(1)*sample_resolution_,vi(2));
                cond = !sdfmap_ -> isOccupied(VI.head<2>());
                cond = cond && sv_manager_->is_shape_kernel_valiable(VI(0), VI(1), VI(2));
                if (cond)
                {
                    neighborPtrSets.push_back(p);
                    edgeCostSets.push_back(sqrt(i*i + j*j + k*k));
                    //std::cout << "neighborPtr -> index :"<<p -> index <<std::endl;
                }
                else
                {
                    p->is_occupied = true;
                }
            }  
        }
    }
}

void AstarPathSearcher::AstarPathSearch(Eigen::Vector2d start, Eigen::Vector2d end , double start_yaw)
{  
    reset();
    start_<<start(0), start(1), start_yaw;
    end_ << end(0), end(1), 0.0; 
    total_time        = 0.0;
    total_kernel_time = 0.0;
    ros::Time time_1 = ros::Time::now();
    // 初始点和末端点不再其中
    if( !sdfmap_->isInGloMap(start) || !sdfmap_->isInGloMap(end) )
    {
       ROS_ERROR("[A*] start or target position is out of map.");
       success_flag = false;
       return;
    }
    // ROS_INFO("\033[0;30;47m    A* start  \033[0m");
    // ROS_INFO("\033[1;3;37m start point: \033[0m %.10f %.10f ", start(0), start(1));
    // ROS_INFO("\033[1;3;37m end point  : \033[0m %.10f %.10f ", end(0), end(1));

    double sample_resolution_half = sample_resolution_ / 2.0;
    Eigen::Vector2d offset(sample_resolution_half, sample_resolution_half);
    Eigen::Vector2d start_double = sdfmap_->coord2gridIndex(start).cast<double>();
    Eigen::Vector2i start_idx = ((start_double + offset) / sample_resolution_).cast<int>();

    // 计算 end_idxstart_yaw
    Eigen::Vector2d end_double = sdfmap_->coord2gridIndex(end).cast<double>();
    Eigen::Vector2i end_idx = ((end_double + offset) / sample_resolution_).cast<int>();

    // ROS_INFO("\033[1;3;37m start index: \033[0m %d %d ", start_idx(0), start_idx(1));
    // ROS_INFO("\033[1;3;37m end index  : \033[0m %d %d ", end_idx(0), end_idx(1));

    Eigen::Vector2i goalIdx = end_idx;

    //position of start_point and end_point
    // Eigen::Vector2d start_pt = sdfmap_->gridIndex2coordd(start_idx * sample_resolution_);
    // Eigen::Vector2d end_pt   = sdfmap_->gridIndex2coordd(end_idx   * sample_resolution_);
    
    int yaw_index;
    double sin_angle = std::sin(start_yaw);
    double cos_angle = std::cos(start_yaw);
    double yaw_nomalized = std::atan2(sin_angle, cos_angle);
    yaw_index =   (std::fmod(yaw_nomalized  + M_PI + resolution_/2, M_PI*2)) / resolution_;
    std::cout<<"first point:"<<yaw_index<<std::endl;
    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNode* startPtr = GridNodeMap[get1DIndex(start_idx(0), start_idx(1), yaw_index)].get();
    GridNode* endPtr = GridNodeMap[get1DIndex(end_idx(0), end_idx(1), 0)].get();

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNode* currentPtr  = nullptr;
    GridNode* neighborPtr = nullptr;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; 

    openSet.insert( std::make_pair(startPtr -> fScore, startPtr) );
 
    // GridNodeMap[get1DIndex(start_idx(0),start_idx(1))] -> id = startPtr -> id; 
    // GridNodeMap[get1DIndex(start_idx(0),start_idx(1))] -> gScore = startPtr -> gScore;
    // GridNodeMap[get1DIndex(start_idx(0),start_idx(1))] -> fScore = startPtr -> fScore;

    std::vector<GridNode*> neighborPtrSets;
    std::vector<double> edgeCostSets;


    // this is the main loop
    while ( !openSet.empty() ){
        auto iter  = std::begin(openSet);
        currentPtr = iter -> second;
        openSet.erase(iter);
        currentPtr -> id = -1;
        // std::cout << "currentPtr -> index :"<<currentPtr -> index <<std::endl;

        // if the current node is the goal 
        if( currentPtr -> index.head<2>() == goalIdx ){
            terminatePtr = currentPtr;
            success_flag = true;
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            
            neighborPtr = neighborPtrSets[i];
            double ec = edgeCostSets[i];  
            
            if(neighborPtr -> id == 0){

                double tg = ec + currentPtr -> gScore; 
                
                neighborPtr -> father = currentPtr;
                neighborPtr -> gScore = tg;
                neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr);

                neighborPtr -> id = 1;
                openSet.insert( std::make_pair(neighborPtr -> fScore, neighborPtr) );
                      
                continue;
            }
        
            else if(neighborPtr -> id == 1){ 
                double tg = ec + currentPtr->gScore;
                if (tg < neighborPtr->gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr);
                }
                continue;
            }
        
            else{

                double tg = ec + currentPtr->gScore;
                if(tg < neighborPtr -> gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr);
                     
                    neighborPtr -> id = 1;
                    openSet.insert( std::make_pair(neighborPtr -> fScore, neighborPtr) );
                }
                continue;
            }
        }      
    }
    //if search fails
    success_flag = false;
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

struct Vector2dHash {
    std::size_t operator()(const Eigen::Vector2i& v) const {
        std::size_t h1 = std::hash<int>()(v.x());
        std::size_t h2 = std::hash<int>()(v.y());
        return h1 ^ (h2 << 1);
    }
};

struct Vector2dEqual {
    bool operator()(const Eigen::Vector2i& a, const Eigen::Vector2i& b) const {
        return a.x() == b.x() && a.y() == b.y();
    }
};

bool AstarPathSearcher::getPathAndCollisionPoints(
    std::vector<Eigen::Vector3d> &path, 
    std::vector<Eigen::Vector2d> &collision_points)
{
    path.clear();
    collision_points.clear();
    if (!terminatePtr) {
        ROS_ERROR("Terminate pointer is null. No path found.");
        return false;
    }

    int yaw_index;
    if(start_(2) < -M_PI)
    {
        yaw_index = - (-M_PI - start_(2) + resolution_/2 ) / resolution_;
    }else
    {
        yaw_index =   (start_(2)  + M_PI + resolution_/2) / resolution_;
    }// 加上0.01保证精度

    std::vector<GridNode*> gridPath;
    GridNode* p = terminatePtr;

    while (p->father != nullptr) {
        gridPath.push_back(p);
        p = p->father;
    }
    gridPath.push_back(p);
    double last_yaw_index = p->index(2);
    // std::cout<<"yaw_index"<<yaw_index<<std::endl;
    // std::cout<<"last_yaw_index"<<last_yaw_index<<std::endl;
    std::reverse(gridPath.begin(), gridPath.end());

    if (gridPath.size() > horizen_) {
        gridPath.resize(horizen_);
    }

    // 使用 unordered_set 来存储唯一的碰撞点
    std::unordered_set<Eigen::Vector2i, Vector2dHash, Vector2dEqual> collision_set;
    // ROS_INFO("\033[0;30;42m Front End Result Detailed \033[0m ");
    for (auto ptr : gridPath) {
        Eigen::Vector3d coord;
        coord.head<2>() = ptr->coord;
        if(ptr->index(2) - last_yaw_index == 0)
        {
            coord(2) = yaw_index*resolution_ - M_PI;
        }
        else if(ptr->index(2) - last_yaw_index == 1 || ptr->index(2) - last_yaw_index == -(pieces_ -1))
        {
            yaw_index ++;
            coord(2) = yaw_index*resolution_ - M_PI;
        }else{
            yaw_index --;
            coord(2) = yaw_index*resolution_ - M_PI;
        }
        last_yaw_index = ptr->index(2);
        path.push_back(coord);

        // //输出一下点的情况
        // std::cout<<ptr->index(0)* sample_resolution_<<" "<<ptr->index(1)* sample_resolution_<<" "<<ptr->index(2)<<std::endl;
        // std::cout<<ptr->coord(0)<<" "<<ptr->coord(1)<< " "<< coord(2)<<std::endl;
        // std::cout<<sv_manager_->is_shape_kernel_valiable(ptr->index(0)* sample_resolution_, ptr->index(1)* sample_resolution_,ptr->index(2))<<std::endl<<std::endl;
        
        std::vector<Eigen::Vector2i> collision_point_index;
        collision_point_index = sv_manager_->get_collision_points(
            ptr->index(0) * sample_resolution_, 
            ptr->index(1) * sample_resolution_, 
            ptr->index(2)
        );
        
        for (const auto& point : collision_point_index) 
        {
            collision_set.insert(point);
        }
    }

    auto it = collision_set.begin();
    while (it != collision_set.end()) 
    {
        int neighbor_present = 0;
        for (const auto& offset : neighbor_offsets_) 
        {
            Eigen::Vector2i neighbor = *it + offset;
            if (collision_set.find(neighbor) != collision_set.end()) 
            {
                neighbor_present++;
            }
            if (neighbor_present >= 3) 
            {
                break;
            }
        }
        if (neighbor_present >= 3) 
        {
            it = collision_set.erase(it); // erase 返回下一个有效迭代器
        } else 
        {
            ++it;
        }
    }

    //把index转换回到double中
    for (const auto& grid_idx : collision_set) 
    {
        double x_collision = grid_idx.x() * grid_interval_ + 
                             grid_interval_ / 2 + 
                             global_x_lower_;

        double y_collision = grid_idx.y() * grid_interval_ + 
                             grid_interval_ / 2 + 
                             global_y_lower_;

        Eigen::Vector2d point(x_collision, y_collision);
        collision_points.push_back(point);
    }


    // 暂时取消了降采样，记得改。 
    if (!path.empty() && path.size() > 10) {
        std::vector<Eigen::Vector3d> downsampled_path;
        for (size_t i = 0; i < 1; i += 1) {
            downsampled_path.push_back(path[i]);
        }
        for (size_t i = 1; i < path.size() - 2; i += 2) {
            downsampled_path.push_back(path[i]);
        }
        for (size_t i = path.size() - 2; i < path.size() - 1; i += 1) {
            downsampled_path.push_back(path[i]);
        }
        path = downsampled_path;
    }

    path[0] = start_;
    std::cout<<start_(0)<<" "<<start_(1)<< " "<< start_(2)<<std::endl;

    return true;
}