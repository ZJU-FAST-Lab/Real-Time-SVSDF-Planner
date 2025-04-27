#ifndef PLANNER_ALGORITHM_FRONT_END_ASTAR_HPP
#define PLANNER_ALGORITHM_FRONT_END_ASTAR_HPP

#include "Eigen/Dense"
#include "swept_volume/swept_volume_manager.hpp"
#include "plan_env/sdf_map.h"
#include "utils/shape.hpp"
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <memory>
#include <cmath>

class GridNode
{
    public:
        GridNode(){
            coord = Eigen::Vector2d(0,0);
            index = Eigen::Vector3i(0,0,0);
            father = nullptr;
            gScore = 0;
            fScore = 0;
            id     = 0;
            is_occupied = false;
        }
        GridNode(const Eigen::Vector3i& ind,const Eigen::Vector2d& cor, bool is_occ){
            coord = cor;
            index = ind;
            father = nullptr;
            gScore = 0;
            fScore = 0;
            id     = 0;
            is_occupied = is_occ;
        }
        ~GridNode(){}

        void reset()
        {
            father = nullptr;
            gScore = 0;
            fScore = 0;
            id     = 0;
        }
    
        Eigen::Vector2d coord;
        Eigen::Vector3i index;
        double gScore;
        double fScore;
        int id;
        GridNode* father;
        bool is_occupied = false;
};

class AstarPathSearcher
{ 
    public:
        ros::NodeHandle nh_;
        std::vector<std::unique_ptr<GridNode>> GridNodeMap;
        GridNode *terminatePtr;
        int GLX_SIZE_;  
        int GLY_SIZE_;
        int GLZ_SIZE_;
        int sample_resolution_ = 2;
        double grid_interval_;
        double global_x_lower_;
        double global_y_lower_;
        std::vector<Eigen::Vector2i> neighbor_offsets_;

        int pieces_;        //角度被离散成几个
        double resolution_ ;//每一个离散的度数间隔

        // 在类中添加声明
        double total_time = 0.0;
        double total_kernel_time = 0.0;
        int total_kernel = 1;

        Eigen::Vector3d start_;
        Eigen::Vector3d end_;
        bool success_flag;

        int horizen_;
    public:
        std::shared_ptr<SDFmap> sdfmap_;
        std::shared_ptr<sv_manager> sv_manager_;

        AstarPathSearcher(ros::NodeHandle &nh, std::shared_ptr<SDFmap> sdfmap , std::shared_ptr<sv_manager> sv_manager);
        ~AstarPathSearcher() = default;
        void reset();

        void AstarGetSucc(GridNode* currentPtr, std::vector<GridNode*> & neighborPtrSets, std::vector<double> & edgeCostSets);
        double getHeu(GridNode* node1, GridNode* node2) const;

        void AstarPathSearch(Eigen::Vector2d start, Eigen::Vector2d goal, double start_yaw);
        bool getPathAndCollisionPoints(std::vector<Eigen::Vector3d> &path, std::vector<Eigen::Vector2d> &collision_points);

        std::multimap<double, GridNode*> openSet;

    private:
        int get1DIndex(int x, int y, int z) const {
            return x * (GLY_SIZE_ * GLZ_SIZE_) + y * GLZ_SIZE_ + z;
        }
};



#endif