#include "swept_volume/swept_volume_manager.hpp"
#include <cassert>
#include <algorithm>

bool sv_manager::is_shape_kernel_valiable(int x, int y, int shape_kernel_index)
{
    if(x + 16 <= 0 || 
       x - 16 >= sdfmap_->map_kernel_->cols*32 || 
       y - 16 >= sdfmap_->map_kernel_->rows ||
       y + 16 <= 0) 
    {
        return true;
    }
    else
    {
        int y_min = std::max(y - 16, 0);
        int y_max = std::min(y + 15, sdfmap_->map_kernel_->rows - 1);
        int x_min = std::floor((x - 16) * 1.0 / 32);
        int shape_row_index = y_min - (y -16);

        for(int row = y_min; row <= y_max; row++)
        {
            uint32_t shape_kernel_row = shape_->shape_kernel_list_[shape_kernel_index][shape_row_index];
            shape_row_index ++;
            // 组合当前列和下一个列的位图为 64 位 mask
            uint64_t map_mask = 0;
            if(x_min < sdfmap_->map_kernel_->cols && x_min >=0)
                map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min]);
            if((x_min + 1) < sdfmap_->map_kernel_->cols && (x_min + 1) >= 0)
                map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min + 1]) << 32;
            
            uint64_t shifted_shape = static_cast<uint64_t>(shape_kernel_row) << ((x+16) % 32);
            
            if((shifted_shape & map_mask) != 0)
                return false;
        }
        
        return true;
    }
}

bool sv_manager::is_shape_kernel_valiable(Eigen::Vector3d pose)
{
    int x = int((pose(0)+10.0) / 0.1);
    int y = int((pose(1)+10.0) / 0.1);

    double sin_angle = std::sin(pose(2));
    double cos_angle = std::cos(pose(2));
    double yaw_nomalized = std::atan2(sin_angle, cos_angle);
    int shape_kernel_index = (yaw_nomalized + M_PI + 0.1) / shape_kernel_resolution_;

    int y_min = std::max(y - 16, 0);
    int y_max = std::min(y + 15, sdfmap_->map_kernel_->rows - 1);
    int x_min = std::floor((x - 16) * 1.0 / 32);
    int shape_row_index = y_min - (y -16);

    for(int row = y_min; row <= y_max; row++)
    {
        uint32_t shape_kernel_row = shape_->shape_kernel_list_[shape_kernel_index][shape_row_index];
        shape_row_index ++;
        // 组合当前列和下一个列的位图为 64 位 mask
        uint64_t map_mask = 0;
        if(x_min < sdfmap_->map_kernel_->cols && x_min >=0)
            map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min]);
        if((x_min + 1) < sdfmap_->map_kernel_->cols && (x_min + 1) >= 0)
            map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min + 1]) << 32;
        
        uint64_t shifted_shape = static_cast<uint64_t>(shape_kernel_row) << ((x+16) % 32);
        
        if((shifted_shape & map_mask) != 0)
            return false;
    }
    return true;
}

std::vector<Eigen::Vector2i> sv_manager::get_collision_points(int x, int y, int shape_kernel_index)
{
    std::vector<Eigen::Vector2i> collision_points;
    // while(shape_kernel_index >= shape_->degree_divisions_)
    //     shape_kernel_index -= shape_->degree_divisions_;
    // while(shape_kernel_index < 0)
    //     shape_kernel_index += shape_->degree_divisions_;
    // 检查 sdfmap_ 是否为空
    // if (!sdfmap_ || !sdfmap_->map_kernel_) {
    //     ROS_ERROR("sdfmap_ 或 map_kernel_ 未初始化！");
    //     return {};
    // }

    // // 检查 shape_kernel_index 是否越界
    // if (shape_kernel_index < 0 || shape_kernel_index >= shape_->shape_kernel_inflate_list_.size()) {
    //     ROS_ERROR("shape_kernel_index 越界: %d", shape_kernel_index);
    //     return {};
    // }

    int y_min = std::max(y - 16, 0);
    int y_max = std::min(y + 15, sdfmap_->map_kernel_->rows - 1);
    int x_min = std::floor((x - 16) * 1.0 / 32);
    int shape_row_index = y_min - (y -16);

    for(int row = y_min; row <= y_max; row++)
    {

        uint32_t shape_kernel_row = shape_->shape_kernel_inflate_list_[shape_kernel_index][shape_row_index];
        shape_row_index ++;
        // 组合当前列和下一个列的位图为 64 位 mask
        uint64_t map_mask = 0;
        if(x_min < sdfmap_->map_kernel_->cols && x_min >=0)
            map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min]);
        if((x_min + 1) < sdfmap_->map_kernel_->cols && (x_min + 1) >= 0)
            map_mask |= static_cast<uint64_t>(sdfmap_->map_kernel_->map_kernel_[row][x_min + 1]) << 32;
        
        uint64_t shifted_shape = static_cast<uint64_t>(shape_kernel_row) << ((x+16) % 32);
        
        uint64_t collision_mask = shifted_shape & map_mask;
        if(collision_mask != 0)
        {
            while(collision_mask != 0)
            {
                // 找到最低位的1
                unsigned long bit_index;
                // __builtin_ctzll 在 collision_mask 为 0 时行为未定义，因此已在外部检查 collision_mask != 0
                bit_index = __builtin_ctzll(collision_mask);

                // 计算碰撞点的 x 和 y 坐标 
                int collision_x = x_min * 32 + static_cast<int>(bit_index);
                int collision_y = row;
                // double x_collision = collision_x*sdfmap_->grid_interval_ + sdfmap_->grid_interval_/2 + sdfmap_->global_x_lower_;
                // double y_collision = collision_y*sdfmap_->grid_interval_ + sdfmap_->grid_interval_/2 + sdfmap_->global_y_lower_;
                
                collision_points.emplace_back(Eigen::Vector2i(collision_x, collision_y));

                // 清除最低位的1
                collision_mask &= (collision_mask - 1);
            }
        }
    }
    return collision_points;
}

// back end
void sv_manager::updateBsplineTraj(const std::shared_ptr<Bspline> new_traj)
{
    this->BSplineTraj_ = new_traj;
    double td = BSplineTraj_->getTotalDuration();
    if (td < 3*1e2)
    {
        traj_duration_ = td;
        t_max_ = td;
    }
}


void sv_manager::gradientDescentBspline(double momentum, double t_min, double t_max,
                            const double x0, double &fx, double &x, const Eigen::Vector3d &pos_eva, bool debug)
{
    assert((t_max > 0) && (t_max < 1000) && " in gradient descent,t_max must > 0 and t_max <100");
    assert((t_min >= 0) && " in gradient descent,t_min must >=0");
    assert((momentum >= 0) && (momentum <= 1) && "momentum must between 0~1");

    int max_iter = 1000;
    double alpha = 0.01; 
    double tau = alpha;
    double g = 0.0; 
    double tol = 1e-16;
    double min = t_min;
    double max = t_max;
    double xmin = x0; 
    double xmax = x0;
    x = x0;

    double projection = 0;
    double change = 0;

    double prev_x = 10000000.0;
    int iter = 0;
    bool stop = false;
    double x_candidate;
    double fx_candidate;

    g = 100.0;
    while (iter < max_iter && !stop && abs(x - prev_x) > tol)
    {
        xmin = std::min(xmin, x);
        xmax = std::max(xmax, x);

        if (iter == 0)
        {
            fx = getSDFAtTimeStampBspline<useScale>(pos_eva, x);    //改好了
        }
        g = getSDF_DOTAtTimeStampBspline<useScale>(pos_eva, x);     //改好了

        tau = alpha;
        prev_x = x;
        for (int div = 1; div < 30; div++) 
        {
            iter = iter + 1;
            assert(iter < max_iter && "不满足iter < max_iter");
            projection = x;
            g = getSDF_DOTAtTimeStampBspline<useScale>(pos_eva, projection);
            change = -tau * ((int)(g > 0) - (g < 0));
            x_candidate = x + change;
            x_candidate = std::max(std::min(x_candidate, t_max), t_min);
            fx_candidate = getSDFAtTimeStampBspline<useScale>(pos_eva, x_candidate);
            if ((fx_candidate - fx) < 0)
            {
                x = x_candidate;
                fx = fx_candidate;
                break;
            }
            tau = 0.5 * tau;
            if (div == 29)
            {
                stop = true;
            }
        }
    }
}



// 实用工具
void sv_manager::getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt)
{
    // xt = BSplineTraj_->getPos(time_stamp);
    // vt = BSplineTraj_->getVel(time_stamp);
    BSplineTraj_->getPosAndVel(time_stamp, xt, vt);

    double yaw = xt(2);
    Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    VRt = Eigen::Matrix3d::Identity();
    double syaw = sin(yaw);
    double cyaw = cos(yaw);
    VRt(0, 0) = -syaw;
    VRt(0, 1) = -cyaw;
    VRt(1, 0) = cyaw;
    VRt(1, 1) = -syaw;
}

void sv_manager::getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt,
                                    Eigen::Matrix3d &St,
                                    Eigen::Matrix3d &dSt)
{
    assert((time_stamp > traj_duration_) && ("[getStateOnTrajStamp] 传入的time_stamp参数大于轨迹的总时长。"));
    // xt = BSplineTraj_->getPos(time_stamp);
    // vt = BSplineTraj_->getVel(time_stamp);
    BSplineTraj_->getPosAndVel(time_stamp, xt, vt);

    double yaw = xt(2);
    Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    VRt = Eigen::Matrix3d::Identity();
    double syaw = sin(yaw);
    double cyaw = cos(yaw);
    VRt(0, 0) = -syaw;
    VRt(0, 1) = -cyaw;
    VRt(1, 0) = cyaw;
    VRt(1, 1) = -syaw;

    St = getScale(time_stamp);
    dSt = getDotScale(time_stamp);
}

void sv_manager::getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt)
{

    xt = BSplineTraj_->getPos(time_stamp);

    double yaw = xt(2);
    Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void sv_manager::getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &St)
{
    assert((time_stamp < traj_duration_) && ("[getStateOnTrajStamp] 传入的time_stamp参数大于轨迹的总时长。"));
    xt = BSplineTraj_->getPos(time_stamp);

    double yaw = xt(2);
    Rt = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    St = getScale(time_stamp);
}

/**
 * 获得轨迹上具体某一时刻的机器人尺寸因子
 * @param time_stamp 时刻
 * @return time_stamp 时刻机器人尺寸因子
 * @attention 当前返回单位矩阵，需要按需修改,注意奇异点问题Scale缩放分母除以0，求inverse问题
 */
Eigen::Matrix3d sv_manager::getScale(const double t)
{
    return Eigen::Matrix3d::Identity();
}

/**
 * 获得轨迹上具体某一时刻的机器人尺寸因子变化导数
 * @param time_stamp 时刻
 * @return time_stamp 时刻机器人尺寸因子
 * @attention 当前返回0矩阵，需要按需修改
 */
Eigen::Matrix3d sv_manager::getDotScale(const double t) 
{
    Eigen::Matrix3d St = Eigen::Matrix3d::Zero();
    return St;
}

Eigen::Vector3d sv_manager::posEva2Rel(const Eigen::Vector3d &pos_eva,
                           const Eigen::Vector3d &xt,
                           const Eigen::Matrix3d &Rt)
{
    return (Rt.transpose() * (pos_eva - xt));
}

Eigen::Vector3d sv_manager::posEva2Rel(const Eigen::Vector3d &pos_eva,
                           const Eigen::Vector3d &xt,
                           const Eigen::Matrix3d &Rt,
                           const Eigen::Matrix3d &St)
{
    // return (pos_eva - xt);
    return (Rt.transpose() * St.inverse() * (pos_eva - xt));
    // 获取的是在物体坐标系下的障碍物点的位置，Rt.transpose()就是Rt的逆，旋转矩阵是左乘的
}

