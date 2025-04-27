#ifndef BSPLINE_OPTIMIZER_HPP
#define BSPLINE_OPTIMIZER_HPP

#include "utils/NURBS.hpp"
#include <ros/ros.h>
#include <utils/visualization.hpp>
#include "swept_volume/swept_volume_manager.hpp"
#include <utils/lbfgs.hpp>
#include <utils/lmbm.h>
#include <cmath>
#include <Eigen/Dense>
#include <cassert>
#include <thread>

// 这个优化器混合了中端和后端的内容，方便管理我新增的代码

class TrajOptimizer{
public:
    std::shared_ptr<Bspline> trajectory_;                 // 属于自己的，优化器对轨迹进行优化，所以包含一个NURBS轨迹类的定义
    
    int parallel_points_num_;
    std::vector<Eigen::Vector2d> parallel_points_;   // 存放避障点，AABB获取的点，我决定沿用之前的。

    ros::NodeHandle nh_;                             //调用的
    std::shared_ptr<sv_manager> sv_manager_;                          
    std::shared_ptr<vis::Visualization> visualizer_;                    // 用于可视化的指针

    double max_acc_ = 0.6;
    double max_vel_ = 0.3;
    double omg_max_ = 0.6;
    double alpha_max_ = 1.2;

    double safe_threshold_ = 0.073;
    double resolution_ = 0.2;
    int threads_num_ = 16;

    bool exit_ = false;              
    bool pause_ = true;
    bool next_step_ = false;
    double* x_ptr;


public:
    TrajOptimizer(ros::NodeHandle &nh, std::shared_ptr<sv_manager> sv_manager, std::shared_ptr<vis::Visualization> visualizer);
    ~TrajOptimizer() {}

    void visTrajectory();

    //中端，可以把一系列的waypoints转换成轨迹
    void calculateTs(double &ts, std::vector<Eigen::Vector3d> &path);
    void InitializeBspline(std::vector<Eigen::Vector3d> &waypoints, 
                           std::vector<Eigen::Vector3d> &start_end_derivative,
                           std::vector<Eigen::Vector2d> &parallel_points);

    //后端，等待完善
    int optimize_lbfgs();
    static double evaluate(void *ptr, 
                           const Eigen::VectorXd &x, 
                           Eigen::VectorXd &g);
    void calcSweptVolumeCost(void *ptr, double &cost, Eigen::MatrixXd &gradient);
    void calcSweptVolumeCostParallel(void *ptr, double &cost, Eigen::MatrixXd &gradient);
    double calcCostofSDFValue(double sdf_value, double& dCost_dSDF);
    void calcYawGrad(const Eigen::Vector3d& pos_eva,
                  const Eigen::Vector3d& Xt,
                  const double& yaw,
                  const double dCost_dSDF,
                  double &d_yaw);
    void calcQGrad(const Eigen::Vector3d dCost_dXt,
                   const double d_yaw, 
                   const double time_star,
                   const Bspline::Ptr trajecotry, 
                   Eigen::MatrixXd &gradient);
    void calcSmoothnessCost(const Eigen::MatrixXd &q, 
                            double &cost, 
                            Eigen::MatrixXd &gradient, 
                            bool falg_use_jerk /* = true*/);
    void calcFeasibilityCost(void *ptr, 
                             const Eigen::MatrixXd &q, 
                             double &cost, 
                             Eigen::MatrixXd &gradient);
    void Time_ReAllocate();
    static int progress(void *ptr,
                        const Eigen::VectorXd &x,
                        const Eigen::VectorXd &g,
                        const double fx,
                        const double step,
                        const int k,
                        const int ls);

    // lmbm版本的后端
    int optimize_lmbm();
    static double costFunctionLmbmParallel(void *ptr,
                                           const double *x,
                                           double *g,
                                           const int n);
    static int earlyExitLMBM(void *instance,
                             const double *x,
                             const int k);

    //实用debug工具
    void AsyncSleepMS(const int ms);
    //用于重规划
    void get_the_predicted_state(double predicted_traj_start_time, 
                                 Eigen::Vector3d &plan_start_state_XYTheta_P,
                                 Eigen::Vector3d &plan_start_state_XYTheta_V,
                                 Eigen::Vector3d &plan_start_state_XYTheta_A);
};


#endif
