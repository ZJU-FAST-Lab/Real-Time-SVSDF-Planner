#include "planner_algorithm/BSpline_optimizer.hpp"
#include <cassert>
#include <assert.h>
#include "utils/lbfgs.hpp"
#include <omp.h>

/// @brief 全局调用初始化时
TrajOptimizer::TrajOptimizer(ros::NodeHandle &nh, std::shared_ptr<sv_manager> sv_manager, std::shared_ptr<vis::Visualization> visualizer) 
{
    nh_ = nh;
    sv_manager_ = sv_manager;
    visualizer_ = visualizer;
    trajectory_ = std::make_shared<Bspline>();
    threads_num_ = nh_.param<int>("threads_num", 16);
    max_acc_ = nh_.param<double>("max_acc", 0.6);
    max_vel_ = nh_.param<double>("max_vel", 0.3);
    omg_max_ = nh_.param<double>("omg_max", 0.6);
    alpha_max_ = nh_.param<double>("alpha_max", 1.2);
}

// step 0
void TrajOptimizer::visTrajectory(){
    std::vector<Eigen::Vector3d> route;
    route = trajectory_->getRoute();
    visualizer_->visBsplineTraj("trajectory", route, 1552999, true);                                      // caution
}


/// @brief 根据给定的waypoints确定Bspline的Interval
/// @param ts 每一段轨迹的时间的长度
/// @param path 要经过的点（最后一个维度是yaw）
void TrajOptimizer::calculateTs(double &ts, std::vector<Eigen::Vector3d> &path){
    ts = 0.0;
    assert(path.size() >= 3);

    // 遍历相邻路径点
    for (size_t i = 0; i < path.size() - 2; ++i) {
        // 当前点和下一个点
        Eigen::Vector3d curr = path[i];
        Eigen::Vector3d next = path[i + 1];
        // 计算两点间的距离
        double dx = std::abs(next(0) - curr(0));
        double dy = std::abs(next(1) - curr(1));
        double distance = std::sqrt(dx*dx + dy*dy);
        // 计算两点间的角度差（偏航角）
        double yaw_diff = std::abs(next(2) - curr(2));  // 偏航角的绝对差值
        // 计算所需时间：max{距离/vmax, 角度差/omgmax}
        double segment_time = std::max(distance / max_vel_, yaw_diff / omg_max_);

        // 累加到总时间
        ts += segment_time;
    }
    ts = ts;

    ts /= path.size() - 1;
}



/// @brief 将一系列waypoints和初末条件转换为控制点
/// @param ts 每一段轨迹的时间的长度
/// @param waypoints 要经过的点
/// @param start_end_derivative 初末导数约束 V0A0 VT AT
/// @param ctrl_pts 转换出来的控制点
void TrajOptimizer::InitializeBspline(std::vector<Eigen::Vector3d> &waypoints, 
                                      std::vector<Eigen::Vector3d> &start_end_derivative,
                                      std::vector<Eigen::Vector2d> &parallel_points)
{
    parallel_points_ = parallel_points;
    parallel_points_num_ = parallel_points.size();
    if(waypoints.size() <= 1)
        return;
    while(waypoints.size() < 5)
    {
        std::vector<Eigen::Vector3d> new_waypoints;
        for(size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            new_waypoints.push_back(waypoints[i]);
            Eigen::Vector3d midpoint = 0.5 * (waypoints[i] + waypoints[i + 1]);
            new_waypoints.push_back(midpoint);
        }
        new_waypoints.push_back(waypoints.back());
        waypoints = new_waypoints;
        ROS_WARN("Waypoints size was less than 5. Inserted midpoints between all pairs.");
    }
    assert(start_end_derivative.size() == 4);
    double ts = 0;
    calculateTs(ts, waypoints);
    assert(ts > 0.001);
    std::cout<<"ts"<<ts<<std::endl;
    int K = waypoints.size();

    // write A
    Eigen::Vector3d prow(1, 4, 1);
    Eigen::Vector3d vrow(-1, 0, 1);
    Eigen::Vector3d arow(1, -2, 1);


    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
    for (int i = 0; i < K; ++i)
    {
        A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose(); //位置点约束
    }
    

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i){
        bx(i) = waypoints[i](0);
        by(i) = waypoints[i](1);
        bz(i) = waypoints[i](2);
    }

    for (int i = 0; i < 4; ++i){
        bx(K + i) = start_end_derivative[i](0);
        by(K + i) = start_end_derivative[i](1);
        bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    Eigen::MatrixXd ctrl_pts;
    // convert to control pts
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();
    
    // SET Start and End Condition
    ctrl_pts.col(0) = waypoints[0] - start_end_derivative[0]*ts + start_end_derivative[2]*ts*ts/3.0;
    ctrl_pts.col(1) = waypoints[0]                           - start_end_derivative[2]*ts*ts/6.0;
    ctrl_pts.col(2) = waypoints[0] + start_end_derivative[0]*ts + start_end_derivative[2]*ts*ts/3.0;

    ctrl_pts.col(K - 1) = waypoints[K-1] - start_end_derivative[1]*ts + start_end_derivative[3]*ts*ts/3.0;
    ctrl_pts.col(K    ) = waypoints[K-1]                              - start_end_derivative[3]*ts*ts/6.0;
    ctrl_pts.col(K + 1) = waypoints[K-1] + start_end_derivative[1]*ts + start_end_derivative[3]*ts*ts/3.0;

    trajectory_->setUniformBspline(ctrl_pts, 3, ts);
    // visTrajectory();
}

// back end 部分

/// @brief 后端优化的核心调用函数
/// @param initState 初始的x,y,yaw
/// @param finalState 最终的x,y,yaw
/// @return lbfgs是否成功
int TrajOptimizer::optimize_lbfgs()
{
    lbfgs::lbfgs_parameter_t lbfgs_params;  //对lbfgs的参数进行初始化
    //lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 20;
    lbfgs_params.g_epsilon = 0.001;
    lbfgs_params.max_linesearch = 16;

    int num_cols = trajectory_->control_points_.cols();
    int start_col = 3; // 从第4列（索引3）开始
    int end_col = num_cols - 3; // 到倒数第4列（索引 num_cols-3）
    
    sv_manager_->updateBsplineTraj(trajectory_);

    assert(num_cols >= 6);
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(trajectory_->control_points_.data() + start_col * 3, 3 * (end_col - start_col));
    double fx = 0.0;    //cost数值
    ROS_INFO("\033[0;30;47m   L-BFGS Optimize start  \033[0m");
    int result = lbfgs::lbfgs_optimize(x, fx, &TrajOptimizer::evaluate,nullptr, &TrajOptimizer::progress, this, lbfgs_params);
    ROS_INFO("\033[1;3;37m result: \033[0m %d ", result);
    //Time_ReAllocate();
    return result;
}


double TrajOptimizer::evaluate(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g){
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;

    Eigen::Map<const Eigen::MatrixXd> Q(x.data(), 3, x.size() / 3);
    Eigen::MatrixXd &control_points = obj.trajectory_->control_points_;
    int total_cols = control_points.cols();
    control_points.block(0, 3, 3, total_cols - 6) = Q;
    int control_points_size = control_points.size();
        /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness = 0;
    double f_distance = 0;
    double f_feasibility = 0;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd grad_3D = Eigen::MatrixXd::Zero(3, control_points_size);

    obj.sv_manager_->updateBsplineTraj(obj.trajectory_);  //更新sv_manager的轨迹类
    obj.sv_manager_->updateBsplineTraj(obj.trajectory_);  //更新sv_manager的轨迹类

    obj.calcSweptVolumeCostParallel(ptr, f_distance, g_distance);
    obj.calcSmoothnessCost(control_points, f_smoothness, g_smoothness, false);
    // obj.calcFeasibilityCost(ptr, control_points, f_feasibility, g_feasibility);

    double f_combine = 0;
    f_combine = 1 * f_smoothness + 500 *f_distance +  0.01*f_feasibility;
    grad_3D = 1 * g_smoothness +  500 *g_distance +  0.01*g_feasibility;

    memcpy(g.data(), grad_3D.data() + 9, g.size() * sizeof(g[0]));
    return f_combine;
}

/// @brief 计算扫略体积避障的Cost与对所有控制点Q的梯度
/// @param ptr      优化器指针，包含一些参数
/// @param cost     【扫略体积避障的Cost】
/// @param gradient 【对所有控制点Q的梯度】
void TrajOptimizer::calcSweptVolumeCost(void *ptr, double &cost, Eigen::MatrixXd &gradient)
{
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;         //获取当前优化器的实例，里面包含了我们的轨迹
        
    double time_star = 0.0;                             //离扫略体积最近的时间t*
    Eigen::Vector3d pos_eva = Eigen::Vector3d::Zero();  //要评估的障碍物点
    Eigen::Vector3d Xt = Eigen::Vector3d::Zero();       //t*时刻轨迹上的点
    double sdf_value = 999.0;
    Eigen::Vector3d cor_point_3d;

    //对轨迹的
    double dCost_dSDF = 0.0; 
    Eigen::Vector3d dSDF_dXt{Eigen::Vector3d::Zero()}; 
    Eigen::Vector3d dCost_dXt{Eigen::Vector3d::Zero()};

    //对偏航角的
    double yaw = 0.0;
    double d_yaw = 0.0;

    bool set_ts = false;
    
    cost = 0.0;                 //总的cost初始化
    gradient.setZero();         //梯度初始化
    double pena = 0.0;          // 惩罚
    
    //对于每一个需要避障的点
    for (const Eigen::Vector2d& point : obj.parallel_points_)
    {
        pos_eva.head<2>() = point;
        pos_eva(2) = 0;

        sdf_value = obj.sv_manager_ -> getTrueSDFofBsplineSweptVolume<true>(pos_eva, time_star, dSDF_dXt, set_ts, cor_point_3d);  
        //here
        Xt = obj.trajectory_->getPos(time_star);
        yaw = Xt(2);
        dCost_dSDF = 0.0;

        Eigen::Matrix3d rotate = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
        dSDF_dXt(2) = 0;
        if(sdf_value > 0)
        {
            dSDF_dXt = rotate * (dSDF_dXt);
        }
            dSDF_dXt = dSDF_dXt.normalized();
              // 这是正确的变化的方向！！！！！
        
        pena = calcCostofSDFValue(sdf_value, dCost_dSDF);
        if(pena > 0)
        {
            dCost_dXt = dCost_dSDF * dSDF_dXt;
            calcYawGrad(pos_eva, Xt, yaw, dCost_dSDF, d_yaw);
            cost += pena;
            calcQGrad(dCost_dXt, d_yaw, time_star, obj.trajectory_, gradient);
        }
        // std::cout<<"point:"<<pos_eva<<std::endl;
        // std::cout<<"cost"<<pena<<std::endl;
        // std::cout<<"sdf"<<sdf_value<<std::endl;
    }
}

/// @brief 计算扫略体积避障的Cost与对所有控制点Q的梯度
/// @param ptr      优化器指针，包含一些参数
/// @param cost     【扫略体积避障的Cost】
/// @param gradient 【对所有控制点Q的梯度】
void TrajOptimizer::calcSweptVolumeCostParallel(void *ptr, double &cost, Eigen::MatrixXd &gradient)
{
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;  // 获取当前优化器的实例，里面包含了我们的轨迹

    // 初始化
    cost = 0.0;                 // 总的cost初始化
    gradient.setZero();         // 梯度初始化

    // 获取并行点的数量
    int num_points = obj.parallel_points_.size();
    if (num_points == 0) return; // 无需计算

    // 获取线程数量
    int num_threads = obj.threads_num_; // 假设 `threads_num` 已正确设置

    // 创建每个线程的私有 cost 和 gradient 副本
    std::vector<double> thread_cost(num_threads, 0.0);
    std::vector<Eigen::MatrixXd> thread_gradients(num_threads, Eigen::MatrixXd::Zero(gradient.rows(), gradient.cols()));

    // 并行化循环
    #pragma omp parallel for num_threads(num_threads) schedule(dynamic)
    for (int point_idx = 0; point_idx < num_points; ++point_idx)
    {
        // 获取线程编号
        int thread_num = omp_get_thread_num();
        // std::cout<<"thread_um"< <thread_num<<std::endl;
        // 获取当前评估的点
        Eigen::Vector2d point = obj.parallel_points_[point_idx];
        Eigen::Vector3d pos_eva;
        pos_eva << point, 0.0;

        // 初始化局部变量
        double time_star = 0.0;                      // 离扫略体积最近的时间t*
        Eigen::Vector3d Xt = Eigen::Vector3d::Zero(); // t*时刻轨迹上的点
        Eigen::Vector3d cor_point_3d;
        double sdf_value = 999.0;

        // 对轨迹的梯度
        double dCost_dSDF = 0.0;
        Eigen::Vector3d dSDF_dXt = Eigen::Vector3d::Zero();
        Eigen::Vector3d dCost_dXt = Eigen::Vector3d::Zero();

        // 对偏航角的梯度
        double yaw = 0.0;
        double d_yaw = 0.0;

        bool set_ts = false;

        // 计算 SDF
        sdf_value = obj.sv_manager_ -> getTrueSDFofBsplineSweptVolume<true>(pos_eva, time_star, dSDF_dXt, set_ts, cor_point_3d);
        Xt = obj.trajectory_->getPos(time_star);
        yaw = Xt(2);
        dCost_dSDF = 0.0;

        // 计算旋转矩阵
        Eigen::Matrix3d rotate = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        dSDF_dXt(2) = 0.0;

        if(sdf_value > 0)
        {
            dSDF_dXt = rotate * (dSDF_dXt);
        }
            dSDF_dXt = dSDF_dXt.normalized(); // 这是正确的变化的方向！！！???

        // 计算惩罚和梯度
        double pena = calcCostofSDFValue(sdf_value, dCost_dSDF);
        if (pena > 0)
        {
            dCost_dXt = dCost_dSDF * dSDF_dXt;
            calcYawGrad(pos_eva, Xt, yaw, dCost_dSDF, d_yaw);

            // 累积成本
            thread_cost[thread_num] += pena;

            // 计算控制点的梯度
            Eigen::MatrixXd local_gradient = Eigen::MatrixXd::Zero(gradient.rows(), gradient.cols());

            calcQGrad(dCost_dXt, d_yaw, time_star, obj.trajectory_, local_gradient);

            // 累积梯度
            thread_gradients[thread_num] += local_gradient;
        }
    }

    // 汇总所有线程的成本和梯度
    for (int t = 0; t < num_threads; ++t)
    {
        cost += thread_cost[t];
        gradient += thread_gradients[t];
    }
}

double TrajOptimizer::calcCostofSDFValue(double sdf_value, double& dCost_dSDF)
{
    double margin = safe_threshold_ - sdf_value;
    if( margin  <= 0.0 )
    {// 根据一个sdf数值计算cost
        return 0.0;
    }else{
        dCost_dSDF = -2*margin;
        return margin*margin;
    }
}

void TrajOptimizer::calcYawGrad(const Eigen::Vector3d& pos_eva,
                                const Eigen::Vector3d& Xt,
                                const double& yaw,
                                const double dCost_dSDF,
                                double &d_yaw)
{
    double dSDF_dyaw = 0.0;
    Eigen::Matrix3d Rt = Eigen::AngleAxisd(yaw - 0.000001, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double sdf_value1      = sv_manager_ -> shape_ -> getSDF(sv_manager_ ->posEva2Rel(pos_eva, Xt, Rt));
    Rt = Eigen::AngleAxisd(yaw + 0.000001, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    double sdf_value2      = sv_manager_ -> shape_ -> getSDF(sv_manager_ ->posEva2Rel(pos_eva, Xt, Rt));
    dSDF_dyaw = (sdf_value2 - sdf_value1)/0.000002;

    d_yaw = dCost_dSDF * dSDF_dyaw;
}

void TrajOptimizer::calcQGrad(const Eigen::Vector3d dCost_dXt,const double d_yaw, const double time_star,const Bspline::Ptr trajecotry, Eigen::MatrixXd &gradient)
{
    if (trajectory_ == nullptr) {
        std::cerr << "Error: trajectory pointer is null!" << std::endl;
        return;
    }

    //判断应该修改哪些控制点，编号i
    double C_X   = dCost_dXt(0);
    double C_Y   = dCost_dXt(1);
    double C_yaw = d_yaw;

    double s = 0.0;
    int i = -1;
    s = trajectory_->calcU2S(time_star, i);  //一个归一化因子s
    assert(s >= 0); assert(i >= 0);

    assert(i >= 0 && i + 3 < gradient.cols());  // Ensure that i + 3 is within bounds
    assert(gradient.cols() > i + 3);  // Ensure matrix has enough columns

    Eigen::MatrixXd Grad_dCost_dQ = Eigen::MatrixXd::Zero(gradient.rows(), gradient.cols());

    Grad_dCost_dQ(0, i) = C_X * (1 - 3 * s + 3 * s * s - s * s * s);
    Grad_dCost_dQ(1, i) = C_Y * (1 - 3 * s + 3 * s * s - s * s * s);
    Grad_dCost_dQ(2, i) = C_yaw * (1 - 3 * s + 3 * s * s - s * s * s);

    Grad_dCost_dQ(0, i + 1) = C_X * (4 - 6 * s * s + 3 * s * s * s);
    Grad_dCost_dQ(1, i + 1) = C_Y * (4 - 6 * s * s + 3 * s * s * s);
    Grad_dCost_dQ(2, i + 1) = C_yaw * (4 - 6 * s * s + 3 * s * s * s);

    Grad_dCost_dQ(0, i + 2) = C_X * (1 + 3 * s + 3 * s * s - 3 * s * s * s);
    Grad_dCost_dQ(1, i + 2) = C_Y * (1 + 3 * s + 3 * s * s - 3 * s * s * s);
    Grad_dCost_dQ(2, i + 2) = C_yaw * (1 + 3 * s + 3 * s * s - 3 * s * s * s);

    Grad_dCost_dQ(0, i + 3) = C_X * (s * s * s);
    Grad_dCost_dQ(1, i + 3) = C_Y * (s * s * s);
    Grad_dCost_dQ(2, i + 3) = C_yaw * (s * s * s);
    
    gradient -= Grad_dCost_dQ / 6.0;
}
void TrajOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
{
    cost = 0.0;
    // 参考egoplanner，采用弹力带损失的光滑损失函数
    if (falg_use_jerk)
    {
        Eigen::Vector3d jerk, temp_j;

        for (int i = 0; i < q.cols() - 3; i++)
        {
            /* evaluate jerk */
            jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
            cost += jerk.squaredNorm();
            temp_j = 2.0 * jerk;
            /* jerk gradient */
            gradient.col(i + 0) += -temp_j;
            gradient.col(i + 1) += 3.0 * temp_j;
            gradient.col(i + 2) += -3.0 * temp_j;
            gradient.col(i + 3) += temp_j;
        }
    }
    else
    {
        Eigen::Vector3d acc, temp_acc;

        for (int i = 0; i < q.cols() - 2; i++)
        {
            /* evaluate acc */
            acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
            cost += acc.squaredNorm();
            temp_acc = 2.0 * acc;
            /* acc gradient */
            gradient.col(i + 0) += temp_acc;
            gradient.col(i + 1) += -2.0 * temp_acc;
            gradient.col(i + 2) += temp_acc;
        }
    }
}

void TrajOptimizer::calcFeasibilityCost(void *ptr, const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    TrajOptimizer &obj = *(TrajOptimizer *)ptr;
    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = obj.trajectory_->interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
        Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

        for (int j = 0; j < 3; j++)
        {
            double max1 = 999;
            if(j == 2){
                max1 = omg_max_;
            }else{
                max1 = max_vel_;
            }
            if (vi(j) > max1 + demarcation)
            {
            double diff = vi(j) - max1;
            cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

            double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
            gradient(j, i + 0) += -grad;
            gradient(j, i + 1) += grad;
            }
            else if (vi(j) > max1)
            {
            double diff = vi(j) - max1;
            cost += pow(diff, 3) * ts_inv3;
            ;

            double grad = 3 * diff * diff / ts * ts_inv3;
            ;
            gradient(j, i + 0) += -grad;
            gradient(j, i + 1) += grad;
            }
            else if (vi(j) < -(max1 + demarcation))
            {
            double diff = vi(j) + max1;
            cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

            double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
            gradient(j, i + 0) += -grad;
            gradient(j, i + 1) += grad;
            }
            else if (vi(j) < -max1)
            {
            double diff = vi(j) + max1;
            cost += -pow(diff, 3) * ts_inv3;

            double grad = -3 * diff * diff / ts * ts_inv3;
            gradient(j, i + 0) += -grad;
            gradient(j, i + 1) += grad;
            }
            else
            {
            /* nothing happened */
            }
        }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
        Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

        for (int j = 0; j < 3; j++)
        {
            double max2 = 999;
            if(j == 2){
                max2 = alpha_max_;
            }else{
                max2 = max_acc_;
            }
            if (ai(j) > max2 + demarcation)
            {
            double diff = ai(j) - max2;
            cost += ar * diff * diff + br * diff + cr;

            double grad = (2.0 * ar * diff + br) * ts_inv2;
            gradient(j, i + 0) += grad;
            gradient(j, i + 1) += -2 * grad;
            gradient(j, i + 2) += grad;
            }
            else if (ai(j) > max2)
            {
            double diff = ai(j) - max2;
            cost += pow(diff, 3);

            double grad = 3 * diff * diff * ts_inv2;
            gradient(j, i + 0) += grad;
            gradient(j, i + 1) += -2 * grad;
            gradient(j, i + 2) += grad;
            }
            else if (ai(j) < -(max2 + demarcation))
            {
            double diff = ai(j) + max2;
            cost += al * diff * diff + bl * diff + cl;

            double grad = (2.0 * al * diff + bl) * ts_inv2;
            gradient(j, i + 0) += grad;
            gradient(j, i + 1) += -2 * grad;
            gradient(j, i + 2) += grad;
            }
            else if (ai(j) < -max2)
            {
            double diff = ai(j) + max2;
            cost += -pow(diff, 3);

            double grad = -3 * diff * diff * ts_inv2;
            gradient(j, i + 0) += grad;
            gradient(j, i + 1) += -2 * grad;
            gradient(j, i + 2) += grad;
            }
            else
            {
            /* nothing happened */
            }
        }   
    }   
}


void TrajOptimizer::Time_ReAllocate()
{
    return;
}

// 以下内容属于展示
int TrajOptimizer::progress(void *ptr,
                            const Eigen::VectorXd &x,
                            const Eigen::VectorXd &g,
                            const double fx,
                            const double step,
                            const int k,
                            const int ls)
{
    // TrajOptimizer &obj = *(TrajOptimizer *)ptr;
    std::cout << "Iteration " << k << ": cost = " << fx << ", step = " << step << ", line search tries = " << ls << std::endl;
    //展示当前的轨迹
    // obj.visTrajectory();
    return 0;
}


void TrajOptimizer::AsyncSleepMS(const int ms){
    auto start    = std::chrono::steady_clock::now();
    auto duration = std::chrono::milliseconds(ms);
    bool done = false;
    while(!done){
        auto elapsed = std::chrono::steady_clock::now() - start;
        if(elapsed > duration) {
            done = true;
        }else{
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// lmbm优化器部分
int TrajOptimizer::optimize_lmbm()
{
    int num_cols = trajectory_->control_points_.cols();
    int start_col = 3; // 从第4列（索引3）开始
    int end_col = num_cols - 3; // 到倒数第4列（索引 num_cols-3）
    sv_manager_->updateBsplineTraj(trajectory_);
    assert(num_cols >= 6);
    // 这是将要被优化的变量
    int N_x = 3 * (end_col - start_col);
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(trajectory_->control_points_.data() + start_col * 3, N_x);
    // double* x_ptr = x.data();
    x_ptr = (double *)malloc(sizeof(double) * N_x);
    for(int i = 0 ; i< N_x; i ++)
    {
        x_ptr[i] = x(i);
    }
    double fx = 0.0;    //cost数值
    ROS_INFO("\033[0;30;47m   LMBM Optimize start  \033[0m");
    lmbm::lmbm_parameter_t param;
    int result = lmbm::lmbm_optimize(x.size(),
                                     x_ptr,
                                     &fx,
                                     costFunctionLmbmParallel,
                                     this,
                                     earlyExitLMBM,
                                     &param);
    ROS_INFO("\033[1;3;37m result: \033[0m %d ", result);
    if (x_ptr != NULL)
    {
      free(x_ptr);
      x_ptr = NULL;
    }
    //Time_ReAllocate();
    return result;
}

double TrajOptimizer::costFunctionLmbmParallel(void *ptr,
                                               const double *x,
                                               double *g,
                                               const int n)
{
    TrajOptimizer &obj = *(TrajOptimizer*)ptr;

    // 使用 Eigen::Map 将传入的 double* 指针映射为 Eigen::VectorXd
    // Eigen::Map<const Eigen::VectorXd> x_map(x, n);
    // Eigen::Map<Eigen::VectorXd> g_map(g, n);

    Eigen::Map<const Eigen::MatrixXd> Q(x, 3, n / 3);
    Eigen::MatrixXd &control_points = obj.trajectory_->control_points_;
    int total_cols = control_points.cols();
    control_points.block(0, 3, 3, total_cols - 6) = Q;
    int control_points_size = control_points.size();

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness = 0;
    double f_distance = 0;
    double f_feasibility = 0;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, control_points_size);
    Eigen::MatrixXd grad_3D = Eigen::MatrixXd::Zero(3, control_points_size);

    // 更新 Sweep Volume 管理器中的轨迹
    obj.sv_manager_->updateBsplineTraj(obj.trajectory_);

    // 计算各项成本和梯度
    obj.calcSweptVolumeCostParallel(ptr, f_distance, g_distance);
    // obj.calcSmoothnessCost(control_points, f_smoothness, g_smoothness, false);
    // obj.calcFeasibilityCost(ptr, control_points, f_feasibility, g_feasibility);

    // std::cout<<"1 * f_smoothness"<<1 * f_smoothness <<std::endl;
    // std::cout<<"1000*f_distance"<<1000*f_distance <<std::endl;
    // std::cout<<"0.01*f_feasibility"<<0.01*f_feasibility <<std::endl;
    // std::cout<<"1 * f_smoothness"<<1 * f_smoothness <<std::endl;
    // std::cout<<"1000*f_distance"<<1000*f_distance <<std::endl;
    // std::cout<<"0.01*f_feasibility"<<0.01*f_feasibility <<std::endl;

    double f_combine = 1.0 * f_smoothness + 1000.0 * f_distance + 0.01 * f_feasibility;
    grad_3D = 1.0 * g_smoothness + 1000.0 * g_distance + 0.01 * g_feasibility;

    memcpy(g, grad_3D.data() + 9, n * sizeof(double));
    // for(int i=0; i < n;i++)
    // {
    //     g[i] = grad_3D((i+9)%3, (i+9)/3);
    // }
    return f_combine;
}

int TrajOptimizer::earlyExitLMBM(void *instance,
                                 const double *x,
                                 const int k)
{
    TrajOptimizer &obj = *(TrajOptimizer *)instance;
    std::cout << "Iteration " << k << std::endl;
    obj.visTrajectory();
    Eigen::MatrixXd &control_points = obj.trajectory_->control_points_;
    // obj.sv_manager_->updateBsplineTraj(obj.trajectory_);
    // obj.sv_manager_->updateBsplineTraj(obj.trajectory_);
    obj.visualizer_->visControlPoints("control_points", obj.trajectory_->control_points_);
    while (obj.pause_){
        if (obj.next_step_){
            obj.next_step_ = false;
            break;
        }
        obj.AsyncSleepMS(500);
    }
    if (obj.exit_){
        return 1;
    }else{
        return 0;
    }
}


// replan部分

/// @brief  获取给定的重规划轨迹开始的时间的PVA
/// @param predicted_traj_start_time_  给定的重规划轨迹开始的时间
/// @param plan_start_state_XYTheta_P_ 将要被修改的P
/// @param plan_start_state_XYTheta_V_ 将要被修改的V
/// @param plan_start_state_XYTheta_A_ 将要被修改的A
void TrajOptimizer::get_the_predicted_state(double predicted_traj_start_time, 
                                            Eigen::Vector3d &plan_start_state_XYTheta_P,
                                            Eigen::Vector3d &plan_start_state_XYTheta_V,
                                            Eigen::Vector3d &plan_start_state_XYTheta_A)
{
    plan_start_state_XYTheta_P = trajectory_->getPos(predicted_traj_start_time);
    plan_start_state_XYTheta_V = trajectory_->getVel(predicted_traj_start_time);
    plan_start_state_XYTheta_A = trajectory_->getAcc(predicted_traj_start_time);
}