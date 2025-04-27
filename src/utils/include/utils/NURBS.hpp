#ifndef NURBS_HPP
#define NURBS_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <memory>
#include "robotstatemsgs/robotTrajectory.h"

//Bspline的轨迹类，包含了一条Bspline的定义和对其详细特征的访问方法
class Bspline{
public:
    //用于优化的信息
    int p_, n_, m_;// n+1 control points控制点,  m+1 knots节点x向量, m+1 = n+1 + p+1, p是次数，阶数 = 次>数 + 1
    Eigen::VectorXd u_;
    Eigen::MatrixXd control_points_;
    double interval_;
    Eigen::Vector3d P_0, P_T, V_0, V_T, A_0, A_T;//起始点与终点约束
    std::vector<Eigen::Vector3d> path_;  //   

private: 
    Eigen::Matrix4d matrix_pos;
    Eigen::Matrix<double, 3, 4> matrix_vel;
    Eigen::Matrix<double, 2, 4> matrix_acc;

public:
    typedef std::shared_ptr<Bspline> Ptr;

    Bspline(){
        matrix_pos <<  1,  4,  1, 0,
                      -3,  0,  3, 0,
                       3, -6,  3, 0,
                      -1,  3, -3, 1;
                       
        matrix_vel << -3,  0,  3, 0,
                       6,-12,  6, 0,
                      -3,  9, -9, 3;
         
        matrix_acc <<  6,-12,  6, 0,
                      -6, 18,-18, 6;
    }
    ~Bspline(){}

    // 服务于msg之间的转换
    Bspline(const robotstatemsgs::robotTrajectory::ConstPtr& msg)
    {
        // 填充控制点
        size_t num_control_points = msg->control_points.size();
        if (num_control_points == 0) {
            throw std::runtime_error("BSplineTrajectory message contains no control points.");
        }
        control_points_.resize(3, num_control_points);
        for (size_t i = 0; i < num_control_points; ++i) {
            control_points_(0, i) = msg->control_points[i].x;
            control_points_(1, i) = msg->control_points[i].y;
            control_points_(2, i) = msg->control_points[i].z;
        }

        // 设置B-spline的阶数（次数）
        p_ = 3; // 设为三阶B-spline

        // 设置n_和m_
        n_ = num_control_points - 1;
        m_ = n_ + p_ + 1;

        // 填充时间点
        size_t expected_u = m_ + 1;
        if (msg->u.size() != expected_u) {
            throw std::runtime_error("u size does not match m_ + 1.");
        }
        u_ = Eigen::VectorXd::Zero(expected_u);
        for (int i = 0; i < expected_u; ++i) {
            u_(i) = msg->u[i];
        }

        matrix_pos <<  1,  4,  1, 0,
                      -3,  0,  3, 0,
                       3, -6,  3, 0,
                      -1,  3, -3, 1;
                       
        matrix_vel << -3,  0,  3, 0,
                       6,-12,  6, 0,
                      -3,  9, -9, 3;
         
        matrix_acc <<  6,-12,  6, 0,
                      -6, 18,-18, 6;
    }

    robotstatemsgs::robotTrajectory getRobotTrajectoryMsg(const ros::Time& traj_start_time) const {
        robotstatemsgs::robotTrajectory msg;
        
        // 填充header和traj_start_time
        msg.Header.stamp = ros::Time::now();
        msg.Header.frame_id = "world";
        msg.traj_start_time = traj_start_time;
        
        // 填充control_points
        for (int i = 0; i < control_points_.cols(); ++i) {
            geometry_msgs::Vector3 point;
            point.x = control_points_(0, i);
            point.y = control_points_(1, i);
            point.z = control_points_(2, i);
            msg.control_points.push_back(point);
        }
        
        // 填充u
        for (int i = 0; i < u_.size(); ++i) {
            msg.u.push_back(u_(i));
        }
        return msg;
    }

    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval){
        control_points_ = points;
        p_ = order;
        interval_ = interval;

        n_ = points.cols() - 1;
        m_ = n_ + p_ + 1;

        u_ = Eigen::VectorXd::Zero(m_ + 1);
        for (int i = 0; i <= m_; ++i){
            if (i <= p_){
                u_(i) = double(-p_ + i) * interval_;
            }else if (i > p_ && i <= m_ - p_){
                u_(i) = u_(i - 1) + interval_;
            }else if (i > m_ - p_){
                u_(i) = u_(i - 1) + interval_;
            }
        }
    }

    void setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

    Eigen::VectorXd evaluateDeBoor(const double &u) const{
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true){
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        /* deBoor's alg */
        std::vector<Eigen::VectorXd> d;
        for (int i = 0; i <= p_; ++i){
            d.push_back(control_points_.col(k - p_ + i));
            // cout << d[i].transpose() << endl;
        }

        for (int r = 1; r <= p_; ++r){
            for (int i = p_; i >= r; --i){
                double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
                // cout << "alpha: " << alpha << endl;
                d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p_];
    }

    /// @brief 对轨迹采样用于作图
    /// @return 轨迹点
    std::vector<Eigen::Vector3d> getRoute(){
        std::vector<Eigen::Vector3d> route;
        double tm, tmp;
        getTimeSpan(tm, tmp);
        for (double u = tm; u <= tmp; u = u + 0.01 ){
            route.push_back(evaluateDeBoor(u));
        }
        return route;
    }

    //服务于重规划
    int getSegNum()
    {
        std::cout<< "m_ - 2*p_ - 1:"<<m_ - 2*p_ - 1<<std::endl;
        std::cout<< "m:"<<m_<<std::endl;
        std::cout<<"p:"<<p_<<std::endl;
        std::cout<<"u.size():"<<u_.size()<<std::endl;
        std::cout<<"control_points_.cols()"<<control_points_.cols()<<std::endl;
        return  m_ - 2*p_ - 1;
    }

    double getMaxVel(int i)
    {
        double velTemp = 0.0;
        double MaxVel = 0.0;
        Eigen::VectorXd Vel;
        Bspline derivative = getDerivative();
        for (double u = u_(p_+ i); u < u_(p_+ i + 1)-0.01; u = u + 0.01 )
        {
            Vel = derivative.evaluateDeBoor(u);
            velTemp = std::sqrt(Vel[0] * Vel[0] + Vel[1] * Vel[1]);
            if(velTemp>MaxVel)
            {
                MaxVel = velTemp;
            }
        }
        // std::cout<<"i"<<i<<std::endl;
        // std::cout<<"MaxVel"<<MaxVel<<std::endl;
        return MaxVel;
    }

    double getMaxAcc(int i)
    {
        double AccTemp = 0.0;
        double MaxAcc = 0.0;
        Eigen::VectorXd Acc;
        Bspline derivative = getDerivative();
        Bspline derivative2 = derivative.getDerivative();
        for (double u = u_(p_+ i); u < u_(p_+ i + 1)-0.01; u = u + 0.01 )
        {
            Acc = derivative2.evaluateDeBoor(u);
            AccTemp = std::sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]);
            if(AccTemp>MaxAcc)
            {
                MaxAcc = AccTemp;
            }
        }
        // std::cout<<"i"<<i<<std::endl;
        // std::cout<<"MaxAcc"<<MaxAcc<<std::endl;
        return MaxAcc;
    }

    double getMaxOmg(int i)
    {
        double OmgTemp = 0.0;
        double MaxOmg = 0.0;
        Eigen::VectorXd Omg;
        Bspline derivative = getDerivative();
        for (double u = u_(p_+ i); u < u_(p_+ i + 1)-0.01; u = u + 0.01 )
        {
            Omg = derivative.evaluateDeBoor(u);
            OmgTemp = std::sqrt(Omg[2] * Omg[2]);
            if(OmgTemp>MaxOmg)
            {
                MaxOmg = OmgTemp;
            }
        }
        // std::cout<<"i"<<i<<std::endl;
        // std::cout<<"MaxOmg"<<MaxOmg<<std::endl;
        return MaxOmg;
    }

    double getMaxAlpha(int i)
    {
        double AlphaTemp = 0.0;
        double MaxAlpha = 0.0;
        Eigen::VectorXd Alpha;
        Bspline derivative = getDerivative();
        Bspline derivative2 = derivative.getDerivative();
        for (double u = u_(p_+ i); u < u_(p_+ i + 1)-0.01; u = u + 0.01 )
        {
            Alpha = derivative2.evaluateDeBoor(u);
            AlphaTemp = std::sqrt(Alpha[2] * Alpha[2]);
            if(AlphaTemp>MaxAlpha)
            {
                MaxAlpha = AlphaTemp;
            }
        }
        // std::cout<<"i"<<i<<std::endl;
        // std::cout<<"MaxAlpha"<<MaxAlpha<<std::endl;
        return MaxAlpha;
    }

    void ReAllocateTime(double rate, int i)
    {
        double current_duration = u_[p_ + i + 1] - u_[p_ + i ];
        // std::cout<<"current_duration"<<current_duration<<std::endl;
        double new_duration = rate * current_duration;
        // std::cout<<"new_duration"<<new_duration<<std::endl;
        // std::cout<<"u_[p_ + i ]"<<u_[p_ + i ]<<std::endl;
        // std::cout<<"u_[p_ + i + 1]"<<u_[p_ + i + 1]<<std::endl;
        
        // 更新u_(p_ + i + 1)的位置，保持新的时间跨度
        double new_u_next = u_[p_ + i ] + new_duration;
        double diff = new_u_next - u_[p_ + i  + 1]; // 计算需要顺延的量
        
        // std::cout<<"new_u_next"<<new_u_next<<std::endl;
        // std::cout<<"diff"<<diff<<std::endl;
        // 更新当前段的时间
        u_[p_ + i  + 1] = new_u_next;
        // std::cout<<"u_[p_ + i + 1]"<<u_[p_ + i + 1]<<std::endl;
        // 向后顺延更新后续的u值
        for (int j = p_ + i  + 2; j < u_.size(); j++) 
        {
            if (j >= u_.size()) 
            {
                // std::cerr << "Index out of bounds when updating u[" << j << "]" << std::endl;
                break;  // 防止超出边界
            }
            u_[j] += diff;  // 顺延后续的所有节点
        }
        //std::cout<<"u"<<u_<<std::endl;
    }

    //服务于SweptVolume
    double getTotalDuration()
    {
        double tm, tmp;
        if (getTimeSpan(tm, tmp))
        return tmp - tm;
        else
        return -1.0;
    }

    bool getTimeSpan(double &um, double &um_p)
    {
        if (p_ > u_.rows() || m_ - p_ > u_.rows())
        return false;

        um = u_(p_);
        um_p = u_(m_ - p_);

        return true;
    }

    inline Eigen::Vector3d getPos(double u) const
    {
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true){
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        double s = (ub - u_(k))/(u_(k+1)-u_(k));
        Eigen::RowVector4d S;
        S<< 1, s,s*s, s*s*s;

        Eigen::Matrix<double, 4, 3> CP = control_points_.block(0, k-3, 3, 4).transpose();
        Eigen::RowVector3d result = S * matrix_pos * CP / 6.0;
        return result.transpose();
    }

    inline Eigen::Vector3d getVel(double u) {
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true){
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        double s = (ub - u_(k))/(u_(k+1)-u_(k));
        Eigen::RowVector3d S;
        S<< 1, s, s*s;
                         
        Eigen::Matrix<double, 4, 3> CP = control_points_.block(0, k-3, 3, 4).transpose();
        Eigen::RowVector3d result = S * matrix_vel * CP / 6.0 / (u_(k+1)-u_(k));
        return result.transpose();
    }

    inline void getPosAndVel(double u, Eigen::Vector3d &pos, Eigen::Vector3d &vel) const
    {
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true){
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        double s = (ub - u_(k))/(u_(k+1)-u_(k));
        Eigen::RowVector4d S;
        S<< 1, s,s*s, s*s*s;

        Eigen::Matrix<double, 4, 3> CP = control_points_.block(0, k-3, 3, 4).transpose();
        Eigen::RowVector3d POS = S * matrix_pos * CP / 6.0;
        Eigen::RowVector3d VEL = S.head<3>() * matrix_vel * CP / 6.0 / (u_(k+1)-u_(k));
        pos = POS.transpose();
        vel = VEL.transpose();
    }

    inline Eigen::Vector3d getAcc(double u) {
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true){
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        double s = (ub - u_(k))/(u_(k+1)-u_(k));
        Eigen::RowVector2d S;
        S<< 1, s;
                         
        Eigen::Matrix<double, 4, 3> CP = control_points_.block(0, k-3, 3, 4).transpose();
        Eigen::RowVector3d result = S * matrix_acc * CP / 6.0 / (u_(k+1)-u_(k))/ (u_(k+1)-u_(k));
        return result.transpose();
    }

    inline Bspline getDerivative()
    {
        Eigen::MatrixXd ctp = getDerivativeControlPoints();
        Bspline derivative;
        derivative.setUniformBspline(ctp, p_ - 1, interval_);

        /* cut the first and last knot */
        Eigen::VectorXd knot(u_.rows() - 2);
        knot = u_.segment(1, u_.rows() - 2);
        derivative.setKnot(knot);

        return derivative;
    }

    inline Eigen::MatrixXd getDerivativeControlPoints()
    {
        // The derivative of a b-spline is also a b-spline, its order become p_-1
        // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
        Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
        for (int i = 0; i < ctp.cols(); ++i)
        {
        ctp.col(i) =
            p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
        }
        return ctp;
    }

    inline double calcU2S(double u, int& i)  //可能有问题，需要深思 day1203
    {
        i = 0;
        for (int j = p_; j <= m_ - p_ - 1; j++){
            if(u >= u_(j) && u <= u_(j + 1))
            {
                i = j - p_;
                return (u - u_(j))/(u_(j + 1) - u_(j));
            }
        }
        return -1;
    }
};


#endif