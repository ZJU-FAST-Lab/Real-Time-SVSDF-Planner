#ifndef SWEPT_VOLUME_MANAGER_HPP
#define SWEPT_VOLUME_MANAGER_HPP

#include "plan_env/sdf_map.h"
#include "utils/shape.hpp"
#include "utils/NURBS.hpp"
#include <vector>
#include "Eigen/Dense"

#define useScale false  // 定义是否使用伸缩变换
// valiable 的意思是“合法的”，我的英语不太好随便凑了一个单词，不想修改了

class CircleCoord2D
{
public:
    double r_k; // [0,1]
    double theta;
    CircleCoord2D() {}
    CircleCoord2D(double r_k_value, double theta_value)
    {
        r_k = r_k_value;
        theta = theta_value;
    }
    inline Eigen::Vector2d getPosition(const Eigen::Vector2d &center, const double r) const
    {
        return center + Eigen::Vector2d(r_k * r * cos(theta), r_k * r * sin(theta));
    }
};
class SampleSet2D
{
public:
    int k;
    double theta_res;
    double theta0;
    double rk0;
    double rk_res;
    Eigen::Vector2d center;
    double r;

    pcl::PointCloud<pcl::PointXYZI> history_Yks;

    inline void setCircleCenter(const Eigen::Vector2d &c) { center = c; }
    inline void setCircleRadius(const double radius) { r = radius; }

    inline Eigen::Vector2d getElementPos(const CircleCoord2D &item) { return item.getPosition(center, r); }

    inline std::vector<CircleCoord2D> getElements()
    {
        std::vector<CircleCoord2D> elements;
        for (double rk = rk0; rk > 0; rk -= rk_res)
        {
            for (double theta = theta0; theta < theta0 + 2 * M_PI; theta += theta_res)
            {
                elements.emplace_back(CircleCoord2D{rk, theta});
            }
        }
        return elements;
    }
    inline pcl::PointCloud<pcl::PointXYZI> getHistoryYkCloud() { return history_Yks; }

    inline void initSet(const Eigen::Vector2d &vel, const Eigen::Vector2d &center, const double r_init)
    {
        k = 0;
        history_Yks.points.clear();
        setCircleCenter(center);
        setCircleRadius(r_init);

        theta0 = atan2(vel(0), -vel(1));
        if (theta0 < 0)
        {
            theta0 += 2 * M_PI;
        }

        theta_res = M_PI + 0.1;
        rk_res = 1.5;
        rk0 = 1.0;

        // just for visualize
        std::vector<CircleCoord2D> elements = getElements();
        Eigen::Vector2d pos;
        pcl::PointXYZI point;
        for (auto e : elements)
        {
            pos = e.getPosition(center, r);
            point.x = pos(0);
            point.y = pos(1);
            point.z = 1.0;
            point.intensity = 1;
            history_Yks.points.emplace_back(point);
        }
    }

    inline void expandSet(int theta_dense, double new_theta0)
    {
        k++;
        theta_res /= (theta_dense + 1);
        theta_res = std::max(0.3, theta_res);
        theta0 = new_theta0;
        // just for visualize
        std::vector<CircleCoord2D> elements = getElements();
        for (auto e : elements)
        {
            pcl::PointXYZI point;
            Eigen::Vector2d pos = e.getPosition(center, r);
            point.x = pos(0);
            point.y = pos(1);
            point.z = 1.0;
            point.intensity = k;
            history_Yks.points.push_back(point);
        }
    }
};


class sv_manager
{
public:
    std::shared_ptr<SDFmap> sdfmap_;
    std::shared_ptr<BasicShape> shape_;
    double shape_kernel_resolution_;

    std::shared_ptr<Bspline> BSplineTraj_;
    double traj_duration_;
    double t_min_{0.0};
    double t_max_{1.0};
    double momentum{0.0}; 
public:
    sv_manager(std::shared_ptr<SDFmap> sdfmap, std::shared_ptr<BasicShape> shape):sdfmap_(sdfmap),shape_(shape){
        shape_kernel_resolution_ = 2*M_PI/shape_->degree_divisions_;
    } 
    // 前端
    bool is_shape_kernel_valiable(int x, int y, int shape_kernel_index);
    bool is_shape_kernel_valiable(Eigen::Vector3d pose);
    std::vector<Eigen::Vector2i> get_collision_points(int x, int y, int shape_kernel_index);

    // 后端
    void updateBsplineTraj(const std::shared_ptr<Bspline> new_traj);
    template <bool need_grad_prel = true>
    double getTrueSDFofBsplineSweptVolume(const Eigen::Vector3d &pos_eva, 
                                          double &time_seed_f, 
                                          Eigen::Vector3d &grad_prel, 
                                          bool set_ts, 
                                          Eigen::Vector3d& cor_point_3d, 
                                          bool debug = false);
    template <bool set_ts = false, bool need_grad_prel = true>
    double getSDFofBsplineSweptVolume(const Eigen::Vector3d &pos_eva, 
                                      double &time_seed_f, 
                                      Eigen::Vector3d &grad_prel);
    template <bool withscale = false>
    double choiceBsplineTInit(const Eigen::Vector3d &pos_eva, double dt);
    void gradientDescentBspline(double momentum, double t_min, double t_max,
                            const double x0, double &fx, double &x, const Eigen::Vector3d &pos_eva, bool debug = false);
    template <bool withscale = false>
    Eigen::Vector3d getGradPrelAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp);                            

    // 实用工具
    void getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt);
    void getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Vector3d &vt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &VRt,
                                    Eigen::Matrix3d &St,
                                    Eigen::Matrix3d &dSt);
    void getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt);
    void getStateOnBsplineTrajStamp(const double &time_stamp,
                                    Eigen::Vector3d &xt,
                                    Eigen::Matrix3d &Rt,
                                    Eigen::Matrix3d &St);
    Eigen::Matrix3d getScale(const double t);
    Eigen::Matrix3d getDotScale(const double t);

    Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                               const Eigen::Vector3d &xt,
                               const Eigen::Matrix3d &Rt);
    Eigen::Vector3d posEva2Rel(const Eigen::Vector3d &pos_eva,
                               const Eigen::Vector3d &xt,
                               const Eigen::Matrix3d &Rt,
                               const Eigen::Matrix3d &St);  

    template <bool withscale = false>
    double getSDFAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp); 
    template <bool withscale = false>
    double getSDF_DOTAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp);               
};


/// @brief 一个结合GSIP方法求解的考虑扫略体积内部与外部的SVSDF，也就是真正的SDF
template <bool need_grad_prel>
double sv_manager::getTrueSDFofBsplineSweptVolume(const Eigen::Vector3d &pos_eva, 
                                          double &time_seed_f, 
                                          Eigen::Vector3d &grad_prel, 
                                          bool set_ts, 
                                          Eigen::Vector3d& cor_point_3d, 
                                          bool debug)
{
    double argmin_dis = 0.0;    //设置最短的距离为0.0
    argmin_dis = getSDFofBsplineSweptVolume<false, true>(pos_eva, time_seed_f, grad_prel);//获取一个外部的SDF计算方式
    if (argmin_dis > 0) 
    { // outside case， 如果大于0的话， 这个SDF就是真实的SDF数值
        return argmin_dis;
    }

    // USE GSIP SOLVER TO SOLVE
    double r0 = -argmin_dis;

    Eigen::Vector3d vel = BSplineTraj_->getVel(time_seed_f);
    if (vel.norm() < 0.01)
    {
        if (time_seed_f < 0.1)
        {
            for (double t_scan = time_seed_f; t_scan <= traj_duration_; t_scan += 0.1)
            {
                vel = BSplineTraj_->getVel(t_scan);
                if (vel.norm() >= 0.01)
                {
                    break;
                }
            }
        }
        else if (time_seed_f > traj_duration_ - 0.1)
        {
            for (double t_scan = time_seed_f; t_scan >= 0; t_scan -= 0.1)
            {
                vel = BSplineTraj_->getVel(t_scan);
                if (vel.norm() >= 0.01)
                {
                    break;
                }
            }
        }
    }
    Eigen::Vector2d vel_2d = vel.head(2);
    Eigen::Vector2d obs_2d = pos_eva.head(2);
    SampleSet2D Y_threadsafe;
    Y_threadsafe.initSet(vel_2d, obs_2d, r0);

    int Y_size = 0;
    Eigen::Vector2d yk;
    Eigen::Vector3d yk_3d;
    CircleCoord2D yk_star;
    double r_star;
    double max_g;
    double cur_g;
    double real_t_star;
    int iter = 1;

    while (true)
    {
        // STEP1 calc r* under Yk
        max_g = -100000;
        std::vector<CircleCoord2D> elements = Y_threadsafe.getElements();
        Y_size = elements.size();
        std::vector<Eigen::Vector3d> ball_positions;

        for (int i = 0; i < Y_size; i++)
        {
            yk = Y_threadsafe.getElementPos(elements[i]);
            yk_3d = Eigen::Vector3d(yk[0], yk[1], 0.0);
            cur_g = getSDFofBsplineSweptVolume<false, true>(yk_3d, time_seed_f, grad_prel);
            
            if (cur_g > max_g)
            {
                max_g = cur_g;
                real_t_star = time_seed_f;
                yk_star = elements[i];
            }
        }
        
        r_star = Y_threadsafe.r - max_g;        // 看看问题是否在这里
        Y_threadsafe.setCircleRadius(r_star);

        if (iter > 8)
        {
            break;
        }
        if (max_g > -0.1)
        {
            break;
        }

        // STEP2 calc y* under r*
        Y_threadsafe.expandSet(2, yk_star.theta);
        iter++;
    }

    Eigen::Vector2d cor_point = yk_star.getPosition(Y_threadsafe.center, r_star);
    cor_point_3d = Eigen::Vector3d(cor_point[0], cor_point[1], 0.0);
    grad_prel = (cor_point_3d - pos_eva);
    grad_prel(2) = 0.0;
    grad_prel.normalize();  
    time_seed_f = real_t_star;
    return -r_star;
    // 皈依化梯度
}

/// @brief 假设这个点在外面，获取一个SDF
/// @tparam set_ts 是否给定了t*的初始数值，也就是time seed
/// @tparam need_grad_prel 是否需要给出对relative的那个点的倒数
/// @param pos_eva 要评估的障碍物点
/// @param time_seed_f 给定的t*的初始数值
/// @param grad_prel 对relative的那个点的倒数
/// @return 在t*处的SDF数值
template <bool set_ts, bool need_grad_prel>
inline double sv_manager::getSDFofBsplineSweptVolume(const Eigen::Vector3d &pos_eva, double &time_seed_f, Eigen::Vector3d &grad_prel)
{
    Eigen::Vector3d xt, vt;
    double ts = time_seed_f;
    double t_star;
    double sdf_star;
    double dtime = 0.1;
    if (set_ts == false)
    {
        ts = choiceBsplineTInit<useScale>(pos_eva, dtime);  // check ok
    }
    double tmin_ = std::max(0.0, ts - 3.4);           
    double tmax_ = std::min(ts + 3.4, traj_duration_); 
    gradientDescentBspline(momentum, tmin_, tmax_, ts, sdf_star, t_star, pos_eva);  //改好了
    
    if (need_grad_prel)
    {
        grad_prel = getGradPrelAtTimeStampBspline<useScale>(pos_eva, t_star);
    }
    time_seed_f = t_star;
    return sdf_star;
}

template <bool withscale>
inline double sv_manager::choiceBsplineTInit(const Eigen::Vector3d &pos_eva, double dt)
{

    Eigen::Vector3d xt;
    Eigen::Matrix3d Rt;

    double min_dis = 1e9;
    double dis = 1e9;
    double time_seed = 0.0;

    int pricision_layers = 4;
    int current_layer = 1;

    double loop_terminal = traj_duration_;

    double t = 0.0;
    while (current_layer <= pricision_layers)
    {
        if (current_layer == 1)
        {
            t = 0.0;
        }
        if (current_layer > 1)
        {
            t = std::max(0.0, time_seed - 10 * dt);
            loop_terminal = std::min(traj_duration_, time_seed + 10 * dt);
        }

        for (; t <= loop_terminal; t += dt)
        {
            getStateOnBsplineTrajStamp(t, xt, Rt);  //check ok 无问题
            dis = shape_->getSDF(posEva2Rel(pos_eva, xt, Rt));
            if (dis < min_dis)
            {
                time_seed = t;
                min_dis = dis;
            }
        }
        dt *= 0.1;
        current_layer += 1;
    }
    return time_seed;
}



template <bool withscale>
Eigen::Vector3d sv_manager::getGradPrelAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp)
{
    Eigen::Vector3d xt;
    Eigen::Matrix3d Rt;
    if (!withscale)
    {
        getStateOnBsplineTrajStamp(time_stamp, xt, Rt);
        return shape_->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt));
    }
    else
    {
        Eigen::Matrix3d St;
        getStateOnBsplineTrajStamp(time_stamp, xt, Rt, St);
        return shape_->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt, St));
    }
}


template <bool withscale>
double sv_manager::getSDFAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp)
{
    Eigen::Vector3d xt;
    Eigen::Matrix3d Rt;
    if (!withscale)
    {
        getStateOnBsplineTrajStamp(time_stamp, xt, Rt);
        return shape_->getSDF(posEva2Rel(pos_eva, xt, Rt));
    }
    else
    {
        Eigen::Matrix3d St;
        getStateOnBsplineTrajStamp(time_stamp, xt, Rt, St);
        return shape_->getSDF(posEva2Rel(pos_eva, xt, Rt, St));
    }
}

template <bool withscale>
double sv_manager::getSDF_DOTAtTimeStampBspline(const Eigen::Vector3d &pos_eva, const double &time_stamp)
{

    double t1 = std::max(0.0, time_stamp - 0.000001);
    double t2 = std::min(traj_duration_, time_stamp + 0.000001);
    double sdf1 = getSDFAtTimeStampBspline<withscale>(pos_eva, t1);
    double sdf2 = getSDFAtTimeStampBspline<withscale>(pos_eva, t2);
    return (sdf2 - sdf1) * 500000;

    Eigen::Vector3d sdf_grad1;
    Eigen::Vector3d point_velocity;
    Eigen::Vector3d xt, vt;
    Eigen::Matrix3d Rt, VRt, Rt_trans;

    if (!withscale)
    {
        getStateOnBsplineTrajStamp(time_stamp, xt, vt, Rt, VRt);
        Rt_trans = Rt.transpose();
        sdf_grad1 = shape_->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt));
        point_velocity = -(Rt_trans * vt + VRt.transpose() * (pos_eva - xt));
    }
    else
    {
        Eigen::Matrix3d St, dSt;
        getStateOnBsplineTrajStamp(time_stamp, xt, vt, Rt, VRt, St, dSt);
        Rt_trans = Rt.transpose();
        sdf_grad1 = shape_->getonlyGrad1(posEva2Rel(pos_eva, xt, Rt, St)); 
        point_velocity = -(Rt_trans * St.inverse() * vt + VRt.transpose() * St.inverse() * (pos_eva - xt) + Rt_trans * St.inverse() * dSt * St.inverse() * (pos_eva - xt));
    } 
    return sdf_grad1.dot(point_velocity);
}

#endif