
#ifndef SHAPE_HPP
#define SHAPE_HPP

#include "Eigen/Dense"
#include <stdio.h>
#include <vector>
#include <iostream>
#include <bits/algorithmfwd.h>



class BasicShape
{
public:
    double threshold = 0.08;
    double threshold_inflate = 0.7;
    // for shape_kernel
    double gridmap_interval_ = 0.1;   //kernel的分辨率
    int degree_divisions_ = 36;
    std::array<uint32_t, 32> mask_array_ = {
        0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80,
        0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000,
        0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000,
        0x1000000, 0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000
    };
    ros::NodeHandle nh_;
    Eigen::Vector2d offset_;

public:
    // for SDF calculate
    virtual double getSDF(const Eigen::Vector3d& point_rel) = 0;
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) = 0;
    //
    // for shape_kernel

    std::vector<std::array<uint32_t, 32>> shape_kernel_list_; // 动态管理第一个维度
    std::vector<std::array<uint32_t, 32>> shape_kernel_inflate_list_; // 动态管理第一个维度


    //构造函数
    BasicShape(ros::NodeHandle &nh) {
        // std::cout << "beigin"<< std::endl;
        // std::cout << "end"<< std::endl;
        nh_ = nh;
    }
    virtual ~BasicShape() = default;
};


class ArcShape : public BasicShape
{
private:
    const Eigen::Vector2d sc{Eigen::Vector2d(std::sin(20), std::cos(20))};
    const double ra{1.16665*0.8};
    const double rb{0.25*0.8};

public:
    double getSDF(const Eigen::Vector3d& point_rel) override
    {
        double x = std::abs(point_rel(0));
        double y = point_rel(1) + ra;
        Eigen::RowVector2d Pos_rel ;
        Pos_rel << x, y;
        bool condition = sc.y() * Pos_rel.x() > sc.x() * Pos_rel.y();
        double dist1 = (Pos_rel - sc.transpose() * ra).norm();
        double dist2 = std::abs(Pos_rel.norm() - ra);
        double result = (condition ? dist1 : dist2) - rb;
        return result;
    }
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) override
    {                                                                      
        double dx = 0.000001;                                              
        Eigen::RowVector3d temp = pos_rel;                                 
        temp(0) -= dx;                                                     
        double sdfold = getSDF(temp);                                      
        temp(0) += 2 * dx;                                                 
        double gradx = getSDF(temp) - sdfold;                              
                                                                        
        temp = pos_rel;                                                    
                                                                        
        temp(1) -= dx;                                                     
        sdfold = getSDF(temp);                                             
        temp(1) += 2 * dx;                                                 
        double grady = getSDF(temp) - sdfold;                              
                                                                        
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);
        return grad;                                                       
    }
    inline void generate_shape_kernel()                                                                                       
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_list_.resize(degree_divisions_);                                                                         
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_list_[i].fill(0);                                                                                    
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold)                                                                                       
                    {                                                                                                         
                        shape_kernel_list_[i][y+16] |= mask_array_[x+16];                                                     
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
    }                                                                                                                         
    inline void generate_shape_kernel_inflate()                                                                               
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_inflate_list_.resize(degree_divisions_);                                                                 
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_inflate_list_[i].fill(0);                                                                            
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold_inflate)                                                                               
                    {                                                                                                         
                        shape_kernel_inflate_list_[i][y+16] |= mask_array_[x+16];                                             
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
        std::cout<<"SHape_Kernel_inflate_initialized!"<<std::endl;                                                            
    }                                                                                                                         
    ArcShape(ros::NodeHandle &nh):BasicShape(nh)
    {
        std::cout << "ArcShape begin" << std::endl;
        generate_shape_kernel();  // 在这里调用
        generate_shape_kernel_inflate();
        std::cout << "ArcShape end" << std::endl;
    }
    ~ArcShape(){}
};



class TShape : public BasicShape
{
public:
    std::vector<Eigen::Vector2d> v = {
        Eigen::Vector2d(-0.6, 0.0),
        Eigen::Vector2d(-0.6, 1.0),
        Eigen::Vector2d(-0.1, 1.0),
        Eigen::Vector2d(-0.1, 0.255),
        Eigen::Vector2d( 1.5, 0.255),
        Eigen::Vector2d( 1.5, 0.0),
    };
    virtual double getSDF(const Eigen::Vector3d& point_rel) override
    {
        Eigen::Vector2d p(point_rel(0), std::abs(point_rel(1)));
        double d = (p-v[0]).dot((p-v[0]));
        double s = 1.0;
        for(int i=0, j=5; i<6; j=i, i++)
        {
            Eigen::Vector2d e = v[j] - v[i];
            Eigen::Vector2d w = p    - v[i];
            double value1 = w.dot(e)/e.dot(e);
            Eigen::Vector2d b = w - e*(  value1<0.0 ? 0.0: (value1 >1.0 ?1.0: value1 ) );
            d = std::min(d, b.dot(b));
            // 计算 winding number，根据几何算法判断点是否在多边形内
            bool cond1 = p.y() >= v[i].y();   // p 的 y 分量大于等于当前顶点的 y 分量
            bool cond2 = p.y() < v[j].y();    // p 的 y 分量小于下一个顶点的 y 分量
            bool cond3 = e.x() * w.y() > e.y() * w.x();  // 边的方向与点的相对方向

            if ((cond1 && cond2 && cond3) || (!(cond1 || cond2) && !(cond3))) {
                s = -s;  // 更新 winding number
            }
        }
        return s * std::sqrt(d); 
    }
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) override
    {                                                                      
        double dx = 0.000001;                                              
        Eigen::RowVector3d temp = pos_rel;                                 
        temp(0) -= dx;                                                     
        double sdfold = getSDF(temp);                                      
        temp(0) += 2 * dx;                                                 
        double gradx = getSDF(temp) - sdfold;                              
                                                                        
        temp = pos_rel;                                                    
                                                                        
        temp(1) -= dx;                                                     
        sdfold = getSDF(temp);                                             
        temp(1) += 2 * dx;                                                 
        double grady = getSDF(temp) - sdfold;                              
                                                                        
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);
        return grad;                                                       
    }
    inline void generate_shape_kernel()                                                                                       
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_list_.resize(degree_divisions_);                                                                         
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_list_[i].fill(0);                                                                                    
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold)                                                                                       
                    {                                                                                                         
                        shape_kernel_list_[i][y+16] |= mask_array_[x+16];                                                     
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
    }                                                                                                                         
    inline void generate_shape_kernel_inflate()                                                                               
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_inflate_list_.resize(degree_divisions_);                                                                 
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_inflate_list_[i].fill(0);                                                                            
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold_inflate)                                                                               
                    {                                                                                                         
                        shape_kernel_inflate_list_[i][y+16] |= mask_array_[x+16];                                             
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
        std::cout<<"SHape_Kernel_inflate_initialized!"<<std::endl;                                                            
    }
    TShape(ros::NodeHandle &nh):BasicShape(nh)
    {
        std::cout << "ArcShape begin" << std::endl;
        generate_shape_kernel();  // 在这里调用
        generate_shape_kernel_inflate();
        std::cout << "ArcShape end" << std::endl;
    }
    ~TShape(){}
};

class AShape : public BasicShape
{
public:
    std::vector<Eigen::Vector2d> v = {
        Eigen::Vector2d(0.0, -0.255),
        Eigen::Vector2d(0.35, -0.255),
        Eigen::Vector2d(0.35, -1.115),
        Eigen::Vector2d(0.8, -1.115),
        Eigen::Vector2d(0.8, 1.035),
        Eigen::Vector2d(0.35, 1.035),
        Eigen::Vector2d(0.0, 1.035)
    };
    virtual double getSDF(const Eigen::Vector3d& point_rel) override
    {
        Eigen::Vector2d p(std::abs(point_rel(0)), point_rel(1));
        // 首先判断是否在圆环区域内
        if(p(0) >= 0.35 && p(1) >= 0.585)
        {
            return std::sqrt((p(0)-0.35)*(p(0)-0.35) + (p(1)-0.585)*(p(1)-0.585)) - 0.45;
        }
        double d = (p-v[0]).dot((p-v[0]));
        double s = 1.0;
        for(int i=0, j=6; i<7; j=i, i++)
        {
            Eigen::Vector2d e = v[j] - v[i];
            Eigen::Vector2d w = p    - v[i];
            double value1 = w.dot(e)/e.dot(e);
            Eigen::Vector2d b = w - e*(  value1<0.0 ? 0.0: (value1 >1.0 ?1.0: value1 ) );
            d = std::min(d, b.dot(b));
            // 计算 winding number，根据几何算法判断点是否在多边形内
            bool cond1 = p.y() >= v[i].y();   // p 的 y 分量大于等于当前顶点的 y 分量
            bool cond2 = p.y() < v[j].y();    // p 的 y 分量小于下一个顶点的 y 分量
            bool cond3 = e.x() * w.y() > e.y() * w.x();  // 边的方向与点的相对方向

            if ((cond1 && cond2 && cond3) || (!(cond1 || cond2) && !(cond3))) {
                s = -s;  // 更新 winding number
            }
        }
        return s * std::sqrt(d); 
    }
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) override
    {                                                                      
        double dx = 0.000001;                                              
        Eigen::RowVector3d temp = pos_rel;                                 
        temp(0) -= dx;                                                     
        double sdfold = getSDF(temp);                                      
        temp(0) += 2 * dx;                                                 
        double gradx = getSDF(temp) - sdfold;                              
                                                                        
        temp = pos_rel;                                                    
                                                                        
        temp(1) -= dx;                                                     
        sdfold = getSDF(temp);                                             
        temp(1) += 2 * dx;                                                 
        double grady = getSDF(temp) - sdfold;                              
                                                                        
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);
        return grad;                                                       
    }
    inline void generate_shape_kernel()                                                                                       
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_list_.resize(degree_divisions_);                                                                         
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_list_[i].fill(0);                                                                                    
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold)                                                                                       
                    {                                                                                                         
                        shape_kernel_list_[i][y+16] |= mask_array_[x+16];                                                     
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
    }                                                                                                                         
    inline void generate_shape_kernel_inflate()                                                                               
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_inflate_list_.resize(degree_divisions_);                                                                 
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_inflate_list_[i].fill(0);                                                                            
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold_inflate)                                                                               
                    {                                                                                                         
                        shape_kernel_inflate_list_[i][y+16] |= mask_array_[x+16];                                             
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
        std::cout<<"SHape_Kernel_inflate_initialized!"<<std::endl;                                                            
    }
    AShape(ros::NodeHandle &nh):BasicShape(nh)
    {
        std::cout << "ArcShape begin" << std::endl;
        generate_shape_kernel();  // 在这里调用
        generate_shape_kernel_inflate();
        std::cout << "ArcShape end" << std::endl;
    }
    ~AShape(){}
};



class FShape : public BasicShape
{
public:
    std::vector<Eigen::Vector2d> v = {
        Eigen::Vector2d(0.93963, -0.255),
        Eigen::Vector2d(0.93963, 0.255),
        Eigen::Vector2d(-0.42037, 0.255),
        Eigen::Vector2d(-0.42037, 0.725),    //here
        Eigen::Vector2d(0.93963, 0.725),
        Eigen::Vector2d(0.93963,1.245),
        Eigen::Vector2d(-0.42037, 1.245),
        Eigen::Vector2d(-0.94037, 0.725),
        Eigen::Vector2d(-0.94037,-1.255),
        Eigen::Vector2d(-0.42037, -1.255),
        Eigen::Vector2d(-0.42037,-0.255)
    };
    virtual double getSDF(const Eigen::Vector3d& point_rel) override
    {
        Eigen::Vector2d p(point_rel(0), point_rel(1));
        // 首先判断是否在圆环区域内
        if(p(0) <= -0.42037 && p(1) >= 0.725)
        {
            double sdf =  std::sqrt((p(0)+0.42037)*(p(0)+0.42037) + (p(1)-0.725)*(p(1)-0.725)) - 0.52;
            if(sdf < 0)
                sdf = std::max(sdf, -sdf-0.52);
            return sdf;
        }
        double d = (p-v[0]).dot((p-v[0]));
        double s = 1.0;
        for(int i=0, j=10; i<11; j=i, i++)
        {
            Eigen::Vector2d e = v[j] - v[i];
            Eigen::Vector2d w = p    - v[i];
            double value1 = w.dot(e)/e.dot(e);
            Eigen::Vector2d b = w - e*(  value1<0.0 ? 0.0: (value1 >1.0 ?1.0: value1 ) );
            if(i != 7)
            {
                d = std::min(d, b.dot(b));
            }
            // 计算 winding number，根据几何算法判断点是否在多边形内
            bool cond1 = p.y() >= v[i].y();   // p 的 y 分量大于等于当前顶点的 y 分量
            bool cond2 = p.y() < v[j].y();    // p 的 y 分量小于下一个顶点的 y 分量
            bool cond3 = e.x() * w.y() > e.y() * w.x();  // 边的方向与点的相对方向

            if ((cond1 && cond2 && cond3) || (!(cond1 || cond2) && !(cond3))) {
                s = -s;  // 更新 winding number
            }
        }
        return s * std::sqrt(d); 
    }
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) override
    {                                                                      
        double dx = 0.000001;                                              
        Eigen::RowVector3d temp = pos_rel;                                 
        temp(0) -= dx;                                                     
        double sdfold = getSDF(temp);                                      
        temp(0) += 2 * dx;                                                 
        double gradx = getSDF(temp) - sdfold;                              
                                                                        
        temp = pos_rel;                                                    
                                                                        
        temp(1) -= dx;                                                     
        sdfold = getSDF(temp);                                             
        temp(1) += 2 * dx;                                                 
        double grady = getSDF(temp) - sdfold;                              
                                                                        
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);
        return grad;                                                       
    }
    inline void generate_shape_kernel()                                                                                       
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_list_.resize(degree_divisions_);                                                                         
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_list_[i].fill(0);                                                                                    
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold)                                                                                       
                    {                                                                                                         
                        shape_kernel_list_[i][y+16] |= mask_array_[x+16];                                                     
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
    }                                                                                                                         
    inline void generate_shape_kernel_inflate()                                                                               
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_inflate_list_.resize(degree_divisions_);                                                                 
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_inflate_list_[i].fill(0);                                                                            
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold_inflate)                                                                               
                    {                                                                                                         
                        shape_kernel_inflate_list_[i][y+16] |= mask_array_[x+16];                                             
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
        std::cout<<"SHape_Kernel_inflate_initialized!"<<std::endl;                                                            
    }
    FShape(ros::NodeHandle &nh):BasicShape(nh)
    {
        std::cout << "ArcShape begin" << std::endl;
        generate_shape_kernel();  // 在这里调用
        generate_shape_kernel_inflate();
        std::cout << "ArcShape end" << std::endl;
    }
    ~FShape(){}
};

class SShape : public BasicShape
{
public:
    const Eigen::Vector2d circle1{Eigen::Vector2d(-0.3, 0.495)};
    const Eigen::Vector2d circle2{Eigen::Vector2d(0.3, -0.495)};
    const Eigen::Vector2d point1{Eigen::Vector2d(-0.3, 0)};
    const Eigen::Vector2d point2{Eigen::Vector2d(0.3, 0)};
    const Eigen::Vector2d point3{Eigen::Vector2d(-0.3, 0.99)};
    const Eigen::Vector2d point4{Eigen::Vector2d(0.3, -0.99)};
    const double r = 0.255;

    virtual double getSDF(const Eigen::Vector3d& point_rel) override
    {
        Eigen::Vector2d p(point_rel(0), point_rel(1));
        //计算左边的圆弧
        double d1 = 0;
        if(p(0) < - 0.3)
        {
            d1 = std::abs((p-circle1).norm() - 0.495);
        }else
        {
            d1 = std::min((p-point3).norm(), (p-point1).norm());
        }
        
        //计算中间的线段
        Eigen::Vector2d e = point1 - point2;
        Eigen::Vector2d w = p      - point2;
        double value1 = w.dot(e)/e.dot(e);
        Eigen::Vector2d b = w - e*(  value1<0.0 ? 0.0: (value1 >1.0 ?1.0: value1 ) );
        double d2 = b.norm();

        //计算右边的圆弧
        double d3 = 0;
        if(p(0) > 0.3)
        {
            d3 = std::abs((p-circle2).norm() - 0.495);
        }else
        {
            d3 = std::min((p-point2).norm(), (p-point4).norm()) ;
        }

        return std::min(d1, std::min(d2,d3)) - r ;
    }
    virtual Eigen::Vector3d getonlyGrad1(const Eigen::RowVector3d &pos_rel) override
    {                                                                      
        double dx = 0.000001;                                              
        Eigen::RowVector3d temp = pos_rel;                                 
        temp(0) -= dx;                                                     
        double sdfold = getSDF(temp);                                      
        temp(0) += 2 * dx;                                                 
        double gradx = getSDF(temp) - sdfold;                              
                                                                        
        temp = pos_rel;                                                    
                                                                        
        temp(1) -= dx;                                                     
        sdfold = getSDF(temp);                                             
        temp(1) += 2 * dx;                                                 
        double grady = getSDF(temp) - sdfold;                              
                                                                        
        Eigen::Vector3d grad = Eigen::Vector3d(gradx, grady, 0) / (2 * dx);
        return grad;                                                       
    }
    inline void generate_shape_kernel()                                                                                       
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_list_.resize(degree_divisions_);                                                                         
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_list_[i].fill(0);                                                                                    
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold)                                                                                       
                    {                                                                                                         
                        shape_kernel_list_[i][y+16] |= mask_array_[x+16];                                                     
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
    }                                                                                                                         
    inline void generate_shape_kernel_inflate()                                                                               
    {                                                                                                                         
        double resolution = 2*M_PI/degree_divisions_;                                                                         
        double degree = -M_PI;                                                                                                
        shape_kernel_inflate_list_.resize(degree_divisions_);                                                                 
        for (int i = 0; i < degree_divisions_; i++) {                                                                         
            shape_kernel_inflate_list_[i].fill(0);                                                                            
        }                                                                                                                     
        for(int i = 0; i < degree_divisions_; i++)                                                                            
        {                                                                                                                     
            std::cout << "i"<< i<<std::endl;                                                                                  
            std::cout << "theta"<<degree<<std::endl;                                                                          
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(degree, Eigen::Vector3d::UnitZ()).toRotationMatrix();                      
            for(int x = -16; x < 16; x++)                                                                                     
            {                                                                                                                 
                for(int y = -16;y < 16; y++)                                                                                  
                {                                                                                                             
                    Eigen::Vector3d point_evaluate, point_relative;                                                           
                    point_evaluate << x*gridmap_interval_ + gridmap_interval_/2, y*gridmap_interval_ + gridmap_interval_/2, 0;
                    point_relative = (Rt.transpose())*point_evaluate;                                                         
                    double sdf = getSDF(point_relative);                                                                      
                    if(sdf < threshold_inflate)                                                                               
                    {                                                                                                         
                        shape_kernel_inflate_list_[i][y+16] |= mask_array_[x+16];                                             
                    }                                                                                                         
                }                                                                                                             
            }                                                                                                                 
            degree += resolution;                                                                                             
        }                                                                                                                     
        std::cout<<"SHape_Kernel_inflate_initialized!"<<std::endl;                                                            
    }
    SShape(ros::NodeHandle &nh):BasicShape(nh)
    {
        std::cout << "ArcShape begin" << std::endl;
        generate_shape_kernel();  // 在这里调用
        generate_shape_kernel_inflate();
        std::cout << "ArcShape end" << std::endl;
    }
    ~SShape(){}
};
#endif