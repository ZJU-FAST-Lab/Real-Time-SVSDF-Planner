## 关于栅格化地图的bug


这是一个非常严重的bug,会导致碰撞检测等多个模块出现问题.原因是对index的转换和选取有问题.
``` C++
int x = int((goal_(0)+10.0) / 0.1);
int y = int((goal_(1)+10.0) / 0.1);
goal_(0) = x*0.1 - 10.0;
goal_(1) = y*0.1 - 10.0; 
``` 

## 关于离散化角度的bug
规划出的前端轨迹在部分位置出现碰撞,但测试对应的shape_kernel和map_kernel卷积,结果没有任何问题. 

问题出现在yaw和对应的index的转换不正确.比如$\frac{2}{9}\pi + \frac{1}{9}\pi =  \frac{3}{9}\pi$,应该对应`index = 3`,但是由于`int()`计算,以及调用`M_PI`的误差,导致结果是`index = 2`.

```C++

bool AstarPathSearcher::FindYaw(int father_yaw_index, int &child_yaw_index,double father_yaw, double &child_yaw, const Eigen::Vector2i &ind)   // 这里要求传进的ind,,ex是标准的gridmap的inde        
{

    double yaw;
    bool success = false;
    
    double sin_angle = std::sin(father_yaw);
    double cos_angle = std::cos(father_yaw);
    double yaw_nomalized = std::atan2(sin_angle, cos_angle);
    int yaw_index = (yaw_nomalized + M_PI) / resolution_;


    for (int i=0; i<= 3; i++)   //以father_yaw为中心，在-pi～pi内以某一个精度搜索
    {
        child_yaw_index = (father_yaw_index + i) % pieces_;
        success = success || sv_manager_->is_shape_kernel_valiable(ind(0), ind(1), child_yaw_index);
        if(success)
        {
            child_yaw = father_yaw + i* resolution_;
            return true;
        }
        child_yaw_index = (father_yaw_index  - i < 0)?( father_yaw_index  - i + pieces_) : (father_yaw_index - i);
        if(success)                        // 如果上面的那个角度不是合适的角度，试试减去delta
        {
            child_yaw = father_yaw - i* resolution_;
            return true;
        }
    }
    return false;
}
```


## 调试了一个段错误：
在launch文件里面加入：
`launch-prefix="xterm -fa 'Monospace' -fs 14 -e gdb --args`
发现是`shape_kernel_inflate`没有初始化导致的



## 发现了一个碰撞点的bug
![alt text](pic/碰撞点bug.png)

问题是碰撞点检测的时候多了一行导致的。


## 发现了replan的bug

replan中，切换轨迹的时候对轨迹的开始位置非常敏感。
因此务必要注意控制点转换为Bspline的中端需要固定起点和终点，如果不固定就会因为shape kernel index离散化中的数值计算问题导致规划反复横跳。

``` C++
// SET Start and End Condition
    ctrl_pts.col(0) = waypoints[0] - start_end_derivative[0]*ts + start_end_derivative[2]*ts*ts/3.0;
    ctrl_pts.col(1) = waypoints[0]                           - start_end_derivative[2]*ts*ts/6.0;
    ctrl_pts.col(2) = waypoints[0] + start_end_derivative[0]*ts + start_end_derivative[2]*ts*ts/3.0;

    ctrl_pts.col(K - 1) = waypoints[K-1] - start_end_derivative[1]*ts + start_end_derivative[3]*ts*ts/3.0;
    ctrl_pts.col(K    ) = waypoints[K-1]                              - start_end_derivative[3]*ts*ts/6.0;
    ctrl_pts.col(K + 1) = waypoints[K-1] + start_end_derivative[1]*ts + start_end_derivative[3]*ts*ts/3.0;
```

在`initiallize`这个函数里面，加入了对初末位置的强行赋值，这样可以保证新规划的轨迹和目前正在执行的轨迹是完全能衔接得上的

## 对于后端的思路
需要一个超参数`horizen`，只会选取一小部分前端进行优化

从前端返回的数据开始修改即可。

这个问题太严重了，就是前端搜索轨迹的起点其实是依赖后端的。


###  具体步骤
1.  done 加入debug过程中的步进操作
2.  done 可以复现的地图和规划起点终点
3.  展现3个cost如何降低的曲线
4.  随时可以可视化梯度和sdf场


5.  done 前端肯定有问题：（1）碰撞点的选取没有问题，但是给出来的前端会碰撞
（2）用来查询的数据有问题（3）先在前端输出一次所有点是否碰撞，看看与实际情况是否相符（点的位置/角度），排除可视化问题和碰撞检测问题，检查findyaw函数。


（1） FindYaw函数果然有问题、
（2）碰撞检测也有问题

真就屎山代码


23.06 line search导致耗时非常严重。


launch-prefix="valgrind --tool=callgrind --collect-jumps=yes"






template <class TOPIC>
        inline void pubFloat64(const TOPIC &topic, const double msg)
        {
            auto got = publisher_map_.find(topic); // 如果unordered_map中没有存储这个键值对，即插入这个对
            if (got == publisher_map_.end())
            {
                ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10000);
                publisher_map_[topic] = pub;
            }
            std_msgs::Float64 datamsg;
            datamsg.data = msg;
            publisher_map_[topic].publish(datamsg);
        }