# BSplineTrajectory.msg

std_msgs/Header Header          # 标准头信息

time traj_start_time            # 轨迹开始时间

geometry_msgs/Vector3[] control_points  # 控制点序列
float64[] u                 # 时间点（每个控制点对应的时间）

geometry_msgs/Vector3 init_p    # 初始位置
geometry_msgs/Vector3 init_v    # 初始速度
geometry_msgs/Vector3 init_a    # 初始加速度

geometry_msgs/Vector3 tail_p    # 末尾位置
geometry_msgs/Vector3 tail_v    # 末尾速度
geometry_msgs/Vector3 tail_a    # 末尾加速度
