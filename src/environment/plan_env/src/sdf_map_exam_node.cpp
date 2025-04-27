
#include "plan_env/sdf_map.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "sdf_map_exam_node");

    ros::NodeHandle nh;
    // Create SDFmap object
    SDFmap sdf_map(nh);

    ros::spin();
    
    return 0;
}


