#include "plan_manager.hpp"
#include <signal.h>

void MySigintHandler(int sig) {
    ros::shutdown();
    // exit(0);
}

int main(int argc,char **argv){
  ros::init(argc, argv, "planmanager");
  ros::NodeHandle nh("~");

  debug_publisher::init(nh);
  ros::Duration(0.5).sleep();
  debug_publisher::DBSendNew("plan_manager", "Program start");

  std::shared_ptr<PlanManager> planmanager_ptr = std::make_shared<PlanManager>(nh);

  signal(SIGINT,MySigintHandler);  
  
  ros::spin();
  return 0;
}