#include "simulator/simulator.hpp"
#include <signal.h>

void MySigintHandler(int sig) {
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh("~");
  
  Simulator simulator(nh);
  signal(SIGINT,MySigintHandler);  

  ros::spin();

  return 0;
}
