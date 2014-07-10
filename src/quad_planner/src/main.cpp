// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 2 for the course "Robot Programming Methods"
// Description
// This program acts as a planning module for the quadcopter motion planning

#include "quad_planner/PlannerNode.h"

void quit(int sig);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadcopter_collision_planner");
  
  if(!ros::master::check()){
    printf("ROS master doesn't exist\n");
    return(0);
  }

  signal(SIGINT,quit);
  
  PlannerNode node;
  node.run();
  
  ros::spin();
      
  return(0);
}

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}
