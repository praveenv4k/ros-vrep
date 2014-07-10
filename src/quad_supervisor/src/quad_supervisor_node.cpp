// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 2/3 for the course "Robot Programming Methods"
// Description
// This program acts as a supervisor for the quadcopter motion planning

#include "quad_supervisor/QuadcopterSupervisor.h"

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadcopter_supervisor");
  if(!ros::master::check()){
    printf("ROS master doesn't exist\n");
    return(0);
  }
  
  QuadcopterSupervisor quadcopter_supervisor;
  quadcopter_supervisor.init();

  signal(SIGINT,quit);
  
  ros::NodeHandle node("~");
  ros::Subscriber targetInfo=node.subscribe("/app/plannersts",1,&QuadcopterSupervisor::plannerStsCallback,&quadcopter_supervisor);
  printf("Subscribed to quadcopter pose data\n");
  	
  //boost::thread my_thread(boost::bind(&QuadcopterSupervisor::mainLoop, &quadcopter_supervisor));
  ros::NodeHandle nh;
  ros::Timer timer_ = nh.createTimer(ros::Duration(1), boost::bind(&QuadcopterSupervisor::mainLoop, &quadcopter_supervisor));
  
  ros::spin();

  //my_thread.interrupt() ;
  //my_thread.join() ;
      
  return(0);
}



