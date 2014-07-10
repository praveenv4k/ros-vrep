// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 1 for the course "Robot Programming Methods"
// Description
// This program controls a Quadcopter simulated in VREP software using keyboard keys
// Controls
// KEYS     ACTION
// 'a'      +X
// 's'      -X
// 'd'      +Y
// 'f'      -X
// 'w'      +Z
// 'x'      -Z

#include "quad_kbctrl/QuadcopterTeleop.h"

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadcopter_teleop");
  if(!ros::master::check()){
    printf("ROS master doesn't exist\n");
    return(0);
  }
  
  signal(SIGINT,quit);
  
  QuadcopterTeleop quadcopter_teleop(kfd);
  ros::NodeHandle node("~");
  ros::Subscriber targetInfo=node.subscribe("/vrep/quadtargetpose",1,&QuadcopterTeleop::targetCallback,&quadcopter_teleop);
  printf("Subscribed to quadcopter pose data\n");
  	
  boost::thread my_thread(boost::bind(&QuadcopterTeleop::keyLoop, &quadcopter_teleop));
  
  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}



