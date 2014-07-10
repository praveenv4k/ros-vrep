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
#include <sstream>

// Keyboard control preprocessor definition
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_F 0x66
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78

// Constructor
QuadcopterTeleop::QuadcopterTeleop(int kb_handle):
  ph_("~"){
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/app/quadcmdpose",1);
  kfd = kb_handle;
  resolution_ = 0.005;
}

// Callback
void QuadcopterTeleop::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& sens)
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  quad_pos = sens->pose;
}

// Main loop
void QuadcopterTeleop::keyLoop()
{
  char c;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the quadcopter.");
  puts("Usage : Press the key");
  puts("'A' : -X direction, 'S' : +X direction");
  puts("'D' : -Y direction, 'F' : +Y direction");
  puts("'X' : -Z direction, 'W' : +Z direction");

  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X\n", c);
  
    geometry_msgs::Pose target_pose;
    {
      boost::mutex::scoped_lock lock(publish_mutex_);
      target_pose = quad_pos;
    }
    
    bool validKey = true;
    double deltaX = 0;
    double deltaY = 0;
    double deltaZ = 0;
    switch(c)
    { 
      case KEYCODE_A:
	ROS_DEBUG("X-");
	deltaX = -1;
	break;
      case KEYCODE_S:
	ROS_DEBUG("X+");
	deltaX = 1;
	break;
      case KEYCODE_D:
	deltaY = -1;
	ROS_DEBUG("Y-");
	break;
      case KEYCODE_F:
	deltaY = 1;
	ROS_DEBUG("Y+");
	break;
      case KEYCODE_X:
	deltaZ = -1;
	ROS_DEBUG("Z-");
	break;
      case KEYCODE_W:
	deltaZ = 1;
	ROS_DEBUG("Z+");
	break;
      default:
	validKey = false;
	break;
    }
    if(validKey){
      target_pose.position.x = target_pose.position.x+deltaX*resolution_;
      target_pose.position.y = target_pose.position.y+deltaY*resolution_;
      target_pose.position.z = target_pose.position.z+deltaZ*resolution_;
      publish(target_pose);
    }
  }

  return;
}

// To publish the topic
void QuadcopterTeleop::publish(geometry_msgs::Pose& pose)  
{
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose = pose;
  pose_pub_.publish(poseStamped);    
  return;
}

