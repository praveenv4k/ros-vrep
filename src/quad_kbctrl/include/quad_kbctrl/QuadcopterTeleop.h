#ifndef __QUADCOPTER_TELEOP_H__
#define __QUADCOPTER_TELEOP_H__

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

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <iostream>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <signal.h>
#include <termios.h>

class QuadcopterTeleop
{
public:
  QuadcopterTeleop(int kb_handle);
  void keyLoop();
  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& sens);
private:
  ros::Time first_publish_;
  ros::Time last_publish_;
  geometry_msgs::Pose quad_pos;
  ros::NodeHandle nh_,ph_;
  ros::Publisher pose_pub_;
  void publish(geometry_msgs::Pose& pose);
  boost::mutex publish_mutex_;
  int kfd;
  double resolution_;
  struct termios cooked, raw;
};

#endif

