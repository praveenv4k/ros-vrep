#ifndef __QUADCOPTER_SUPERVISOR_H__
#define __QUADCOPTER_SUPERVISOR_H__

// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 2/3 for the course "Robot Programming Methods"
// Description
// This program acts as a supervisor for the quadcopter motion planning

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatus.h>
#include <iostream>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <signal.h>
#include <termios.h>

#include <vector>

class QuadcopterSupervisor
{
public:
  QuadcopterSupervisor();
  void mainLoop();
  void watchdog();
  void init();
  void plannerStsCallback(const actionlib_msgs::GoalStatus::ConstPtr& sts);
  std::string getGoalStatus(uint8_t sts){
    std::string str = "";
    switch(sts){
      case 0:
	str = "Pending";
	break;
      case 1:
	str = "Active";
	break;
      case 2:
	str = "Preempted";
	break;
      case 3:
	str = "Succeeded";
	break;
      case 4:
	str = "Aborted";
	break;
      case 5:
	str = "Rejected";
	break;
      case 6:
	str = "Preempting";
	break;
      case 7:
	str = "Recalling";
	break;
      case 8:
	str = "Recalled";
	break;
      case 9:
	str = "Lost";
	break;
      case 10:
	str = "Ready";
	break;
    }
    return str;
  }
private:
  ros::Time first_publish_;
  ros::Time last_publish_;
  ros::NodeHandle nh_,ph_;
  ros::Publisher pose_pub_;
  void publish(geometry_msgs::Pose& pose);
  boost::mutex publish_mutex_;
  actionlib_msgs::GoalStatus status_;
  std::vector<geometry_msgs::Pose> targets;
  geometry_msgs::Pose pose;
  size_t targetId;
  int step;
  std::string name;
  int count;
  bool plannerExist;
  bool reached;
  int repeat_;
  int count_;
};

#endif

