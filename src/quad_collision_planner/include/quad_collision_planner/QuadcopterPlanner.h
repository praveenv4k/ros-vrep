#ifndef __QUADCOPTER_PLANNER_H__
#define __QUADCOPTER_PLANNER_H__

// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 3 for the course "Robot Programming Methods"
// Description
// This program acts as a planning module for the quadcopter motion planning

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/String.h>
#include <iostream>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <signal.h>
#include <termios.h>
#include <tf/transform_listener.h>

#include <queue>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/mpl/list.hpp>

#include <quad_collision_planner/ProximitySensorData.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

namespace mpl = boost::mpl; 
namespace fsm = boost::statechart;

struct Idle;
struct EvInitialize : fsm::event< EvInitialize > {};
struct EvClock : fsm::event< EvClock > {};

struct QuadcopterPlanner:fsm::state_machine< QuadcopterPlanner, Idle >
{
  QuadcopterPlanner();
  void mainLoop();
  void watchdog();
  void init();
  void gpsCallback(const std_msgs::String::ConstPtr& sens);
  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& sens);
  
  void proximityCallback(const vrep_common::ProximitySensorDataConstPtr& prox);  
  void processProximityData(const vrep_common::ProximitySensorDataConstPtr& prox);
  
  void supervisorCmdCallback(const geometry_msgs::Pose::ConstPtr& target);
  void computeResidualDistance(const geometry_msgs::Point& p1,const geometry_msgs::Point& p2,geometry_msgs::Point& residue){
    residue.x = p2.x-p1.x;
    residue.y = p2.y-p1.y;
    residue.z = p2.z-p1.z;
  }
  void computeStep(const geometry_msgs::Point& p1,geometry_msgs::Point& ptStep,int steps){
    ptStep.x = p1.x/steps;
    ptStep.y = p1.y/steps;
    ptStep.z = p1.z/steps;
  }

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
    }
    return str;
  }
  
  bool areEqual(double a,double b,double threshold){
    return fabs(a-b)<=threshold;
  }
  void publish(geometry_msgs::PosePtr pose);
  void publishSts(uint8_t sts,std::string text, std::string id);
  ros::Time first_publish_;
  ros::Time last_publish_;
  geometry_msgs::Pose quad_pos;
  geometry_msgs::Point gps_pos;
  ros::NodeHandle nh_,ph_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  boost::mutex publish_mutex_;
  boost::mutex target_mutex_;
  geometry_msgs::Point wayPtThreshold_;
  actionlib_msgs::GoalStatus targetReached;
  actionlib_msgs::GoalStatus viaptReached;
  geometry_msgs::Pose finalTarget_;
  geometry_msgs::Pose startPose_;
  bool reached_;
  bool reached2_;
  geometry_msgs::Point step_;
  geometry_msgs::Pose next_target;
  geometry_msgs::Point min_threshold;
  int target_id;
  bool debug;
  int proxDebug;
  
  typedef struct __tagProximityData{
    geometry_msgs::Point32 _point;
    tf::Vector3 _normal;
    double distance;
    double threshold;
    int sensorId;
  }ProximityData;
  
  typedef struct __tagTransformData{
    tf::StampedTransform tf;
  }TransformData;
  
  typedef struct __tagCollisionData{
    bool collision;
    int avoidStep;
    tf::Vector3 steerVector;
  }CollisionData;
  
  ProximityData proxityData_;
  TransformData transformData_;
  CollisionData collisionData_;
};

#endif

