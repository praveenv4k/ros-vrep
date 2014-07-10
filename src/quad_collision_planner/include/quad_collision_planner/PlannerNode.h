// Author : PraveenKumar Vasudevan, Matricola: S3999251
#ifndef __PLANNER_NODE_H__
#define __PLANNER_NODE_H__

#include "quad_collision_planner/QuadcopterPlanner.h"
#include "quad_collision_planner/StateMachine.h"
#include <vector>

class PlannerNode{
public:
  PlannerNode():node_("~"){
  }
  void run(){
    targetInfo_=node_.subscribe("/vrep/quadtargetpose",1,&QuadcopterPlanner::targetCallback,&planner_);
    std::cout << "Subscribed to quadcopter pose data\n";

    gpsInfo_=node_.subscribe("/vrep/quadgpspos",1,&QuadcopterPlanner::gpsCallback,&planner_);
    std::cout << "Subscribed to quadcopter gps data\n";
  
    supInfo_=node_.subscribe("/app/plannercmd",1,&QuadcopterPlanner::supervisorCmdCallback,&planner_);
    std::cout << "Subscribed to supervision cmd\n";
    
    proxData1_=node_.subscribe("/vrep/proxData",1,&QuadcopterPlanner::proximityCallback,&planner_);
    std::cout << "Subscribed to proximity sensor data\n";
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    try{
      std::cout << "Retrieving Sensor Transforms from /tf topic\n";
      
      std::string parentFrame = "/Quadricopter";
      std::string targetFrame1 = "/Proximity_sensor";
      
      if(listener.waitForTransform(targetFrame1, parentFrame,
			      ros::Time(0),ros::Duration(2.0))){
	listener.lookupTransform(targetFrame1,parentFrame,ros::Time(0),planner_.transformData_.tf);
        tf::Quaternion quat1 = planner_.transformData_.tf.getRotation();
        disp(quat1);
        planner_.initiate();
	timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&PlannerNode::process, this));
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }    
  }
  
  void disp(tf::Quaternion& q){
    std::cout << q.x() << ","<< q.y()<<","<<q.z() <<"," <<q.w() <<std::endl;
  }
  
  void process(){
    planner_.process_event(EvClock());
  }
private:
  ros::Timer timer_;
  ros::Subscriber supInfo_;
  ros::Subscriber proxData1_;
  ros::Subscriber proxData2_;
  ros::Subscriber proxData3_;
  ros::Subscriber proxData4_;
  ros::Subscriber targetInfo_;
  ros::Subscriber gpsInfo_;
  ros::NodeHandle node_;
  QuadcopterPlanner planner_;
  ros::NodeHandle nh_;
};

#endif