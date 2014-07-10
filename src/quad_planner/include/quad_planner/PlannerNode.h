// Author : PraveenKumar Vasudevan, Matricola: S3999251
#ifndef __PLANNER_NODE_H__
#define __PLANNER_NODE_H__

#include "quad_planner/QuadcopterPlanner.h"
#include "quad_planner/StateMachine.h"
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
            
    planner_.initiate();
    timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&PlannerNode::process, this));
         
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
  ros::Subscriber targetInfo_;
  ros::Subscriber gpsInfo_;
  ros::NodeHandle node_;
  QuadcopterPlanner planner_;
  ros::NodeHandle nh_;
};

#endif