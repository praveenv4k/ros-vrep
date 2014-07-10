// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 2 for the course "Robot Programming Methods"
// Description
// This program acts as a planning module for the quadcopter motion planning

#include "quad_planner/QuadcopterPlanner.h"
#include <limits>
QuadcopterPlanner::QuadcopterPlanner():
  ph_("~"){
    
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/app/quadcmdpose",1);
    status_pub_ = nh_.advertise<actionlib_msgs::GoalStatus>("/app/plannersts",1);
      
    bool prmAvail = false;
    std::string param;
    if(ph_.getParam("wayPtThreshold",param)){
      std::stringstream os1(param.c_str());
      os1 >> wayPtThreshold_.x>> wayPtThreshold_.y>> wayPtThreshold_.z;
      if(ph_.getParam("minThreshold",param)){
	std::stringstream os2(param.c_str());
	os2 >> min_threshold.x>> min_threshold.y>> min_threshold.z;
	if(ph_.getParam("step",param)){
	  std::stringstream os3(param.c_str());
	  os3 >> step_.x>> step_.y>> step_.z;
	  std::cout << "Parameter read from launch file!" << std::endl;
	  prmAvail=true;
        }
      }
    }
    if(!prmAvail){      
      wayPtThreshold_.x = 5e-3*3;
      wayPtThreshold_.y = 5e-3*3;
      wayPtThreshold_.z = 5e-3*3;
      
      min_threshold.x = 5e-3*3;
      min_threshold.y = 5e-3*3;
      min_threshold.z = 5e-3*3;
      
      step_.x = 1e-2*3;
      step_.y = 1e-2*3;
      step_.z = 1e-2*3;
    }

    reached_ = true;
    reached2_=true;
    target_id = 0;
    proxDebug = 0;
}

void QuadcopterPlanner::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& sens)
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  quad_pos = sens->pose;
}

void QuadcopterPlanner::gpsCallback(const std_msgs::String::ConstPtr& sens)
{
  std::stringstream os(sens->data.c_str());
  os >> gps_pos.x>> gps_pos.y>> gps_pos.z;
}

void QuadcopterPlanner::supervisorCmdCallback(const geometry_msgs::Pose::ConstPtr& target)
{
  if(targetReached.status != actionlib_msgs::GoalStatus::ACTIVE){
      boost::mutex::scoped_lock lock(target_mutex_);
      if(!reached_){
	publishSts(actionlib_msgs::GoalStatus::REJECTED,"Target Rejected","");
	return;
      }
      
      std::cout << "Received new target... ";
      std::cout << "[ "<<target->position.x << "," <<target->position.y<<","<<target->position.z<<" ]"<<std::endl;
      
      geometry_msgs::Point gps=gps_pos;
      geometry_msgs::Point residue;

      std::cout << "Threshold : " << wayPtThreshold_.x << "," <<wayPtThreshold_.y<<","<<wayPtThreshold_.z<<std::endl;
      reached_ = false;
      reached2_ = false;
      target_id = 0;
      startPose_.position = gps_pos;
      finalTarget_ = *target;
  }
}

void QuadcopterPlanner::init(){
}

void QuadcopterPlanner::publish(geometry_msgs::PosePtr pose)  
{
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose = *pose;
  pose_pub_.publish(poseStamped);
}

void QuadcopterPlanner::publishSts(uint8_t sts,std::string text, std::string id){
    actionlib_msgs::GoalStatus status;
    status.status = sts;
    status.text = text;
    status.goal_id.id = id;
#if 0
    std::cout << "Publishing Status:  "<< getGoalStatus(sts) << " "<< text << " " << id << std::endl;
#endif
    status_pub_.publish(status);
}

