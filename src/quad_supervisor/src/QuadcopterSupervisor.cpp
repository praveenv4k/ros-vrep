// Author : PraveenKumar Vasudevan, Matricola: S3999251
// Developed as part of Assignment 2/3 for the course "Robot Programming Methods"
// Description
// This program acts as a supervisor for the quadcopter motion planning

#include "quad_supervisor/QuadcopterSupervisor.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>

QuadcopterSupervisor::QuadcopterSupervisor():
  ph_("~"){
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/app/plannercmd",1); 
  
  std::string configFile;
  
  XmlRpc::XmlRpcValue targetList;
  if(ph_.getParam("targets",targetList)){
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    for (i = targetList.begin(); i != targetList.end(); i++)
    {
      std::string targetname;
      std::string target;

      targetname = i->first;
      target.assign(i->second["value"]);
      std::cout << targetname << "," << target << std::endl;
      
      geometry_msgs::Pose pose1;
      std::stringstream os(target.c_str());
      os >> pose1.position.x>> pose1.position.y>> pose1.position.z;
      targets.push_back(pose1);
    }
    int repeat=1;
    if(ph_.getParam("repeat",repeat)){
      repeat_ = repeat;
    }
    std::cout << repeat_ << "," << targets.size() << std::endl;
  }else{
    tf::Quaternion quat180(tf::Vector3(0,0,1),M_PI);
    tf::Quaternion quat0(tf::Vector3(0,0,1),0);

    geometry_msgs::Pose pose1;
    pose1.position.x = -1.3;
    pose1.position.y = 0;
    pose1.position.z = 0.5;
    pose1.orientation.x = quat0.x();
    pose1.orientation.y = quat0.y();
    pose1.orientation.z = quat0.z();
    pose1.orientation.w = quat0.w();
    
    targets.push_back(pose1);
    
    pose1.position.x = 1.6;
    pose1.position.y = 0;
    pose1.position.z = 0.5;
    pose1.orientation.x = quat0.x();
    pose1.orientation.y = quat0.y();
    pose1.orientation.z = quat0.z();
    pose1.orientation.w = quat0.w();
    
    targets.push_back(pose1);
    
    pose1.position.x = 1.3;
    pose1.position.y = 0;
    pose1.position.z = 0.5;
    pose1.orientation.x = quat0.x();
    pose1.orientation.y = quat0.y();
    pose1.orientation.z = quat0.z();
    pose1.orientation.w = quat0.w();
    
    targets.push_back(pose1);
    
    pose1.position.x = -1.6;
    pose1.position.y = 0;
    pose1.position.z = 0.5;
    pose1.orientation.x = quat0.x();
    pose1.orientation.y = quat0.y();
    pose1.orientation.z = quat0.z();
    pose1.orientation.w = quat0.w();
    
    targets.push_back(pose1);
    
    repeat_ = 1;
  }

  targetId =0;
  step = 0;
  reached = false;
  count_ = 0;
}

void QuadcopterSupervisor::plannerStsCallback(const actionlib_msgs::GoalStatus::ConstPtr& sts)
{
  plannerExist = true;
  if(sts->status == 10){
    reached = true;
  }
  status_ = *sts;
  if(sts->text.size()>0){
    std::cout << "Planner Status:  {"<< getGoalStatus(sts->status) << "} "<< sts->text << " " << sts->goal_id.id << std::endl;
  }
}

void QuadcopterSupervisor::init(){
  puts("QuadcopterSupervisor");
  puts("---------------------------");
}

void QuadcopterSupervisor::mainLoop()
{
  if (ros::ok())
  {
    int newStep = step;
    switch(step){
      case 0: // Idle
	if(plannerExist && status_.status != 1 && targets.size()>0){
	  step++;
	  name = "Send";
	  reached = false;
	}
	break;
      case 1: // Send
	if(status_.status != 1){
	  pose = targets[targetId];
	  publish(pose);
	  std::cout << "Sending new target: " << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
	  step++;
	  name = "Wait";
	}
	break;
      case 2: //Dummy states
      case 3:
	step++;
	break;
      case 4: // Wait
	if(status_.status == 5){
	  name = "Send";
	  step=1;
	}
	if(reached){
	  name = "Final";
	  step++;
	}else if(status_.status == 9){
	  name = "Error";
	  step=9;
	}
	break;
      case 5: // Final
	if(status_.status != 1){
	  name = "Idle";
	  targetId++;
	  bool finish=false;
	  if(targetId>=targets.size()){
	    count_++;
	    if(count_>=repeat_){
	      finish=true;
	    }
	    targetId=0;
	  }
	  if(!finish){
	    step++;
	  }
	  else{
	    name = "Dormant";
	    std::cout << "Supervisor Node operation complete!" << std::endl;
	    step=10;
	  }
	}
	break; 
      case 6: //Dummy states
      case 7:
	step++;
	break;
      case 8:
	step = 0;
	break;
      case 9: // Error
	name = "Idle";
	step = 0;
	break;
      case 10:
	break;
      default:
	break;
    }
    if(step!=newStep){
      std::cout << "Transition to " << name << " state" << std::endl;
    }
  }
  return;
}

void QuadcopterSupervisor::publish(geometry_msgs::Pose& pose)  
{
  pose_pub_.publish(pose);
  return;
}

