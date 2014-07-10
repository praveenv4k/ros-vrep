// Author : PraveenKumar Vasudevan, Matricola: S3999251
#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#define VERBOSE_MODE 1
#define USEGPS 1

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/in_state_reaction.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deferral.hpp>

namespace mpl = boost::mpl; 
namespace fsm = boost::statechart;

struct Idle;
struct Run;
struct Wait;
struct Final;
struct Error;

#include "quad_planner/QuadcopterPlanner.h"

// Idle state
struct Idle: fsm::state<Idle,QuadcopterPlanner>{
  Idle( my_context ctx):my_base(ctx){
#if VERBOSE_MODE
    std::cout << "Entered the Idle State" << std::endl;
#endif
  }
  typedef mpl::list<fsm::custom_reaction<EvClock> > reactions;

  fsm::result react( const EvClock &evClock ){
    if(!context<QuadcopterPlanner>().reached_){
      return transit<Run>();
    }
    context<QuadcopterPlanner>().publishSts(actionlib_msgs::GoalStatus::SUCCEEDED,"","");
    return forward_event();
  }
};

// Running state. Send the targets one by one 
struct Run: fsm::state<Run,QuadcopterPlanner>{
  
  Run(my_context ctx):my_base(ctx){
#if VERBOSE_MODE
    std::cout << "Entered the Run State" << std::endl;
#endif
  }
  
  typedef mpl::list<fsm::custom_reaction<EvClock> > reactions;
  
  fsm::result react( const EvClock &evClock ){
    if(context<QuadcopterPlanner>().reached2_){
      return transit<Final>();
    }
    else{
      geometry_msgs::PosePtr pPose(new geometry_msgs::Pose());
      
#if USEGPS
      geometry_msgs::Point gps = context<QuadcopterPlanner>().gps_pos;
#else
      geometry_msgs::Point gps = context<QuadcopterPlanner>().quad_pos.position;
#endif
      
      geometry_msgs::Point step = context<QuadcopterPlanner>().step_;
      geometry_msgs::Pose target = context<QuadcopterPlanner>().finalTarget_;      
      geometry_msgs::Pose start = context<QuadcopterPlanner>().startPose_;
      if(context<QuadcopterPlanner>().areEqual(gps.x,target.position.x,fabs(0.5*step.x)) && 
	 context<QuadcopterPlanner>().areEqual(gps.y,target.position.y,fabs(0.5*step.y)) && 
	 context<QuadcopterPlanner>().areEqual(gps.z,target.position.z,fabs(0.5*step.z))){
	std::cout << "Next target is final target" << std::endl;
	pPose->position=target.position;
      }
      else{
	tf::Vector3 vec(target.position.x-gps.x,
			     target.position.y-gps.y,
			     target.position.z-gps.z);
        tf::Vector3 unitVector = vec.normalized();
      
	pPose->position.x=gps.x+step.x*unitVector.getX();
	pPose->position.y=gps.y+step.y*unitVector.getY();
	pPose->position.z=gps.z+step.z*unitVector.getZ();
      }
      pPose->orientation=target.orientation;
      context<QuadcopterPlanner>().next_target = *pPose;
      context<QuadcopterPlanner>().target_id++;
      context<QuadcopterPlanner>().publish(pPose);
      context<QuadcopterPlanner>().debug = true;
      return transit<Wait>();
    }
  }
};

// Wait state. Send the targets one by one 
struct Wait: fsm::state<Wait,QuadcopterPlanner>{
  
  Wait(my_context ctx):my_base(ctx){
#if VERBOSE_MODE
    std::cout << "Entered the wait State" << std::endl;
#endif
  }
   
  fsm::result react( const EvClock &evClock ){
    geometry_msgs::Pose pose = context<QuadcopterPlanner>().next_target;
    
#if USEGPS
      geometry_msgs::Point gps = context<QuadcopterPlanner>().gps_pos;
#else
      geometry_msgs::Point gps = context<QuadcopterPlanner>().quad_pos.position;
#endif
    
    geometry_msgs::Point thres = context<QuadcopterPlanner>().wayPtThreshold_;
    geometry_msgs::Pose finalTarget = context<QuadcopterPlanner>().finalTarget_;
    
    if(context<QuadcopterPlanner>().debug){
      
#if VERBOSE_MODE
      std::cout  << " Next Target: " << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
      std::cout  << " GPS Pos: " << gps.x << "," << gps.y << "," << gps.z << std::endl;
      std::cout  << " Abs Diff: " << fabs(gps.x-pose.position.x) << "," << fabs(gps.y-pose.position.y) << "," << fabs(gps.z-pose.position.z) << std::endl;
#endif
      context<QuadcopterPlanner>().debug = false;
    }
        
    if(context<QuadcopterPlanner>().areEqual(gps.x,pose.position.x,thres.x) && 
       context<QuadcopterPlanner>().areEqual(gps.y,pose.position.y,thres.y) && 
       context<QuadcopterPlanner>().areEqual(gps.z,pose.position.z,thres.z)){
      int goalId =  context<QuadcopterPlanner>().target_id;
      std::ostringstream str("");
      str << "Id: "<< goalId << " Position: " << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
      context<QuadcopterPlanner>().publishSts(actionlib_msgs::GoalStatus::ACTIVE,"WayPoint Reached",str.str());
    
#if VERBOSE_MODE
      std::cout << "Way point reached" << std::endl;
#endif
      
      if(context<QuadcopterPlanner>().areEqual(pose.position.x,finalTarget.position.x,fabs(0.5*context<QuadcopterPlanner>().step_.x)) && 
	 context<QuadcopterPlanner>().areEqual(pose.position.y,finalTarget.position.y,fabs(0.5*context<QuadcopterPlanner>().step_.y)) && 
	 context<QuadcopterPlanner>().areEqual(pose.position.z,finalTarget.position.z,fabs(0.5*context<QuadcopterPlanner>().step_.z))){
	context<QuadcopterPlanner>().reached2_ = true;
      }
      return transit<Run>();
    }
    else{
    }
    return forward_event();
  }
  
  typedef mpl::list<fsm::custom_reaction<EvClock> > reactions;
};

// Final State
struct Final: fsm::state<Final,QuadcopterPlanner>{
  
  Final(my_context ctx):my_base(ctx){
#if VERBOSE_MODE
    std::cout << "Entered the Final State" << std::endl;
#endif
  }
  
  typedef mpl::list<fsm::custom_reaction<EvClock> > reactions;
	
  fsm::result react( const EvClock &evClock ){
    std::ostringstream str("");
    geometry_msgs::Pose target = context<QuadcopterPlanner>().finalTarget_;
    str << " Position: " << target.position.x << "," << target.position.y << "," << target.position.z << std::endl;
    context<QuadcopterPlanner>().publishSts(actionlib_msgs::GoalStatus::ACTIVE,"Final Target Reached",str.str());
    context<QuadcopterPlanner>().publishSts(10,"Ready",str.str());
    context<QuadcopterPlanner>().reached_ = true;
    return transit<Idle>();
  }
};

// Error State
struct Error: fsm::state<Error,QuadcopterPlanner>{
  
  Error(my_context ctx):my_base(ctx){
    std::cout << "Entered the Error State" << std::endl;
  }
  
  typedef mpl::list<fsm::custom_reaction<EvClock> > reactions;
	
  fsm::result react( const EvClock &evClock ){
    std::cout << "Target reach failed" << std::endl;
    context<QuadcopterPlanner>().publishSts(actionlib_msgs::GoalStatus::LOST,"Target reach failed","");
    return transit<Idle>();
  }
};

#endif