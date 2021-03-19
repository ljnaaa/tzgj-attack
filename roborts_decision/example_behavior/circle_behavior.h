#ifndef ROBORTS_DECISION_CIRCLE_BEHAVIOR_H
#define ROBORTS_DECISION_CIRCLE_BEHAVIOR_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "io/io.h"
#include <iostream>
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
//旋转动作，Movetype=2
namespace roborts_decision {
class CircleBehavior {
 public:
  CircleBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
						blackboard_(blackboard){}

  void Run() {

	if(blackboard_->GetMoveType()==1||blackboard_->GetMoveType()==3)chassis_executor_->Cancel();
    	int movetype=2;
	blackboard_->MoveTypeCallback(movetype);
    auto executor_state = Update();
    if (executor_state != BehaviorState::RUNNING) {
   	 roborts_msgs::TwistAccel t;
 	   t.twist.angular.z = 0.1;
	chassis_executor_->Execute(t);
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~CircleBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}

#endif 
