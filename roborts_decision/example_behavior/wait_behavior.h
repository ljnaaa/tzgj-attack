#ifndef ROBORTS_DECISION_WAIT_BEHAVIOR_H
#define ROBORTS_DECISION_WAIT_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class WaitBehavior{
public:
  WaitBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
						blackboard_(blackboard){}

void Run() {
	//
if(blackboard_->GetMoveType()==2||blackboard_->GetMoveType()==3)chassis_executor_->Cancel();
        int movetype=1;
	blackboard_->MoveTypeCallback(movetype);
    auto executor_state = Update();
    if (executor_state != BehaviorState::RUNNING) {
	//Cancel();	
//}
   	 roborts_msgs::TwistAccel t;
 	   //t.twist.angular.z = 1;
   	 //while(1)
  	  //{
		//CBP.publish(t);
  		  if(blackboard_->GetMoveType()==1)
	            chassis_executor_->Execute(t);
              else
              {
                chassis_executor_->Cancel();
                movetype=0;
                	blackboard_->MoveTypeCallback(movetype);
              }

    //}
    }
    //return;
  }
//标记：这里可能定义了无用节点
  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~WaitBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! argc&&argv
  //int argc_in;
  //char **argv_in;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}

#endif
