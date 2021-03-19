#ifndef ROBORTS_DECISION_GIMBAL_AIM_BEHAVIOR_H
#define ROBORTS_DECISION_GIMBAL_AIM_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
	class GimbalAimBehavior{
	public:
        GimbalAimBehavior(GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {}

	void Run()
	{
		gimbal_executor_->Update();
		roborts_msgs::GimbalAngle gae=blackboard_->GetGangle();
		gimbal_executor_->Execute(gae);		
	}

        void Cancel(){
            gimbal_executor_->Cancel();
        }

  BehaviorState Update() {
    return gimbal_executor_->Update();
  }
        ~GimbalAimBehavior() = default;

        private:
        GimbalExecutor *gimbal_executor_; 
       Blackboard* const blackboard_;
         geometry_msgs::PoseStamped boot_position_;
};
}
#endif
