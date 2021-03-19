#ifndef ROBORTS_DECISION_GIMBALGET0_BEHAVIOR_H
#define ROBORTS_DECISION_GIMBALGET0_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
    class GimbalGet0{
        public:
        GimbalGet0(GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {}
        
        void Run(){
            gimbal_executor_->Update();
	roborts_msgs::GimbalAngle gae;//这里的gae用来发送角度信息给云台执行器
	gae.yaw_mode=0;
	gae.yaw_angle=0;
	gimbal_executor_->Execute(gae);
            //gimbal_executor_->Execute(blackboard_->GetGangle());
        }

        void Cancel(){
            gimbal_executor_->Cancel();
        }

  BehaviorState Update() {
    return gimbal_executor_->Update();
  }
        ~GimbalGet0() = default;

        private:
        GimbalExecutor *gimbal_executor_; 
       Blackboard* const blackboard_;
         geometry_msgs::PoseStamped boot_position_;
    };
}

#endif
