#ifndef ROBORTS_DECISION_SHAKE_BEHAVIOR_H
#define ROBORTS_DECISION_SHAKE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "roborts_msgs/GimbalRate.h"

#include "line_iterator.h"
#include <unistd.h>

namespace roborts_decision {
    class ShakeBehavior{
        public:
        ShakeBehavior(GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {
                orien = 1;
                
       }

        void Run()
        {
                gae.yaw_angle+=0.01*orien;
                //std::cout<<gae.yaw_angle<<std::endl;
                if(gae.yaw_angle<-0.5||gae.yaw_angle>0.5)
                {
                        orien = -orien;
                }
	   	gimbal_executor_->Execute(gae);
        }

        void Cancel(){
            gimbal_executor_->Cancel();
        }

          BehaviorState Update() {
                return gimbal_executor_->Update();
          }

         ~ ShakeBehavior()=default;

     private:
         roborts_msgs::GimbalAngle gae;
         int orien;
        GimbalExecutor *const gimbal_executor_; 
         Blackboard* const blackboard_;
         geometry_msgs::PoseStamped boot_position_;
    };
    }

#endif
