#ifndef ROBORTS_DECISION_DANCE_BEHAVIOR_H
#define ROBORTS_DECISION_DANCE_BEHAVIOR_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "roborts_msgs/GimbalRate.h"
#include "../executor/gimbal_executor.h"

#include "line_iterator.h"
#include <unistd.h>
#include "std_msgs/Int16.h"
//扭腰动作，Movetype=3，还在完善中
namespace roborts_decision {
    class DanceBehavior{
        public:
  DanceBehavior(ChassisExecutor* &chassis_executor,
                GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor), 
                gimbal_executor_(gimbal_executor),
						blackboard_(blackboard){
                        DanceCount=100;
                        orien = 1;
			if_first=0;
			p=0.8;}

    void Run()
    {
if(blackboard_->GetMoveType()==2||blackboard_->GetMoveType()==1)chassis_executor_->Cancel();
        int movetype=3;
	blackboard_->MoveTypeCallback(movetype);
        auto executor_state_cha = Update_chassis();
        auto executor_state_gim = Update_gimbal();
	if(if_first==0){
		if_first=1;
		origin_gimbal=blackboard_->GetGimbalAngle();
       
		}
        t.twist.angular.z = 0.4*orien;
        DanceCount++;
        if(DanceCount == 200)
        {
            DanceCount = 0;
            orien = -orien;
        }
                gimbal_angle_execute.yaw_mode=true;
		        //std::cout<<"bias:"<<blackboard_->GetGimbalAngle()-origin_gimbal<<std::endl;
                gimbal_angle_execute.yaw_angle=-1*t.twist.angular.z*0.02-p*(blackboard_->GetGimbalAngle()-origin_gimbal);
                gimbal_executor_->Execute(gimbal_angle_execute);
                chassis_executor_->Execute(t);
    }

    void Cancel()
    {
        chassis_executor_->Cancel();
        gimbal_executor_->Cancel();
    }

    BehaviorState Update_chassis() {
    return chassis_executor_->Update();
      }

    BehaviorState Update_gimbal() {
    return gimbal_executor_->Update();
    }

     private:
        roborts_msgs::GimbalAngle gimbal_angle_execute;//最终发送的云台角度数据
         int DanceCount;//扭腰计数器，以200为周期
         int orien;//记录方向，其值为+1或者-1
	bool if_first;//用来表示是否第一次进入行为，初始为0，进入后为1
	double p;//回拉比例系数，默认0.01
	double origin_gimbal;//初始位置角度
         roborts_msgs::TwistAccel t;
        ChassisExecutor* const chassis_executor_;
        GimbalExecutor *const gimbal_executor_; 
         Blackboard* const blackboard_;
         geometry_msgs::PoseStamped boot_position_;
    };
}

#endif
