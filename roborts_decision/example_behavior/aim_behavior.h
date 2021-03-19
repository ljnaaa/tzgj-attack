#ifndef ROBORTS_DECISION_AIM_BEHAVIOR_H
#define ROBORTS_DECISION_AIM_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class AimBehavior {
 public:
  AimBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    aim_position.header.frame_id = "map";
    aim_position.pose.orientation.x = 0;
    aim_position.pose.orientation.y = 0;
    aim_position.pose.orientation.z = 0;
    aim_position.pose.orientation.w = 1;

    aim_position.pose.position.x = 0;
    aim_position.pose.position.y = 0;
    aim_position.pose.position.z = 0;

  }

  void Run() {
        if (blackboard_->GetMoveType()==1||blackboard_->GetMoveType()==2||blackboard_->GetMoveType()==3){
	chassis_executor_->Cancel();
	int bg=0;
	blackboard_->MoveTypeCallback(bg);}

    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
	auto aim_position=blackboard_->GetAimPosition();
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = aim_position.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = aim_position.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(aim_position.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(aim_position.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_->Execute(aim_position);

      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~AimBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped aim_position;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
