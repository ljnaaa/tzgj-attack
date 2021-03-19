#ifndef ROBORTS_DECISION_GOGOAL_BEHAVIORH
#define ROBORTS_DECISION_GOGOAL_BEHAVIORH

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision{
class Gogoal{
 public:
  Gogoal(ChassisExecutor* &chassis_executor,
		Blackboard* &blackboard,
		const std::string & proto_file_path) : chassis_executor_(chassis_executor),
							blackboard_(blackboard) {


    goal_position_.header.frame_id = "map";
    goal_position_.pose.orientation.x = 0;
    goal_position_.pose.orientation.y = 0;
    goal_position_.pose.orientation.z = 0;
    goal_position_.pose.orientation.w = 1;

    goal_position_.pose.position.x = 0;
    goal_position_.pose.position.y = 0;
    goal_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
	//
        if (blackboard_->GetMoveType()==1||blackboard_->GetMoveType()==2||blackboard_->GetMoveType()==3){
	chassis_executor_->Cancel();
	int bg=0;
	blackboard_->MoveTypeCallback(bg);}
    auto executor_state = Update();

    if (executor_state != BehaviorState::RUNNING) {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = goal_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = goal_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto goal_yaw = tf::getYaw(goal_position_.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(goal_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_->Execute(goal_position_);

      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    goal_position_.header.frame_id = "map";

    goal_position_.pose.position.x = decision_config.goal_bot().start_position().x();
    goal_position_.pose.position.z = decision_config.goal_bot().start_position().z();
    goal_position_.pose.position.y = decision_config.goal_bot().start_position().y();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.goal_bot().start_position().roll(),
                                                                     decision_config.goal_bot().start_position().pitch(),
                                                                     decision_config.goal_bot().start_position().yaw());
    goal_position_.pose.orientation = master_quaternion;

    return true;
  }

  ~Gogoal() = default;

 private:
  //！执行器
  ChassisExecutor* const chassis_executor_;

  //！黑板
  Blackboard* const blackboard_;

  //！目标位置
  geometry_msgs::PoseStamped goal_position_;

  //！追击系列位置（？）
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_GO_TO_ANY_GOAL_H
