#include <ros/ros.h>
#include "executor/chassis_executor.h"
//#include "executor/gimbal_executor.h"
#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_tree.h"
#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/go_to_any_goal.h"
#include "example_behavior/circle_behavior.h"


int patrolcondition_;
int circlecondition_;

void param_init(ros::NodeHandle& p)
{
	p.param("patrol_condition", patrolcondition_,0);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "tree_test_node");
	std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
	ros::NodeHandle nh_p("~") ;
	param_init(nh_p);

	auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(full_path);
	auto chassis_executor = new roborts_decision::ChassisExecutor;
	//auto gimbal_executor = new roborts_decision::GimbalExecutor;
	auto blackboard = new roborts_decision::Blackboard(full_path);
	//auto region = new roborts_decision::regional_function();

	roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
	roborts_decision::CircleBehavior       circle_behavior(chassis_executor, blackboard, full_path);

	auto patrol_action_ = std::make_shared<roborts_decision::PatrolAction>(blackboard_ptr_);
	auto circle_action_ = std::make_shared<roborts_decision::CircleAction>(blackboard_ptr_);

	auto patrol_condition_ = std::make_shared<roborts_decision::PreconditionNode>("patrol condition", blackboard_ptr_, [&]()
	{
		if (blackboard->GetBlood().data >= 1000) return true;
		else return false;
	} , roborts_decision::AbortType::BOTH);

	patrol_condition_->SetChild(patrol_action_);

	auto root_selector_ = std::make_shared<roborts_decision::SelectorNode>("patrol or not",blackboard_ptr_);
	root_selector_->AddChildren(patrol_condition_);
	root_selector_->AddChildren(circle_action_);

	roborts_decision::BehaviorTree root(root_selector_,200);
	root.Run();
	
	return 0;
}
