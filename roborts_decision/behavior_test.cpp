#include <ros/ros.h>
#include<iostream>
#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/gogoal_behavior.h"
#include "example_behavior/circle_behavior.h"
#include "example_behavior/wait_behavior.h"
#include "example_behavior/gimbalget0_behavior.h"
#include "example_behavior/shake_behavior.h"
#include "example_behavior/dance_behavior.h"
#include "example_behavior/aim_behavior.h"
#include "example_behavior/gimbal_aim_behavior.h"


void Command();
char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
  roborts_decision::Gogoal               gogoal(chassis_executor, blackboard, full_path);
  roborts_decision::CircleBehavior     circle_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::WaitBehavior     wait_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GimbalGet0   gimbal_get0(gimbal_executor, blackboard, full_path);
  roborts_decision::ShakeBehavior       shake_behavior(gimbal_executor, blackboard, full_path);
  roborts_decision::DanceBehavior        dance_behavior(chassis_executor, gimbal_executor,blackboard, full_path);
  roborts_decision::AimBehavior            aim_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GimbalAimBehavior    gimbal_aim_behavior(gimbal_executor, blackboard, full_path);

  auto command_thread= std::thread(Command);
//Command();
//std::cin>>command;
  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      //back to boot area
      case '1':
        back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':
        patrol_behavior.Run();
        break;
        //chase.
      case '3':
        chase_behavior.Run();
        break;
        //search
      case '4':
        search_behavior.Run();
        break;
        //escape.
      case '5':
        escape_behavior.Run();
        break;
        //goal.
      case '6':
        goal_behavior.Run();
        break;
      case 'g':
        gogoal.Run();
        break;
      case '8':
	circle_behavior.Run();
	break;
      case '9':
	if (blackboard->GetBlood().data >= 1000)
		patrol_behavior.Run();
	else
		circle_behavior.Run();
	break;
      case 'q':
	wait_behavior.Run();
	break;
	case 's':
	gimbal_get0.Run();
	break;
	case 'u':
	shake_behavior.Run();
	break;
	case 'r':
	dance_behavior.Run();
	break;
   case 'a':
     aim_behavior.Run();
     break;
     case 'v':
     gimbal_aim_behavior.Run();
     break;
      case 27:
        //if (command_thread.joinable()){
          //command_thread.join();
        //}
        return 0;
      default:
        break;
    }
    rate.sleep();
  }


  return 0;
}

void Command() {
  std::cout<<"wtf\n";
  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
	      << "g: go to one goal" << std::endl
	      << "8: circle behavior"<< std::endl
	      << "9: behavior_tree test"<< std::endl
	      << "q: wait"<< std::endl
	      <<"s: stop the gimbal into 0"<<std::endl
	      <<"u:shake behavior"<<std::endl
	      <<"r:dance"<<std::endl
        <<"a:go into aim_received"<<std::endl
        <<"v:gimbal go into aim_received"<<std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 'g' && command != '8' && command != '9' &&command != 'q' &&command!='s'&&command!='u'&&command!='r' &&command!='a'&&command!='v'&&command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

