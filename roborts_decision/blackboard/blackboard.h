/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "std_msgs/Int64.h"
#include "roborts_msgs/GimbalAngle.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

namespace roborts_decision{

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true){
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    if (!decision_config.simulate()){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }
    //以下是自定义数据的初始化
    //laser_sub_ =nh.subscribe<sensor_msgs::LaserScan>("scan",1,&Blackboard::LaserCallback, this);
    //blood_sub_ =nh.subscribe<std_msgs::Int64>("blood", 10, &Blackboard::BloodCallback, this);
   gimbalangle_sub_=nh.subscribe<roborts_msgs::GimbalAngle>("G_angle", 10, &Blackboard::GangleCallback, this);
	aim_position_sub_=nh.subscribe<geometry_msgs::PoseStamped>("aim_position",10,&Blackboard::AimPositionCallback,this);
    gimbal_angle_sub_=nh.subscribe<std_msgs::Int16>("gimbal_gyro",10,&Blackboard::GimbalAngleCallback, this);//云台对地角度
	if_shoot_sub_=nh.subscribe<std_msgs::Bool>("if_shoot",10,&Blackboard::IfShootCallback,this);
	if_enemy_sub_=nh.subscribe<std_msgs::Bool>("if_enemy",10,&Blackboard::IfEnemyCallback,this);
//odom
    MoveType_=0;
  }

  ~Blackboard() = default;


  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected){
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;

      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
          camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);

      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        if(GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2){
          enemy_pose_ = global_pose_msg;

        }
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    } else{
      enemy_detected_ = false;
    }

  }

  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

  //以下是自定义的函数
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   scan_=*scan;
  }

  void BloodCallback(const std_msgs::Int64::ConstPtr& blood){
   blood_=*blood;
  }

  sensor_msgs::LaserScan GetLaser() const{
    return scan_;
  }

  std_msgs::Int64 GetBlood() const{
   return blood_;
  }

  void MoveTypeCallback(const int bg){
   MoveType_=bg;
  }

  int GetMoveType() const{
   return MoveType_;
  }

  void GangleCallback(const roborts_msgs::GimbalAngle::ConstPtr& gimbal_angle){
   gimbalangle_=*gimbal_angle;
  }

  roborts_msgs::GimbalAngle GetGangle() const{
    return gimbalangle_;
  }

  void GimbalAngleCallback(const std_msgs::Int16::ConstPtr& gimbal_angle){

   yaw_angle_ = gimbal_angle->data/1800.0*3.14159265;
  }

  double GetGimbalAngle() const{
   return yaw_angle_;
}

	void AimPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& aim_position){
	aim_position_=*aim_position;
	}

	geometry_msgs::PoseStamped GetAimPosition() const{
	return aim_position_;
	}

	void IfShootCallback(const std_msgs::Bool::ConstPtr& if_shoot){
	if_shoot_=*if_shoot;
	}

	std_msgs::Bool GetIfShoot() const{
	return if_shoot_;
	}

	void IfEnemyCallback(const std_msgs::Bool::ConstPtr& if_enemy){
	if_enemy_=*if_enemy;
	}

	std_msgs::Bool GetIfEnemy() const{
	return if_enemy_;
	}
 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  //! 以下是自定义的数据
  sensor_msgs::LaserScan scan_;
  ros::Subscriber laser_sub_;
  std_msgs::Int64 blood_;
  ros::Subscriber blood_sub_;
  roborts_msgs::GimbalAngle gimbalangle_;
  ros::Subscriber gimbalangle_sub_;

  double yaw_angle_;//这个数据记录了云台的角位置, 对地面参考
  ros::Subscriber gimbal_angle_sub_;
  int MoveType_;//标记上一个行动状态，0表示没有行动
	geometry_msgs::PoseStamped aim_position_;//接受传输的信息
	ros::Subscriber aim_position_sub_;
	std_msgs::Bool if_shoot_;
	ros::Subscriber if_shoot_sub_;
	std_msgs::Bool if_enemy_;
	ros::Subscriber if_enemy_sub_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
