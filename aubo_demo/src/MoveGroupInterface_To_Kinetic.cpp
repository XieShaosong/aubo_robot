/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 *
 *  Author: zhaoyu
 *  email : zhaoyu@aubo-robotics.cn
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <industrial_msgs/RobotStatus.h>
#include <aubo_msgs/JointMsg.h>
#include <aubo_msgs/Vector3.h>
#include <aubo_msgs/WayPoint.h>
#include "std_msgs/String.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;

void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  ROS_INFO("protective stop flag: %d", msg->in_error.val);
}

void jointMsgCB(const aubo_msgs::JointMsg::ConstPtr &msg)
{
  ROS_INFO("actual_current: %d", msg->actual_current[1]);
}

void wayPointCB(const aubo_msgs::WayPoint::ConstPtr &msg)
{
  ROS_INFO("actual_rotation.x: %f", msg->actual_rotation.x);
}

bool moveitPlan(std::vector<double> group_variable_values, const char* mode, double velo_multiplier, double eef_step,
                geometry_msgs::Pose* target_pose, std::vector<double>* target_joints, std::vector<geometry_msgs::Pose>* waypoints,
                trajectory_msgs::JointTrajectory& trajectory)
{
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;
  std::string plan_mode = mode;

  // set start state
  robot_state::RobotState start_state(*group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group.getName());
  group.getCurrentState()->copyJointGroupPositions(joint_model_group, group_variable_values);

  start_state.setJointGroupPositions(joint_model_group, group_variable_values);
  group.setStartState(start_state);

  // if waypoints have only one points, then simply usee pose mode
  if (plan_mode == "waypoints" && waypoints->size() < 2)
  {
    plan_mode = "pose";
    target_pose = &(waypoints->at(0));
  }

  // set target pose,there are two types of target: pos or joint
  if (plan_mode == "joint")
  {
    group.setJointValueTarget(*target_joints);
  }
  else if (plan_mode == "pose")
  {
    group.setPoseTarget(*target_pose);
  }
  else if (plan_mode == "waypoints")
  {

  }
  else
    ROS_ERROR("error, don't have such mode: %s", mode);

  group.setPlannerId("RRTConnectkConfigDefault");

  if (plan_mode == "joint"||plan_mode == "pose")
  {
    group.setPlanningTime(1);

    success = (group.plan(plan)  == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
      trajectory = plan.trajectory_.joint_trajectory;
  }
  else if (plan_mode == "waypoints")
  {
    double fraction = 0;
    moveit_msgs::RobotTrajectory moveit_trajectory;
    fraction = group.computeCartesianPath(*waypoints,
                                          eef_step, // eef_step
                                          0.0,      // jump_threshold
                                          moveit_trajectory);

    trajectory = moveit_trajectory.joint_trajectory;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveGroupInterface_To_Kinetic");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh;
  
  ros::Publisher robot_control_pub_ = nh.advertise<std_msgs::String>("robot_control" ,100);
  // poweOn
  // sleep(1);
  // std_msgs::String cmd;
  // cmd.data = "powerOn";
  // robot_control_pub_.publish(cmd);

  ActionClient* action_client = new ActionClient(nh, "aubo_i3_controller/follow_joint_trajectory", true);

  ros::Subscriber sub_robot_status_ = nh.subscribe("robot_status", 1, robotStatusCB);
  ros::Subscriber sub_joint_msg_ = nh.subscribe("/aubo_driver/joint_msgs", 1, jointMsgCB);
  ros::Subscriber sub_wayPoint_ = nh.subscribe("/aubo_driver/waypoint", 1, wayPointCB);

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  control_msgs::FollowJointTrajectoryGoal joint_trajectory_goal;
  
  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //joint Position
  std::vector<double> home_position;
  home_position.push_back(-0.434921);
  home_position.push_back(-1.253805);
  home_position.push_back(0.923242);
  home_position.push_back(0.037245);
  home_position.push_back(-1.418411);
  home_position.push_back(0.043521); 

  // move home
  move_group.setJointValueTarget(home_position);
  move_group.move();
  
  sleep(2);

  // target_position
  std::vector<double> target_position;
  target_position.push_back(-0.652182);
  target_position.push_back(-1.518637);
  target_position.push_back(-0.465616);
  target_position.push_back(-0.718959);
  target_position.push_back(-1.553274);
  target_position.push_back(-0.096143); 
 
  bool ret = moveitPlan(home_position, "joint", 0.08, 0.01, 0, &target_position, 0, joint_trajectory_goal.trajectory);

  action_client->waitForServer();
  action_client->sendGoal(joint_trajectory_goal);
  int finished = action_client->waitForResult();

  sleep(2);

  // target pose
  tf::Quaternion q;
  q.setRPY(-0.201168, -0.002089, -2.320873);
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.409621;
  target_pose1.position.y = -0.464676;
  target_pose1.position.z = 0.079343;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  ret = moveitPlan(target_position, "pose", 0.03, 0.01, &target_pose1, 0, 0, joint_trajectory_goal.trajectory);

  action_client->waitForServer();
  action_client->sendGoal(joint_trajectory_goal);
  finished = action_client->waitForResult();
  
  sleep(2);

  // Move to the target position
  move_group.setJointValueTarget(target_position);
  move_group.move();

  sleep(2);

  //waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);
  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose2.position.z -= 0.2;
  waypoints.push_back(target_pose2);
  target_pose2.position.y -= 0.15;
  waypoints.push_back(target_pose2);
  target_pose2.position.z += 0.2;
  target_pose2.position.y += 0.2;
  target_pose2.position.x -= 0.2;
  waypoints.push_back(target_pose2);

  ret = moveitPlan(target_position, "waypoints", 0.5, 0.01, 0, 0, &waypoints, joint_trajectory_goal.trajectory);

  action_client->waitForServer();
  action_client->sendGoal(joint_trajectory_goal);
  finished = action_client->waitForResult();

  sleep(2);

  move_group.setJointValueTarget(home_position);
  move_group.move();

  spinner.stop();

  ros::spin();
  return 0;
}
