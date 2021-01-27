/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#define XMIN -24.5
#define XMAX 24.5
#define YMIN -16
#define YMAX 16
#define ZMIN 0.2
#define ZMAX 4.0
using namespace boost;
bool defaultstart=false;
bool defaulttarget=false;
bool odom_received=false;
bool targetpose_received=false;
bool startpose_received=false;
bool odomasstart=false;
bool trajectory_received=false;
geometry_msgs::Pose odometry_information;
geometry_msgs::Pose targetpose, startpose;
ros::Subscriber base_sub,plan_sub, targetpose_sub, startpose_sub, odomasstart_sub;
static const std::string PLANNING_GROUP = "DroneBody";
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::unique_ptr<robot_state::RobotState> start_state1; 
moveit_msgs::RobotTrajectory trajectory2;
std::vector<geometry_msgs::Pose> trajectory;

void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    trajectory.clear();
    for(auto robot_traj: msg->trajectory){
        for(auto point : robot_traj.multi_dof_joint_trajectory.points){
            geometry_msgs::Pose waypoint;
            waypoint.position.x = point.transforms[0].translation.x;
            waypoint.position.y = point.transforms[0].translation.y;
            waypoint.position.z = point.transforms[0].translation.z;

            waypoint.orientation.x = point.transforms[0].rotation.x;
            waypoint.orientation.y = point.transforms[0].rotation.y;
            waypoint.orientation.z = point.transforms[0].rotation.z;
            waypoint.orientation.w = point.transforms[0].rotation.w;

            trajectory.push_back(waypoint);
        }
    }
    trajectory_received = true;
}

void odomasstartCallback(const std_msgs::String::ConstPtr & msg)
{
	std::string text = "token, test   string";
    std::vector<std::string> tok_string;
    char_separator<char> sep(", ");
    tokenizer< char_separator<char> > tokens2(msg->data, sep);
    BOOST_FOREACH (const std::string& t, tokens2) {
	    std::cout << t << "." <<std::endl;
	    tok_string.push_back(t);
	    
    }
    if (tok_string[0]=="1"){
	    odomasstart = true;
    }else
	    odomasstart = false;

    float xmin, ymin, xmax, ymax;
    //more cmds...
    if (tok_string.size()>1){
      if (tok_string[1]=="ws"){
	      std::cout<<"change workspace for planning\n";
	      xmin=std::stof(tok_string[2]);
	      ymin=std::stof(tok_string[3]);
	      xmax=std::stof(tok_string[4]);
	      ymax=std::stof(tok_string[5]);
    		move_group->setWorkspace(xmin,ymin, ZMIN,xmax,ymax,ZMAX);
    	} 
      if (tok_string[1]=="dp"){ //default start/target pose, "dp,1,1"
	      if (tok_string[2]=="1")
		      defaultstart=true;
	      else 
		      defaultstart=false;
	      if (tok_string[3]=="1")
		      defaulttarget=true;
	      else 
		      defaulttarget=false;

      }
    }

    std::cout<<"get odomasstart msg "<<msg->data<<" \n";
}
void startposeCallback(const geometry_msgs::Pose::ConstPtr & msg)
{
    startpose  = *msg;
    startpose_received = true;
    std::cout<<"get a start pose\n";
}
void targetposeCallback(const geometry_msgs::Pose::ConstPtr & msg)
{
    targetpose  = *msg;
    //targetpose  = msg->pose.pose;
    targetpose_received = true;
    std::cout<<"get a target pose\n";
}
void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    odometry_information = msg->pose.pose;
    odom_received = true;
}
void go()
{
	if (!( startpose_received || targetpose_received || odomasstart))
		return;
    static std::vector<double> target(7);
    if (defaulttarget){
    target[0] = -12;
    target[1] = -10;
    target[2] = 1;
    target[3] = 0;
    target[4] = 0;
    target[5] = 0;
    target[6] = 1;
    	defaulttarget=false;
    }
    static std::vector<double> start_state_(7);
    if (defaultstart){
    	start_state_[0] = -12.44;
    	start_state_[1] = -12.44;
    	start_state_[2] = 0.53;
    	start_state_[3] = 0;
    	start_state_[4] = 0;
    	start_state_[5] = 0;
    	start_state_[6] = 1;
    	defaultstart=false;
    }
    if (odomasstart){
    	start_state_[0] = odometry_information.position.x;
    	start_state_[1] = odometry_information.position.y;
    	start_state_[2] = odometry_information.position.z;
    	start_state_[3] = odometry_information.orientation.x;
    	start_state_[4] = odometry_information.orientation.y;
    	start_state_[5] = odometry_information.orientation.z;
    	start_state_[6] = odometry_information.orientation.w;
    }
    if (startpose_received){
    	start_state_[0] = startpose.position.x;
    	start_state_[1] = startpose.position.y;
    	start_state_[2] = startpose.position.z;
    	start_state_[3] = startpose.orientation.x;
    	start_state_[4] = startpose.orientation.y;
    	start_state_[5] = startpose.orientation.z;
    	start_state_[6] = startpose.orientation.w;
    	startpose_received=false;
    }
    if (targetpose_received){
    	target[0] = targetpose.position.x;
    	target[1] = targetpose.position.y;
    	target[2] = targetpose.position.z;
    	target[3] = targetpose.orientation.x;
    	target[4] = targetpose.orientation.y;
    	target[5] = targetpose.orientation.z;
    	target[6] = targetpose.orientation.w;
    	targetpose_received=false;
    }
    move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    start_state1->setVariablePositions(start_state_);
    move_group->setStartState(*start_state1);

    bool collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf]",start_state_[0], start_state_[1], start_state_[2] );
    ROS_INFO("Try to go to [%lf,%lf,%lf]",target[0],target[1],target[2]);

    bool isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //this function will block for sometime, if fail to calculate 
    //it will print a message Fail: ABORTED: No motion plan found. No execution attempted.
    
    moveit_msgs::RobotTrajectory plan_trajectory;
    // if plan succed, move_group will publish the result in topic /move_group/display_planned_path, so the while loop will not block forever.
    if(isPathValid){
   	std::cout<<"good, check /move_group/display_planned_path for detail \n";
	plan_trajectory = plan.trajectory_; //this might not have the actualdata so we have to get it from the /move_group/display_planned_path? tested and seems both methods has the traj data. 
	std::cout<<"plan_trajectory size: "<<plan_trajectory.multi_dof_joint_trajectory.points.size();

	/*
       	while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            ros::Duration(0.2).sleep();
        }
	trajectory_received=false;
	std::cout<<"trajectory from /move_group/display_planned_path"<<std::endl;
	for(int i=0;i<trajectory.size();i++){
		std::cout<<trajectory[i]<<std::endl;
	}
	*/

    }

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

   move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
   plan_sub = node_handle.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,planCallback);

    base_sub = node_handle.subscribe<nav_msgs::Odometry>("/airsim_node/SimpleFlight/odom_local_ned",10,poseCallback);
    startpose_sub = node_handle.subscribe<geometry_msgs::Pose>("/startpose",10,startposeCallback);
    targetpose_sub = node_handle.subscribe<geometry_msgs::Pose>("/targetpose",10,targetposeCallback);
    odomasstart_sub = node_handle.subscribe<std_msgs::String>("/odomasstart",10,odomasstartCallback);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setNumPlanningAttempts(10);
    move_group->setWorkspace(XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX);
start_state1.reset(new robot_state::RobotState(move_group->getRobotModel()));

    std::vector<double> target(7);
    target[0] = -12;
    target[1] = -10;
    target[2] = 1;
    target[3] = 0;
    target[4] = 0;
    target[5] = 0;
    target[6] = 1;
    /*
      x: -23.7370815277
      y: -12.4475603104
      z: -0.535913169384
      */
    std::vector<double> start_state_(7);
    start_state_[0] = -23.73;
    start_state_[1] = -12.44;
    start_state_[2] = 0.53;
    start_state_[3] = 0;
    start_state_[4] = 0;
    start_state_[5] = 0;
    start_state_[6] = 1;

    move_group->setJointValueTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf]",start_state_[0], start_state_[1], start_state_[2] );
    ROS_INFO("Try to go to [%lf,%lf,%lf]",target[0],target[1],target[2]);
    start_state1->setVariablePositions(start_state_);
    move_group->setStartState(*start_state1);

    bool isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(isPathValid){
	    std::cout<<"good\n";
    }
    ros::Rate rate(2);
    while(ros::ok()){
        while(!odom_received && !targetpose_received && !startpose_received)
            rate.sleep();
	go();
        ros::spinOnce();
        rate.sleep();

    }
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group->setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
  /* move_group->move(); */

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group->setJointValueTarget(joint_group_positions);

  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
 /*
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
*/
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group->getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group->setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group->setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group->setPlanningTime(10.0);

  success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // When done with the path constraint be sure to clear it.
  move_group->clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group->setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  //moveit_msgs::RobotTrajectory trajectory;
  //std::vector<geometry_msgs::Pose> trajectory;

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory2);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
