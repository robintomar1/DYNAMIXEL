/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/



/* Authors: Taehun Lim (Darby) */
/*comments and edited by Robin Singh Tomar */
//Fri 07 Jun 2019 07:19:33 AM IST 
//edited
//Sat 08 Jun 2019 12:09:58 PM IST 

#include "dynamixel_workbench_operators/joint_operator.h"

#define max_number_of_joints 5 //change this value for more number of joints
	


	/*global variable to store no of joints*/
  uint8_t my_joint_size;

JointOperator::JointOperator()
  :node_handle_(""),
  priv_node_handle_("~"),
  is_loop_(false)
{

  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");

	/*array of trajectory msgs*/
	  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory [max_number_of_joints];

	/* 
	*	jnt_tra_msg is a reference type message variable that stores the message of type joint trajectory
	* 
	* 	trajectory_msgs::JointTrajectory   
	*	
	*	Header header
	*	string[] joint_names
	*	JointTrajectoryPoint[] points
	*/

	/* now the reference type vriable is passed to function getTrajectoryInfo(); */

	/*edited argument from jnt_tra_msg_ to *jnt_tra_msg_ */

  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);



	/*if any error occured while reading values from the yaml file*/
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    exit(0);
  }
	/* joint_trajectory_pub_ is a publisher that publishes on the joint trajectory topic*/
 
  joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1000);
	
	/* 
	*	move_command_server_ is a service provider whenever called it calls the publisher 
	*	joint_trajectory_pub_ which publishes the jnt_tra_msg_ of type 
	*	trajectory_msgs::JointTrajectory on the topic Joint_trajectory
	*/

//  move_command_server_ = node_handle_.advertiseService<dynamixel_workbench_operators::pose_name>("execution",  &JointOperator::moveCommandMsgCallback, this);
 // is_loop_ = priv_node_handle_.param<bool>("is_loop", "false");
 move_command_server_ = node_handle_.advertiseService("pose_name",&JointOperator::moveCommandMsgCallback,this);
}

JointOperator::~JointOperator()
{
}

	/*move_command_server_
	* bool JointOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
	* 
	*	Input : 
	*		a yaml file save its name to a string type variable 
	*		a reference type variable of type trajectory_msgs::JointTrajectory
	*	Output:
	*		boolean value true if no error occurs and false if any error occurs like file not found or anything
	*	
	*	Functionality:
	*		the function takes the joint names and motion values from the passed yaml file
	*	 and save those values in the reference type variabe passed	--jnt_tra_msg---
	*/
	/*edited argument *jnt_tra_msg to **jnt_tra_msg  * but didnt worked so changed back*/

bool JointOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
	/*file is a maybe pointer variable of type yaml::node*/

  YAML::Node file;


	/*	.c_str()
	*	
	*	Get C string equivalent
	*	Returns a pointer to an array that contains a null-terminated sequence
	*	of characters (i.e., a C-string) representing the current value of the string object.
	*	This array includes the same sequence of characters that make up the value of the string
	*	object plus an additional terminating null-character ('\0') at the end.
	*/

	/*
	*	YAML::LoadFile() takes the file name/address and opens it and assigns the address to the variable on the left
	*	file here becomes the root node of the whole yaml file all other infor mation will be acessed through this pointer
	*/

  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

	/*
	*	what we are doing here is that we created a new node in the tree or map and assigned it the address of node "joint"
	*	this is a really nice feature the syntex below works on operator overloading and automatically finds the node whose 
	*	name is given as an index value
	*/
	
	/*edited code start*/
	/*created a parent node in the tree*/
  YAML::Node myjoints = file["myjoints"];

	/*getting the numbers of joints from the names array*/
  my_joint_size = myjoints["names"].size();
  ROS_INFO("my_joint_size %d",my_joint_size);

	/*repeat for each joint*/
  for (uint8_t index_robin = 0; index_robin < my_joint_size; index_robin++)
  {
	/*get name of each joint*/
  std::string joint_name = myjoints["names"][index_robin].as<std::string>();
  ROS_INFO("joint name %s",joint_name.c_str());

	/*create a node in the tree with the joint name as child of parent node myjoints*/
  YAML::Node joint = myjoints[joint_name];
	/*
	*	the search operation is performed in the tree/map below the node[" "]. this  works in hierarchy.
	*	we can search any node under current node just by writing ---parent_node["node_to_be_found"]---
	*/
	/* 	find the number of joints in the file*/

  uint8_t joint_size = joint["names"].size();
  printf("joint size %d\n",joint_size);
	/*save each joint name in array of strings of the jnt_tra_msg->joint_names */
  for (uint8_t index = 0; index < joint_size; index++)
  {
	  
	/*	as()
	*
	*	Force An Object To Belong To A Class

	*	the line below finds the value of index in names array under joint node then force it to be string 
	*	and then assign it ti the string type variable joint_name
	*/
    std::string joint_name = joint["names"][index].as<std::string>();

	/* push_back() function
	*	Vectors are same as dynamic arrays with the ability to resize itself 
	*	automatically when an element is inserted or deleted, with their storage 
	*	being handled automatically by the container.
	*
	*	vector::push_back()
	*	push_back() function is used to push elements into a vector from the back. 
	*	The new value is inserted into the vector at the end, after the current 
	*	last element and the container size is increased by 1.
	*
	*/
    (jnt_tra_msg+index_robin)->joint_names.push_back(joint["names"][index].as<std::string>());
  }
	/*
	*	now we are done with the joint name part of the ---jnt_tra_msg---
	*	now we will find the second parameter of jnt_tra_msg
	*/

  YAML::Node motion = joints["motion"];

	/*find the number of motions */

  uint8_t motion_size = motion["names"].size();
	
	/*
	*	for each motion
	*/

  for (uint8_t index = 0; index < motion_size; index++)
  {
	/*
	*	generate trajectory points and each point is saved to the
	*	jointTrajectoryPoints array of jnt_tra_msg
	*/
	
	/*  trajectory_msgs::JointTrajectoryPoint
	*
	*	# Each trajectory point specifies either positions[, velocities[, accelerations]]
	*	# or positions[, effort] for the trajectory to be executed.
	*	# All specified values are in the same order as the joint names in JointTrajectory.msg
	*
	*	float64[] positions
	*	float64[] velocities
	*	float64[] accelerations
	*	float64[] effort
	*	duration time_from_start
	*/


    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    std::string name = motion["names"][index].as<std::string>();
    YAML::Node motion_name = motion[name.c_str()];
    printf("motion name %s\n",name.c_str());


    for (uint8_t size = 0; size < joint_size; size++)
    {
	/*	
	*	each motion contains step values of each joint 
	*	due to which the size of step array of each motion
	*	contains values for each joints and joint name array
	*	has the same size as that of step array
	*	
	*	so if they are not equal its an error
	*/
	printf("motion step size %d\n",motion_name["step"].size());
      if (joint_size != motion_name["step"].size())
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }
	/*
	*	push the step size to the position portion of the trajectory point
	*	as a double value
	*/

      jnt_tra_point.positions.push_back(motion_name["step"][size].as<double>());
	
      ROS_INFO("motion_name : %s, step : %f", name.c_str(), motion_name["step"][size].as<double>());
      ROS_INFO("generating trajectory points\n");
    }

    if (motion_name["time_from_start"] == NULL)
    {
      ROS_ERROR("Please check time_from_start. It must be set time_from_start each step");
      return 0;
    }

    jnt_tra_point.time_from_start.fromSec(motion_name["time_from_start"].as<double>());

    ROS_INFO("time_from_start : %f", motion_name["time_from_start"].as<double>());
	
	/*  example case:
	*	[ INFO] [1559347822.221009792]: motion_name : right, step : -3.140000
	*	[ INFO] [1559347822.221056854]: motion_name : right, step : -3.140000
	*	[ INFO] [1559347822.221076416]: time_from_start : 2.000000
	*	[ INFO] [1559347822.221098780]: motion_name : zero, step : 0.000000
	*	[ INFO] [1559347822.221114778]: motion_name : zero, step : 0.000000
	*	[ INFO] [1559347822.221130406]: time_from_start : 3.000000
	*/


    (jnt_tra_msg+index_robin)->points.push_back(jnt_tra_point);
	ROS_INFO("value inserted to array\n");
  }
}
  return true;
}


bool JointOperator::moveCommandMsgCallback(dynamixel_workbench_operators::pose_name::Request &req,
                                           dynamixel_workbench_operators::pose_name::Response &res)
{	ROS_INFO("*************inside service call back **********");
	if (0==req.pose_name.compare("one"))
	{
			ROS_INFO("inside service call back with pose one");
			joint_trajectory_pub_.publish(*(jnt_tra_msg_+0));  
			res.result = true;
	}
	else if (0==req.pose_name.compare("two"))
	{
			ROS_INFO("inside service call back with pose two");
			joint_trajectory_pub_.publish(*(jnt_tra_msg_+1));  
			res.result = true;
	}
 	else if (0==req.pose_name.compare("three"))
	{
			ROS_INFO("inside service call back with pose three");
			joint_trajectory_pub_.publish(*(jnt_tra_msg_+2));  
			res.result = true;
	}
	else
	{
			ROS_INFO("could not publish service");
			res.result = false;
	}
 //res.message = "Success to publish joint trajectory"; 
 return true;
}

int main(int argc, char **argv)
{
	// Init ROS node
  ros::init(argc, argv, "joint_operator");
  JointOperator joint_operator;

  ROS_INFO("For now, you can use publish joint trajectory msgs by triggering service(/execution)");

  ros::spin();

  return 0;
}
