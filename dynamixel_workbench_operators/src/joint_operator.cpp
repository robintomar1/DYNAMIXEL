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
//edited for different number of motors in single file 
//Sat 08 Jun 2019 12:38:58 PM IST 
//from now on any changes made will be recorded

#include "dynamixel_workbench_operators/joint_operator.h"
uint8_t my_joint_size;
#define max_number_of_joints 5 //change this value for more number of joints

JointOperator::JointOperator()
  :node_handle_(""),
  priv_node_handle_("~"),
  is_loop_(false)
{
	  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
	  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory [max_number_of_joints];
	  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);
	  if (result == false)
		  {
			    ROS_ERROR("Please check YAML file");
			    exit(0);
		  }
	 joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1000);
	 move_command_server_ = node_handle_.advertiseService("pose_name",&JointOperator::moveCommandMsgCallback,this);
	}

JointOperator::~JointOperator()
{
}

bool JointOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
	/*file is a maybe pointer variable of type yaml::node*/

	  YAML::Node file;

	  file = YAML::LoadFile(yaml_file.c_str());

	  if (file == NULL)
	    return false;
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

		  uint8_t joint_size = joint["names"].size();
		  printf("joint size %d\n",joint_size);
			/*save each joint name in array of strings of the jnt_tra_msg->joint_names */
		  for (uint8_t index = 0; index < joint_size; index++)
			  {
			    	std::string joint_name = joint["names"][index].as<std::string>();
			    	(jnt_tra_msg+index_robin)->joint_names.push_back(joint["names"][index].as<std::string>());
			  }

		  YAML::Node motion = joint["motion"];
		  uint8_t motion_size = motion["names"].size();

		  for (uint8_t index = 0; index < motion_size; index++)
		 {
			    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

			    std::string name = motion["names"][index].as<std::string>();
			    YAML::Node motion_name = motion[name.c_str()];
			    printf("motion name %s\n",name.c_str());


			    for (uint8_t size = 0; size < joint_size; size++)
			    {
					printf("motion step size %d\n",motion_name["step"].size());
				      if (joint_size != motion_name["step"].size())
				      {
						ROS_ERROR("Please check motion step size. It must be equal to joint size");
						return 0;
				      }

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
 return true;
}

int main(int argc, char **argv)
{
	// Init ROS node
  ros::init(argc, argv, "multi_joint_operator");
  JointOperator joint_operator;

  ROS_INFO("For now, you can use publish joint trajectory msgs by triggering service(/execution)");

  ros::spin();

  return 0;
}
