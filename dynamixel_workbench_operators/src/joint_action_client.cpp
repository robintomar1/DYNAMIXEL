/*
* joint action client node 
* subscriptions-
* subscriber to topic pose_name
* client of dynamixel_workbench_operators/joint_opAction.h
* functionality-
* takes a pose name and send the msg to action server as a goal
*/
/*robin singh tomar*/
//Wed 19 Jun 2019 02:43:54 PM IST 

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <string.h>
#include<dynamixel_workbench_operators/joint_opAction.h>
std::string pose;
bool flag;

void doneCb(const actionlib::SimpleClientGoalState& state, const dynamixel_workbench_operators::joint_opResultConstPtr& result) 
{
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %s; goal_stamp = %s ", result->output.c_str(), result->goal_stamp.c_str());
}

void posecallback(const std_msgs::String::ConstPtr& msg)
{   
	ROS_INFO("inside topic callback");
	ROS_INFO("message received %s",msg->data.c_str());
	pose=msg->data.c_str();
	flag=true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_action_client"); // name this node 
    ros::NodeHandle client_node;
       // the "true" argument says that we want our new client to run as a separate thread (a good idea)
actionlib::SimpleActionClient<dynamixel_workbench_operators::joint_opAction> action_client("pose_action", true);
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
  if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server;
    ros::Subscriber pose_name_topic  = client_node.subscribe("pose_name", 1000, posecallback);
    dynamixel_workbench_operators::joint_opGoal goal;
     
    while ( ros::ok()) {
    	if (flag)
    
     {
	goal.input = pose; // this merely sequentially numbers the goals sent
	action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
	flag=false;
    }
        ros::spinOnce(); //normally, can simply do: ros::spin();  

    }
    return 0;
}

