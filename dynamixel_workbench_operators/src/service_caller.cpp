/* this node is used to get the name of pose from outside and call the service to execute that pose */
/*created by Robin tomar */
/*Fri 07 Jun 2019 11:07:02 AM IST */


#include <ros/ros.h>
#include <dynamixel_workbench_operators/pose_name.h> 
#include "std_msgs/String.h"
#include <string.h>

/*request and response objects for calling the service */
dynamixel_workbench_operators::pose_name::Request request_object;
dynamixel_workbench_operators::pose_name::Response response_object;



/*client for service pose_client*/
	ros::ServiceClient pose_Client;

/*subscriber callback function*/
void posecallback(const std_msgs::String::ConstPtr& msg)
{ 

	//string name=msg->data;
	ROS_INFO("inside topic callback");
	
	ROS_INFO("%s",msg->data.c_str());
	if (strcmp((msg->data.c_str()),"one")==0)
	    { 
	    	ROS_INFO("inside topic callback inside one");
	    	request_object.pose_name="one";	
	    	/*call the service with argument : "one"*/
	        ROS_INFO("pose name sent to service %s",request_object.pose_name.c_str());	
	        
	    	bool success = pose_Client.call( request_object, response_object ) ;
	    	//ROS_INFO(sucess);
	    	// Check for success and use the response .
		if ( success ) 
		{
			ROS_INFO( " service executed : ") ;
			} else {
			ROS_INFO( "  service could not be executed  " ) ;
		}
	    }
	else if (strcmp((msg->data.c_str()),"two")==0)
	    { 
	    	ROS_INFO("inside topic callback inside two");
		request_object.pose_name="two";
	    	/*call the service with argument : "two"*/		
	    	bool success = pose_Client.call( request_object, response_object ) ;
	   	// Check for success and use the response .
		if ( success ) 
		{
			ROS_INFO( " service executed ") ;
			} else {
			ROS_INFO( "  service could not be executed  " ) ;
		}

	    }
	else if (strcmp((msg->data.c_str()),"three")==0)
	    {
	    	ROS_INFO("inside topic callback inside three");
	    	request_object.pose_name="three";
	    	/*call the service with argument : "three"*/
	    	bool success = pose_Client.call( request_object, response_object ) ;
	    	// Check for success and use the response .
		if ( success ) 
		{
			ROS_INFO( " service executed " ) ;
			} else {
			ROS_INFO( "  service could not be executed  " ) ;
		}

	    }
}


 int main(int argc, char **argv)
 {	
 /*service caller is the name of the node */
 	ros::init(argc, argv, "service_caller");
/*node handle for current node*/
	ros::NodeHandle service_node;
	pose_Client = service_node.serviceClient<dynamixel_workbench_operators::pose_name>("/dynamixel_workbench/pose_name");
/*subscriber to the pose_name_topic topic*/
	ros::Subscriber pose_name_topic  = service_node.subscribe("pose_name_topic", 1000, posecallback);
/*have to study about rate*/
	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;

}


