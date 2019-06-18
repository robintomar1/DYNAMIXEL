/* this node is used to get the name of pose from outside and call the service to execute that pose */
/*created by Robin tomar */
/*Fri 07 Jun 2019 11:07:02 AM IST */
//  <node name="service_caller" pkg="dynamixel_workbench_operators" type="service_caller"/>

#include <ros/ros.h>
#include <dynamixel_workbench_operators/pose_name.h> 
#include "std_msgs/String.h"

/*request and response objects for calling the service */
dynamixel_workbench_operators::pose_name::Request request_object;
dynamixel_workbench_operators::pose_name::Response response_object;

/*client for service pose_client*/
	ros::ServiceClient pose_Client= service_node.serviceClient<dynamixel_workbench_operators::pose_name>("pose_name");

/*subscriber callback function*/
bool posecallback(const std_msgs::String::ConstPtr& msg)
{ 

	std::string name=msg->data;
	
	if (name.compare("one")==0)
	    {
	    	request_object.pose_name="one";	
	    	/*call the service with argument : "one"*/
	    	bool success = pose_Client.call( request_object, response_object ) ;
	    	// Check for success and use the response .
		i f ( success ) 
		{
			ROS_INFO_STREAM( " service executed : "<< response_object.result ) ;
			} else {
			ROS_ERROR_STREAM( "  service could not be executed  " ) ;
		}
	    }
	else if (name.compare("two")==0)
	    {
		request_object.pose_name="two";
	    	/*call the service with argument : "two"*/		
	    	bool success = pose_Client.call( request_object, response_object ) ;
	   	// Check for success and use the response .
		i f ( success ) 
		{
			ROS_INFO_STREAM( " service executed : "<< response_object.result ) ;
			} else {
			ROS_ERROR_STREAM( "  service could not be executed  " ) ;
		}

	    }
	else if (name.compare("three")==0)
	    {
	    	request_object.pose_name="three";
	    	/*call the service with argument : "three"*/
	    	bool success = pose_Client.call( request_object, response_object ) ;
	    	// Check for success and use the response .
		i f ( success ) 
		{
			ROS_INFO_STREAM( " service executed : "<< response_object.result ) ;
			} else {
			ROS_ERROR_STREAM( "  service could not be executed  " ) ;
		}

	    }
}


 int main(int argc, char **argv)
 {
 	/*service caller is the name of the node */
 	  ros::init(argc, argv, "service_caller");


//pose_service = node_handle_.advertiseService<dynamixel_workbench_operators::pose_name>("pose_name", posecallback);

/*node handle for current node*/
	ros::NodeHandle service_node;


/*subscriber to the pose_name_topic topic*/
	ros::Subscriber pose_name_topic=service_node.subscribe("pose_name_topic", 100, posecallback);
}


