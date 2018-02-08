/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Gesture Processing service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/GestureProcessing.h"
#include <cmath>

/* Service function */
bool findTarget(miro_teleop::GestureProcessing::Request  &req,
         	miro_teleop::GestureProcessing::Response &res)
{
	/* Check whether user is pointing upwards */
	if(req.gesture.orientation.z > 0)
	{
		// TODO Invalid gesture, do something... ****
	}
	else
	{
		/* If not, find target position in the x-y plane */
		double a = -req.gesture.position.z/req.gesture.orientation.z;
		res.target.x = req.gesture.position.x + 
						 a*req.gesture.orientation.x;
		res.target.y = req.gesture.position.y + 
						 a*req.gesture.orientation.y;
		res.target.theta = atan2(res.target.y-req.gesture.position.y, 
					res.target.x-req.gesture.position.x);
	
		// TODO Include boundaries conditions... ****
	}
 	
	ROS_INFO("Target found: (%f,%f)", res.target.x, res.target.y);
  	
	return true;
}

/* Main function */
int main(int argc, char **argv)
{
        /* Initialize, assign a node handler and advertise service */
	ros::init(argc, argv, "gesture_processing_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
			n.advertiseService("gesture_processing", findTarget);
	ROS_INFO("Gesture processing service active");
	ros::spin();

	return 0;
}
