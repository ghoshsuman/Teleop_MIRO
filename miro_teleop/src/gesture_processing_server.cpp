#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "miro_teleop/GestureProcessing.h"

bool findTarget(miro_teleop::GestureProcessing::Request  &req,
         	miro_teleop::GestureProcessing::Response &res)
{
	if(req.gesture.orientation.z > 0)
	{
		// Invalid gesture, do something... ****
	}
	else
	{
		double a = -req.gesture.position.z/req.gesture.orientation.z;
		res.target.x = req.gesture.position.x + 
						 a*req.gesture.orientation.x;
		res.target.y = req.gesture.position.y + 
						 a*req.gesture.orientation.y;
		res.target.theta = atan2(res.target.y-req.gesture.position.y, 
					res.target.x-req.gesture.position.x);
	
		// Include boundaries conditions... ****
	}
 	ROS_INFO("Target found: (%f,%f,%f)", 
			       res.target.x, res.target.y, res.target.theta);
  	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gesture_processing_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
			n.advertiseService("gesture_processing", findTarget);
	ROS_INFO("Gesture processing service active");
	ros::spin();

	return 0;
}
