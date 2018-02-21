/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Gesture Processing service source code */

/* Libraries */
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "miro_teleop/GestureProcessing.h"
#include <cmath>

/* Definitions */
#define HSIZE 400
#define VSIZE 400

/* Service function */
bool findTarget(miro_teleop::GestureProcessing::Request  &req,
         	miro_teleop::GestureProcessing::Response &res)
{
	/* Obtain body position */
	geometry_msgs::Point position = req.gesture.position;

	/* Initial orientation reference (assuming alignment with x axis) */
	tf::Vector3 reference(1,0,0);

	/* Convert from quaternion to rotated vector representation */
	tf::Quaternion rotation;
	quaternionMsgToTF(req.gesture.orientation, rotation);
	tf::Vector3 direction = tf::quatRotate(rotation, reference);

	ROS_INFO("Quaternion received: (%3.2f, %3.2f, %3.2f, %3.2f)",
		req.gesture.orientation.x,
		req.gesture.orientation.y,
		req.gesture.orientation.z,
		req.gesture.orientation.w);

	ROS_INFO("Position: (%3.2f, %3.2f, %3.2f)", 
				position.x, position.y, position.z);
	ROS_INFO("Direction: (%3.2f, %3.2f, %3.2f)",
				direction.x(), direction.y(), direction.z());

	/* Check whether user is pointing upwards */
	if(direction.z() > 0)
	{
		ROS_INFO("Invalid gesture");
		// Send a position out of the bounds
		res.target.x = 2*HSIZE;
		res.target.y = 2*VSIZE;
		res.target.theta = 0;
	}
	else
	{
		/* If not, find target position in the x-y plane */

		double a = -(position.z-80)/(direction.z());
		res.target.x = position.x + a*(direction.x());
		res.target.y = position.y + a*(direction.y());
		res.target.theta = atan2(res.target.y-position.y, 
						res.target.x-position.x);
		// Bound conditions are verified by the master
	}
 	
	ROS_INFO("Target: (%f,%f)", res.target.x, res.target.y);
  	
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
