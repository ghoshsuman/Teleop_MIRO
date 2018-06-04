#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <iostream>
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#define H 0 // Relative height with respect to ground
#define HSIZE 300 // Horizontal map size (in cm)
#define VSIZE 300 // Vertical map size (in cm)


geometry_msgs::Point p1, p0;
geometry_msgs::PoseStamped stablePose;
geometry_msgs::PointStamped target;
	
// Frequency (Motion capture is 120Hz) - Changes detection
int frequency = 30;
// Under this reference speed the pointing is considered admissible
float v_ref = 0.025;	
// Distance theshold calculated in relation to the reference speed and the rate(frequency) of the node
double d_treshold = v_ref * (1.0 /(double) frequency);

// Flag used to decide when the node can publish messages
bool pub_flag = false;

// Publisher declaration on the topic
ros::Publisher pub;
	
void poseCallback(const geometry_msgs::PoseStampedPtr& msg){
  
  	if(p0.z!=p1.z) // If the position is not the 'default' one
  	{	
  		// If any coordinate is strictly the same, object is out of range
  		if(msg->pose.position.x==p0.x) 
  		{
  			p0 = p1; // Reset position
  		}
  		else
  		{
			// Distance calculated between two subsequent hand's points
			double distance = sqrt( pow((double)p0.x-(double)msg->pose.position.x,2)+
									pow((double)p0.y-(double)msg->pose.position.y,2)+
									pow((double)p0.z-(double)msg->pose.position.z,2));
			// Setting pub_flag true when the user begin to move his right arm with a certain speed (3 times the distance trashold)
			if(!pub_flag && distance > 3*d_treshold){
				pub_flag = true;
			}
		
			// Under this threshold the arm is considered stationary and the message could be published
			if(distance < d_treshold && pub_flag)
			{							
				stablePose = *msg;

				/* Obtain body position */
				geometry_msgs::Point position;
				position.x = 100*stablePose.pose.position.x;
				position.y = 100*stablePose.pose.position.y;

				/* Initial orientation reference (assuming alignment with x axis) */
				tf::Vector3 reference(1,0,0);

				/* Convert from quaternion to rotated vector representation */
				tf::Quaternion rotation;
				quaternionMsgToTF(stablePose.pose.orientation, rotation);
				tf::Vector3 direction = tf::quatRotate(rotation, reference);

				ROS_INFO("Quaternion received: (%3.2f, %3.2f, %3.2f, %3.2f)",
					stablePose.pose.orientation.x,
					stablePose.pose.orientation.y,
					stablePose.pose.orientation.z,
					stablePose.pose.orientation.w);

				ROS_INFO("Position: (%3.2f, %3.2f, %3.2f)", 
							position.x, position.y, position.z);
				ROS_INFO("Direction: (%3.2f, %3.2f, %3.2f)",
							direction.x(), direction.y(), direction.z());

				/* Check whether user is pointing upwards */
				if(direction.z() > 0)
				{
					ROS_INFO("Invalid gesture");
				}
				else
				{
					/* If not, find target position in the x-y plane */

					double a = -(position.z-H)/(direction.z());
					target.point.x = position.x + a*(direction.x());
					target.point.y = position.y + a*(direction.y());


					if(std::isfinite(target.point.x) && std::isfinite(target.point.y) && 
							target.point.x > -HSIZE/2 && target.point.x < HSIZE/2 && 
							target.point.y > -VSIZE/2 && target.point.y < VSIZE/2)
					{
					
						// Publishing of a Joints msg 
						ROS_INFO("Stable pointing found at (%f,%f)",target.point.x,target.point.y);
						target.header.stamp = ros::Time::now();
						pub.publish(target);			
					}
				}
				pub_flag = false;
			}
  		}
  	}
  	p0 = msg->pose.position; 
  		
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "tracker_node");
	ros::NodeHandle node;
	// Initial point position to be set when it is undefined
	p1.x = 0;
	p1.y = 0;
	p1.z = 200;
	p0 = p1;
	
	pub = node.advertise<geometry_msgs::PointStamped>("stable_gndpose", 5);
	
	// Rate
	ros::Rate r(frequency);
	
	// Subscriber declaration
	ros::Subscriber sub 
		= node.subscribe("Gesture/pose", 10, &poseCallback);
	
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
