#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <iostream>

geometry_msgs::Point p1, p0;
geometry_msgs::PoseStamped stablePose;
	
// Frequency (Motion capture is 120Hz) - Changes detection
int frequency = 30;
// Under this reference speed the pointing is considered admissible
float v_ref = 0.025;	
// Distance theshold calculated in relation to the reference speed and the rate(frequency) of the node
double d_treshold = v_ref * (1.0 /(double) frequency);

// Flag used to decide when the node can publish messages
bool pub_flag = true;

// Publisher declaration on the topic
ros::Publisher pub, blah;
	
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
  			blah.publish(p0);
			// Distance calculated between two subsequent hand's points
			double distance = sqrt( pow((double)p0.x-(double)msg->pose.position.x,2)+
									pow((double)p0.y-(double)msg->pose.position.y,2)+
									pow((double)p0.z-(double)msg->pose.position.z,2));
			// Setting pub_flag true when the user begin to move his right arm with a certain speed (3 times the distance trashold)
			if(!pub_flag && distance > 3*d_treshold){
				pub_flag = true;
			}
			
			ROS_INFO("Distance: %f",distance);
		
			// Under this threshold the arm is considered stationary and the message could be published
			if(distance < d_treshold && pub_flag){							
				stablePose = *msg;
				// Publishing of a Joints msg 
				ROS_INFO("Stable pointing found");
				pub.publish(stablePose);			
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
	// For debuggine purposes
	blah = node.advertise<geometry_msgs::Point>("p0", 5);
	
	
	pub = node.advertise<geometry_msgs::PoseStamped>("stable_pose", 5);
	
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
