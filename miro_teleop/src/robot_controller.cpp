/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Robot Controller node source code */

/* Libraries */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>

/* Global variables */
bool enable = false;  // Controller status flag
std::vector<geometry_msgs::Vector3> path; // Trajectory array
geometry_msgs::Pose2D robot; // Robot position

/* Subscriber callback functions */
void getPoint(const geometry_msgs::Vector3::ConstPtr& point)
{
	/* Append every path position received to current reference path */
	path.push_back(*point);
}
void getStatus(const std_msgs::Bool::ConstPtr& status)
{
	/* Obtain enable/disable flag from master */
	enable = status->data;
	if(enable) ROS_INFO("Controller enabled");
	else ROS_INFO("Controller disabled");
}
void getRobotPos(const geometry_msgs::Pose2D::ConstPtr& pos)
{
	/* Obtain current robot position from motion capture */
	robot = *pos;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Definitions */
    	geometry_msgs::Vector3 ref; // Reference position (from trajectory)
	double dr, dtheta; // Linear and angular displacements
	double ktheta = 0.1; // Angular control gain
	double vr, vtheta; // Desired linear and angular velocities
	geometry_msgs::Twist vel; // Velocity message to be published
	double tol = 5.0;  // Displacement tolerance (in cm)

	/* Initialize and assign node handler */
	ros::init(argc, argv, "robot_controller");
  	ros::NodeHandle n;

	/* Initialize publisher and subscriber */
	// TODO [OBTAIN ROBOT POSITION FROM MOCAP]
  	ros::Publisher  ctl_pub = 
		n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber path_sub = 
		n.subscribe("path", 1000, getPoint);
	ros::Subscriber en_sub = 
		n.subscribe("enable", 1, getStatus);	

	/* Update rate (period) */
	ros::Rate loop_rate(10);

	ROS_INFO("Robot Controller node active");

	/* Main loop */
	while (ros::ok())
	{
		/* Perform control only with flag enabled */
		if(enable)   
		{
			/* Compute displacements */
			dr = sqrt(pow(ref.x-robot.x,2)+pow(ref.y-robot.y,2));
			dtheta = atan2(ref.y-robot.y,ref.x-robot.x)-robot.theta;
			/* Obtain reference speeds (linear and angular) */
      			vr = 0.4; // Maximum robot speed
      			vtheta = ktheta*dtheta; // Proportional angular control

			/* Compose message and publish */
      			vel.linear.x = vr;
      			vel.angular.z = vtheta;
      			ctl_pub.publish(vel);

		     	ROS_INFO("Position reference: (%f, %f)",ref.x,ref.y);
			ROS_INFO("Robot position: (%f, %f)",robot.x,robot.y);
      			ROS_INFO("Set speed linear %f, angular %f\n",vr,vtheta);
    		
			/* Verify that the robot is close enough to target */
			if(dr<=tol)
			{
				/* Obtain new reference position */
      				if(!path.empty())
				{	
					ref = path.front();
      					path.erase(path.begin());
				}
				else
				{
					/* If path is empty, disable control */
					enable = false;
					ROS_INFO("Goal position reached");
				}
			}
		}
		else
		{
			/* Otherwise send a null velocity to robot */
			vel.linear.x = 0;
			vel.angular.z = 0;
			ctl_pub.publish(vel);
		}

		/* Spin and wait for next period */
    		ros::spinOnce();
    		loop_rate.sleep();
	}

	return 0;
}
