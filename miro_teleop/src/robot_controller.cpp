/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Robot Controller node source code */

/* Libraries */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "miro_msgs/platform_control.h"
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
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& pose)
{
	/* Obtain current robot position from motion capture */
	robot.x = 100*pose->x;
	robot.y = 100*pose->y;
	robot.theta = pose->theta;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Definitions */
    	geometry_msgs::Vector3 ref; // Reference position (from trajectory)
	double dr, dtheta; // Linear and angular displacements
	double ktheta = 1.0; // Angular control gain
	double vr, vtheta; // Desired linear and angular velocities
	miro_msgs::platform_control cmd_vel; // Message to be published
	double tol = 5.0;  // Displacement tolerance (in cm)

	/* Initialize and assign node handler */
	ros::init(argc, argv, "robot_controller");
  	ros::NodeHandle n;

	/* Initialize publishers and subscribers */
  	ros::Publisher  ctl_pub = 
		n.advertise<miro_msgs::platform_control>
					("/miro/rob01/platform/control", 10);
	ros::Subscriber path_sub = 
		n.subscribe("path", 1000, getPoint);
	ros::Subscriber en_sub = 
		n.subscribe("enable", 1, getStatus);	
	ros::Subscriber mocap_sub = 
		n.subscribe("Robot/ground_pose", 10, getRobotPose);

	/* Update rate (period) */
	ros::Rate loop_rate(10);

	ROS_INFO("Robot Controller node active");

	/* [SIMULATION ONLY] Initial robot position
	robot.x = -100;
	robot.y = -100;
	robot.theta = 0;
	*/

	/* Initialize reference with current robot position */
	ref.x = robot.x;
	ref.y = robot.y;

	/* Main loop */
	while (ros::ok())
	{
	
		/* [DEBUG] TEST RANDOM MSG TO MIRO
			cmd_vel.body_vel.linear.x = 250;
			cmd_vel.body_vel.angular.z = 0.01;
			ctl_pub.publish(cmd_vel);
		*/

		/* Perform control only with flag enabled */
		if(enable)   
		{
			/* Compute displacements */
			dr = sqrt(pow(ref.x-robot.x,2)+pow(ref.y-robot.y,2));
			dtheta = atan2(ref.y-robot.y,ref.x-robot.x)-robot.theta;

			/* Verify if the robot is close enough to target */
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
					ROS_INFO("Controller disabled");
				}
			}
			else
			{
		
				/* Obtain reference speeds (linear/angular) */
      				vr = 400*cos(dtheta); // Max. robot speed (m/s)
      				vtheta = ktheta*dtheta; // P angular control

				/* Compose message and publish */
      				cmd_vel.body_vel.linear.x = vr;
      				cmd_vel.body_vel.angular.z = vtheta;
      				ctl_pub.publish(cmd_vel);

		     		ROS_INFO("Position reference: (%f, %f)",
							   ref.x,ref.y);
				ROS_INFO("Robot position: (%f, %f)",
						   robot.x,robot.y);
      				ROS_INFO("Set speed linear %f, angular %f\n",
								  vr,vtheta);

				/* [FOR SIMULATION ONLY] Emulate robot movement
				double DT = 0.1;
				robot.x = robot.x + 100*DT*vr*cos(robot.theta);
				robot.y = robot.y + 100*DT*vr*sin(robot.theta);
				robot.theta = robot.theta + DT*vtheta; */
			}
		}
		else
		{
			/* Otherwise send a null velocity to robot */
			cmd_vel.body_vel.linear.x = 0.0;
			cmd_vel.body_vel.angular.z = 0.0;
			ctl_pub.publish(cmd_vel);
		}

		/* Spin and wait for next period */
    		ros::spinOnce();
    		loop_rate.sleep();
	}

	return 0;
}
