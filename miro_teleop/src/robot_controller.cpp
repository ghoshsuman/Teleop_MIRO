/* Libraries */
#include "miro_teleop/Path.h"
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
geometry_msgs::Pose2D gesture; // Gesture position

/** 
 * Subscriber callback function.
 * Obtains trajectory from Command Logic node. 
 */
void getPoint(const miro_teleop::Path::ConstPtr& points)
{
	/* Append every path position received to current reference path */
	path.clear();
	// path = points->path;
	ROS_INFO("Received new path");
	for(int i=0;i< points->path.size();i++)
		path.push_back(points->path[i]);
}

/** 
 * Subscriber callback function.
 * Obtains enable/disable flag from Command Logic node. 
 */
void getStatus(const std_msgs::Bool::ConstPtr& status)
{
	enable = status->data;
	if(enable) ROS_INFO("Controller enabled");
	else ROS_INFO("Controller disabled");
}

/** 
 * Subscriber callback function.
 * Obtains current robot pose from Motion Capture node. 
 */
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& pose)
{
	robot.x = 100*pose->x; //in cm
	robot.y = 100*pose->y; //in cm
	robot.theta = pose->theta;
}

/** 
 * Subscriber callback function.
 * Obtains current gesture pose from Motion Capture node. 
 */
void getCommanderPose(const geometry_msgs::Pose2D::ConstPtr& pose)
{
	gesture.x = 100*pose->x;
	gesture.y = 100*pose->y;
}

/**
 * Robot Controller Node main function.
 * Performs robot position and orientation control.
 *
 * Given a reference from the trajectory received from the Command Logic,
 * the reference linear and angular velocities are computed based on the
 * current distance and orientation error. This only happens if the flag
 * enable is set to 'true'. Otherwise, the command velocities are set to 0.
 *
 * When the reference position is reached (i.e. the distance is less than tol),
 * the new desired position is taken from from the trajectory array. 
 *
 * If the path is empty, the goal position is considered reached and the enable
 * flag is set to 'false'.
 */
int main(int argc, char **argv)
{
	/* Definitions */
	geometry_msgs::Vector3 ref; // Reference position (from trajectory)
	geometry_msgs::Pose2D commPos; // Commander position
	double dr, dtheta; // Linear and angular displacements
	double ktheta = 1; // Angular control gain
	double vr, vtheta; // Desired linear and angular velocities
	miro_msgs::platform_control cmd_vel; // Message to be published
	double tol = 20.0;  // Displacement tolerance (in cm)
	bool turn_blink;

	/* Initialize and assign node handler */
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle n;

	/* Initialize publishers and subscribers */
	ros::Publisher  ctl_pub =
	n.advertise<miro_msgs::platform_control>
	("/miro/rob01/platform/control", 10);
	ros::Subscriber path_sub =
	n.subscribe("path", 1, getPoint);
	ros::Subscriber en_sub =
	n.subscribe("enable", 1, getStatus);
	ros::Subscriber mocap_sub =
	n.subscribe("Robot/ground_pose", 10, getRobotPose);
	ros::Subscriber gest_sub = 
	n.subscribe("Gesture/ground_pose", 10, getCommanderPose);

	/* Update rate (period) */
	ros::Rate loop_rate(10);

	ROS_INFO("Robot Controller node active");

	/* Main loop */
	while (ros::ok())
	{

		/* Perform control only with flag enabled */
		if(enable)
		{
			ref = path.front();
			/* Compute displacements */
			dr = sqrt(pow(ref.x-robot.x,2)+pow(ref.y-robot.y,2));
			dtheta = atan2(ref.y-robot.y,ref.x-robot.x)-robot.theta;
			dtheta = atan2(sin(dtheta),cos(dtheta));

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
					
					// Then MiRo turn and blink to commander
					turn_blink = true;
				}
			}
			else
			{

				/* Obtain reference speeds (linear/angular) */
				vr = 200*cos(dtheta); // Max speed: 400 cm/s
				vtheta = ktheta*dtheta; // P angular control

				/* Compose message and publish */
				cmd_vel.body_vel.linear.x = vr;
				cmd_vel.body_vel.angular.z = vtheta;
				ctl_pub.publish(cmd_vel);

				ROS_INFO("Position reference: (%f, %f)",
				ref.x,ref.y);
				ROS_INFO("Robot position: (%f, %f)",
				robot.x,robot.y);
				ROS_INFO("Displacement: (%f, %f)",dr,dtheta);
				ROS_INFO("Set speed linear %f, angular %f\n",
				vr,vtheta);
			}
		}
		else
		{
			if(turn_blink)
			{
			commPos = gesture;
			dtheta = atan2(commPos.y-robot.y,
			commPos.x-robot.x)-robot.theta;
                       	dtheta = atan2(sin(dtheta),cos(dtheta));
			cmd_vel.body_vel.linear.x = 0;
                       	cmd_vel.body_vel.linear.z = dtheta;
                      	cmd_vel.blink_time = 10; // 200ms blink
			ctl_pub.publish(cmd_vel);
			if(dtheta<0.1) turn_blink = false;
			}
			else
			{
			/* Otherwise send a null velocity to robot */
			cmd_vel.body_vel.linear.x  = 0.0;
			cmd_vel.body_vel.angular.z = 0.0;
			ctl_pub.publish(cmd_vel);
			}
		}

		/* Spin and wait for next period */
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
