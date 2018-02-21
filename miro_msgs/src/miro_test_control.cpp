/* MIRO Controller Testing Application */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "miro_msgs/platform_control.h"
#include "miro_msgs/platform_sensors.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>

geometry_msgs::Pose2D mocap, robot; // Robot position (mocap and current)
nav_msgs::Odometry odo; // Odometry information retrieved from miro sensors
double dt = 0.1; // Time between instances (which corresponds to 1/loop_rate)
bool mocap_available = false; // Flag that indicates if mocap detects the robot

/* Function that updates robot pose based on odometry */
geometry_msgs::Pose2D updateRobotPos(geometry_msgs::Pose2D pose)
{
	double vr = odo.twist.twist.linear.x;
	double vtheta = odo.twist.twist.angular.z;

	double dx = vr*cos(pose.theta)*dt;
	double dy = vr*sin(pose.theta)*dt;
    	double dtheta = vtheta*dt;

    	pose.x += dx;
    	pose.y += dy;
    	pose.theta += dtheta;

	return pose;
}

/* Subscriber callback functions */
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& pose)
{
	/* Verify whether data retrieved from mocap has changed */
	// When mocap doesn't detect an object it retrieves the exact values
	if(mocap.x==pose->x && mocap.y==pose->y && mocap.theta==pose->theta)
		mocap_available = false;
	else
	{
		mocap_available = true;
        	/* Obtain robot position from motion capture */
        	mocap.x = 100*pose->x;
	        mocap.y = 100*pose->y;
        	mocap.theta = pose->theta;
	}
}
void getOdomInfo(const miro_msgs::platform_sensors::ConstPtr& msg)
{
        /* Obtain robot odometry information */
        odo = msg->odometry;
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
        double tol = 10.0;  // Displacement tolerance (in cm)

	/* Initialize and assign node handler */
        ros::init(argc, argv, "robot_controller");
        ros::NodeHandle n;

        /* Initialize publishers and subscribers */
        ros::Publisher  ctl_pub =
                n.advertise<miro_msgs::platform_control>
                                        ("/miro/rob01/platform/control", 10);

	// Obtain position from motion capture
	ros::Subscriber mocap_sub =
                n.subscribe("Robot/ground_pose", 10, getRobotPose);
	// Estimate position from odometry when motion capture is unavailable
	ros::Subscriber odom_sub = 
		n.subscribe("/miro/rob01/platform/sensors", 10, getOdomInfo);

	/* Update rate (period) */
        ros::Rate loop_rate(10);

	ROS_INFO("TESTING Robot Controller");

	/* Set initial reference */
	ref.x = -10.0;
	ref.y = -10.0;

	/* Main loop */
        while (ros::ok())
        {
		/* Verify whether odometry is necessary */
		if(mocap_available) robot = mocap;
		else robot = updateRobotPos(robot);

		/* Compute displacements */
                dr = sqrt(pow(ref.x-robot.x,2)+pow(ref.y-robot.y,2));
                dtheta = atan2(ref.y-robot.y,ref.x-robot.x)-robot.theta;

                /* Verify if the robot is close enough to target */
                if(dr<=tol)
                {
			ROS_INFO("Reference position reached.");
                	break;
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

			/* Display current position and velocity */
                        ROS_INFO("Position reference: (%f, %f)", ref.x, ref.y);
                        ROS_INFO("Robot position: (%f, %f)", 
							robot.x, robot.y);
                        ROS_INFO("Set speed linear %f, angular %f\n",vr,vtheta);

			/* Display odometry information */
			ROS_INFO("Odometry:");
                        ROS_INFO("vr = %f", odo.twist.twist.linear.x);
                        ROS_INFO("vtheta = %f\n", odo.twist.twist.angular.z);
                }

		/* Spin and wait for next period */
                ros::spinOnce();
                loop_rate.sleep();
	}

	return 0;
}
