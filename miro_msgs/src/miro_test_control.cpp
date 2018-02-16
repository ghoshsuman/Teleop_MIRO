/* MIRO Controller Testing Application */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "miro_msgs/platform_control.h"
#include "miro_msgs/platform_sensors.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>

geometry_msgs::Pose2D robot; // Robot position
nav_msgs::Odometry odo; // Odometry information

/* Function that updates robot pose based on odometry */
void updateRobotPos(nav_msgs::Odometry odo, geometry_msgs::Pose2D pos, float dt)
{
	double vr = odo.twist.twist.linear.x;
	double vtheta = odo.twist.twist.angular.z;

	double dx = vr*cos(pos.theta)*dt;
	double dy = vr*sin(pos.theta)*dt;
    	double dtheta = vtheta*dt;

    	pos.x += dx;
    	pos.y += dy;
    	pos.theta += dtheta;
}

/* Subscriber callback functions */
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& pose)
{
        /* Obtain current robot position from motion capture */
        robot.x = 100*pose->x;
        robot.y = 100*pose->y;
        robot.theta = pose->theta;
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
        double tol = 5.0;  // Displacement tolerance (in cm)

	/* From odometry */


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
                        ROS_INFO("Robot position: (%f, %f)", robot.x, robot.y);
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
