/* Libraries */
#include "miro_teleop/Path.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "miro_msgs/platform_control.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <cmath>
#include <sstream>
#include <fstream>

#define GOAL_SIZE 20

/* Global variables */
bool enable = false;  // Controller status flag
std::vector<geometry_msgs::Vector3> path; // Trajectory array
geometry_msgs::Pose2D robot; // Robot position
geometry_msgs::Pose2D gesture; // Gesture position

const std::string getData(){
	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
	std::string currentDate(mbstr);
	return currentDate;
}

// For logging purposes - function to write to file a single string
std::string printPath = "robotcontroller_" + getData() + ".log";
void writeStrToFile( const std::string &toWrite){
        std::string formattedTime = getData();
	std::ofstream file;
	file.open(printPath.c_str(), std::ofstream::out | std::ofstream::app);
        file << formattedTime << ": " << toWrite << "\n";
	file.close();
}

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
	for(int i=0;i< points->path.size();i++)
	{	
		std::stringstream ss;
		ss << "\nPath point "<<i<<" : "<< path[i].x << ", "<<path[i].y;
		std::string sout = ss.str();		
		writeStrToFile(sout);
		ROS_INFO("Path point %d: %f, %f,", i, path[i].x, path[i].y);
	}
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
	// If data from mocap is lost, stop robot
	if(robot.x==100*pose->x && robot.y==100*pose->y)
	{	
		enable = false;
	}

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

/* Maps color to lights in robot */
void colormap(int color, miro_msgs::platform_control *cmd_vel)
{
	// Convention: 0 = none, 1 = red, 2 = green, 3 = blue, 4 = yellow
	// 5 - purple. 6 - white
	for (int i = 0; i<18; i++)
	{
		if(color>0 && color<4) 
			cmd_vel->lights_raw[i] = 255*((i+4-color)%3==0);
		else if(color==4)
			cmd_vel->lights_raw[i] = 255-255*((i-2)%3==0);
		else if(color==5)
			cmd_vel->lights_raw[i] = 255-255*((i-1)%3==0);
		else if(color==6)
			cmd_vel->lights_raw[i] = 255;
		else
			cmd_vel->lights_raw[i] = 0;
	}
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
	int color, color_temp; // Corresponding colors
	double tol = GOAL_SIZE;  // Displacement tolerance (in cm)

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
		/* Get color from parameter server (default is -1 - no color) */
  		n.param("/color_key", color_temp, -1);
		if(color_temp!=color) ROS_INFO("Color: %d", color_temp);
		color = color_temp;

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
				}
			}
			else
			{

				/* Obtain reference speeds (linear/angular) */
                                vr = 400*cos(dtheta); // Max speed: 400 cm/s
				vtheta = ktheta*dtheta; // P angular control

				/* Compose message and publish */
				cmd_vel.body_vel.linear.x = vr;
				cmd_vel.body_vel.angular.z = vtheta;
				colormap(color, &cmd_vel);
				ctl_pub.publish(cmd_vel);

				ROS_INFO("Position reference: (%f, %f)",ref.x,ref.y);
				ROS_INFO("Robot position: (%f, %f)",robot.x,robot.y);
				ROS_INFO("Displacement: (%f, %f)",dr,dtheta);
				ROS_INFO("Set speed linear %f, angular %f\n",vr,vtheta);
			}
		}
		else
		{
			/* Otherwise send a null velocity to robot */
			cmd_vel.body_vel.linear.x  = 0.0;
			cmd_vel.body_vel.angular.z = 0.0;
			colormap(color, &cmd_vel);
			ctl_pub.publish(cmd_vel);
		}

		/* Spin and wait for next period */
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
