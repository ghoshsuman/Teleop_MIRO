/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Monte Carlo Simulation service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/MonteCarlo.h"
#include <cmath>

/* Constants */
#define H_SIZE 400 // Horizontal map size (in cm)
#define V_SIZE 400 // Vertical map size (in cm)
#define NZ 5 // Number of zones
#define RES 100 // Grid resolution

/* Service function */
bool MCSimulation(miro_teleop::MonteCarlo::Request  &req,
         	  miro_teleop::MonteCarlo::Response &res)
{

	/* Input matrix containing the pertinences (received from master) */	
	std_msgs::Float64 landscape[(RES+1)*(RES+1)];

	/* Obtain input from request */
	for(int i=0;i<req.landscape.size();i++)
		landscape[i].data = req.landscape[i].data;

	/* TODO [BY NOW ASSUMING THAT GOAL = TARGET] */
	res.goal.x = 40;
	res.goal.y = 40;
	res.goal.theta = 0;

 	ROS_INFO("Goal position: (%f,%f,%f)", 
			       res.goal.x, res.goal.y, res.goal.theta);
  	return true;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Initialize, assign a node handler and advertise service */
	ros::init(argc, argv, "monte_carlo_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
			n.advertiseService("monte_carlo", MCSimulation);
	ROS_INFO("Monte Carlo Simulation service active");
	ros::spin();

	return 0;
}
