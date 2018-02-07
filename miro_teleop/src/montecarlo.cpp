/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Monte Carlo Simulation service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/MonteCarlo.h"
#include <cmath>

/* Service function */
bool MCSimulation(miro_teleop::MonteCarlo::Request  &req,
         	  miro_teleop::MonteCarlo::Response &res)
{
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
