#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>
#include "miro_teleop/MonteCarlo.h"

#define RES 100

bool MCSimulation(miro_teleop::MonteCarlo::Request  &req,
         	  miro_teleop::MonteCarlo::Response &res)
{

	res.goal.x = 40;
	res.goal.y = 40;
	res.goal.theta = 0;

 	ROS_INFO("Goal position: (%f,%f,%f)", 
			       res.goal.x, res.goal.y, res.goal.theta);
  	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "monte_carlo_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
			n.advertiseService("monte_carlo", MCSimulation);
	ROS_INFO("Monte Carlo service active");
	ros::spin();

	return 0;
}
