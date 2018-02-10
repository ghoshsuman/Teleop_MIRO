/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Monte Carlo Simulation service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/MonteCarlo.h"
#include <cmath>
#include <map>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

//Method to quantize coordinate of a point to index of discretized matrix, along a particular dimension
int quantize(float x, float xmin, float xmax, float quantum){
  int n = ceil((xmax-xmin)/quantum);
  int posOrigin = ceil(n/2.0)-1;
  float delta = 0;
  if (n%2 != 0)
    delta = quantum/2.0;
  int index = posOrigin + ceil((x-delta)/quantum);
  return index;
}

/* Constants */
#define H_SIZE 400 // Horizontal map size (in cm)
#define V_SIZE 400 // Vertical map size (in cm)
#define NZ 5 // Number of zones
#define RES 100 // Grid resolution

/* Service function */
bool MCSimulation(miro_teleop::MonteCarlo::Request  &req,
         	  miro_teleop::MonteCarlo::Response &res)
{
<<<<<<< HEAD
  int iters = 10000, batch = 1, max_x, max_y;
  float quantum = 10.0, sdx = 30, sdy = 30, rx, ry, xmin=-400, xmax=400, ymin=-400, ymax=400;
  float max_obj = 0, obj;

  //For c++11
  // random_device rd;
  // mt19937 e2(rd());
  // normal_distribution<> distx(Px, sdx);
  // normal_distribution<> disty(Py, sdy);

  //For boost with c++98
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
=======

	/* Input matrix containing the pertinences (received from master) */	
	std_msgs::Float64 landscape[(RES+1)*(RES+1)];

	/* Obtain input from request */
	for(int i=0;i<req.landscape.size();i++)
		landscape[i].data = req.landscape[i].data;

	/* TODO [BY NOW ASSUMING THAT GOAL = TARGET] */
	res.goal.x = 40;
	res.goal.y = 40;
	res.goal.theta = 0;
>>>>>>> d35845ce568f92a71dd64751e7cbe77d42bcc2d5

  //Normal Distribution
  //boost::normal_distribution<> distx(req.P.x, sdx);
  //boost::normal_distribution<> disty(req.P.y, sdy);
  // boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dx(*rng, distx);
  // boost::variate_generator< boost::mt19937, boost::normal_distribution<> > dy(*rng, disty);

  //Uniform Distribution
  boost::random::uniform_real_distribution<> distx(xmin, xmax);
  boost::random::uniform_real_distribution<> disty(ymin, ymax);
  boost::variate_generator< boost::mt19937, boost::random::uniform_real_distribution<> > dx(*rng, distx);
  boost::variate_generator< boost::mt19937, boost::random::uniform_real_distribution<> > dy(*rng, disty);


  //For c++98
  //std::srand(std::time(NULL));
  //float e2 =std::rand();
  while(max_obj<0.6){
  for (int i = 1; i <= iters; i++) {
    rx=dx();
    ry=dy();
    std::cout<<"Random point generated at ("<<rx<<","<<ry<<")\n";
    //Excluding points on and outside the boundary
    if(rx>xmin && rx<xmax && ry>ymin && ry<ymax){
      int index_x = quantize(rx, xmin, xmax, quantum);
      int index_y = quantize(ry, ymin, ymax, quantum);
      int ncols = ceil((xmax-xmin)/quantum);
      obj = req.landscape[index_y*ncols + index_x].data;
      if(obj>max_obj) {
        max_obj=obj;
        max_x=rx;
        max_y=ry;
      }
    }
  }
}
  res.goal.x = max_x;
  res.goal.y = max_y;
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
