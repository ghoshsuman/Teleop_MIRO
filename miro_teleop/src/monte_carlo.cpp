/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/MonteCarlo.h"
#include <cstdio>
#include <cmath>
#include <map>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

/* Constants */
#define HSIZE 400 // Horizontal map size (in cm)
#define VSIZE 400 // Vertical map size (in cm)
#define RES 40 // Grid resolution
#define PERT_THRESH 0.8 // Minimum acceptable output pertinence
#define LIMIT 10000 // Limit simulation rounds (timeout constraint)

/** 
 * Method to quantize the coordinates of a point to indexes of a discretized 
 * matrix, along a particular dimension.
 *
 * @param x Current value
 * @param xmin Minimum value
 * @param xmax Maximum value
 * @param quantum Increment
 */
int quantize(float x, float xmin, float xmax, float quantum)
{
  	int n = ceil((xmax-xmin)/quantum);
  	int posOrigin = ceil(n/2.0)-1;
  	float delta = 0;
  	if (n%2 != 0) delta = quantum/2.0;
  	int index = posOrigin + ceil((x-delta)/quantum);
  	return index;
}

/**
 * Monte Carlo Simulation Service function.
 * Outputs a goal position by generating multiple random positions.
 *
 * Setting a minimum value and a maximum number of iterations, the algorithm
 * generates a set of random numbers and associates them with their respective
 * positions within the workspace grid. 
 * 
 * The position with maximum value is returned, if sufficiently pertinent.
 */
bool MCSimulation(miro_teleop::MonteCarlo::Request  &req,
  		  miro_teleop::MonteCarlo::Response &res)
{
   	int iters = 1000, batch = 1, max_x, max_y, count = 0;
    float quantum = HSIZE/RES, sdx = RES, sdy = RES, rx, ry, 
	xmin=-HSIZE/2, xmax=HSIZE/2, ymin=-VSIZE/2, ymax=VSIZE/2;
    float max_obj = 0, obj;

    // Obtain input request data
    std_msgs::Float64 landscape[RES*RES];
	float norm_val = 0.0;
    for(int i=0;i<req.landscape.size();i++)
	{
		landscape[i].data = req.landscape[i].data;
		if(landscape[i].data > norm_val) norm_val = landscape[i].data;
	}

	/* Normalize data received */
	if(norm_val>0)
    	for(int i=0;i<req.landscape.size();i++)
			landscape[i].data = landscape[i].data/norm_val;

    boost::mt19937 *rng = new boost::mt19937();

    rng->seed(time(NULL));

    // Uniform Distribution
    boost::random::uniform_real_distribution<> distx(xmin, xmax);
    boost::variate_generator< boost::mt19937,	
	boost::random::uniform_real_distribution<> > dx(*rng, distx);
    
	while(max_obj<PERT_THRESH)
	{
      	for (int i = 1; i <= iters; i++) 
		{
        	rx=dx();
        	ry=dx();
        	std::cout<<"Random point generated at ("<<rx<<","<<ry<<") ";
        	// Excluding points on and outside the boundary
        	if(rx>xmin && rx<xmax && ry>ymin && ry<ymax)
			{
          		int ncols = ceil((xmax-xmin)/quantum);
          		int nrows = ceil((ymax-ymin)/quantum);
          		int index_x = quantize(rx, xmin, xmax, quantum);
          		int index_y = (nrows-1-quantize(ry, ymin, ymax, quantum)); 
				// Invert to maintain mapping consistency 
          		obj = landscape[index_y*ncols + index_x].data;
          		std::cout<<"val = "<<obj<<std::endl;
          		if(obj>1)
            		ROS_INFO("rx=%f, ry=%f, obj=%f, (%d,%d)\n", 
								rx, ry, obj, index_x, index_y);
          		if(obj>max_obj && obj<=1) 
				{
            		max_obj=obj;
            		max_x=rx;
            		max_y=ry;
          		}
        	}
      	}
      	count++;
      	if(count>=LIMIT)
      	{
       		ROS_INFO("Timeout: maximum number of rounds exceeded");
       		// Return out-of-bound numbers as a timeout flag
			res.goal.x = 2*HSIZE;
			res.goal.y = 2*VSIZE;
       		return true;
      	}
   	}

    res.goal.x = max_x;
    res.goal.y = max_y;
    ROS_INFO("Goal position: (%f,%f) with val=%f", max_x, max_y, max_obj);
    return true;
}

/**
 * Monte Carlo Simulation Service Main function.
 * Initializes and advertises the service.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "monte_carlo_server");
    ros::NodeHandle n;
   	ros::ServiceServer service =
   		n.advertiseService("monte_carlo", MCSimulation);
   	ROS_INFO("Monte Carlo Simulation service active");
   	ros::spin();

   	return 0;
}
