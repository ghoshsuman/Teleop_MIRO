/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Pertinence Mapper service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/PertinenceMapping.h"
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/* Constants */
#define HSIZE 400 // Horizontal map size (in cm)
#define VSIZE 400 // Vertical map size (in cm)
#define NZ 5 // Number of zones
#define RES 40 // Grid resolution

/* Service function */
bool PertinenceMapper(miro_teleop::PertinenceMapping::Request  &req,
         	      miro_teleop::PertinenceMapping::Response &res)
{
	/* Input 3-D matrix to be processed (received from master) */
	std_msgs::Float64 matrices[NZ*RES*RES];

	/* Landscape matrix to be returned (mapped into an 1-D array) */
	std_msgs::Float64 landscape[RES*RES];

	double max = 0;

	ROS_INFO("Request received from master node");

	// DEBUG
	ROS_INFO("Target: (%f %f)",req.target.x,req.target.y);
	ROS_INFO("Matrices size: %d",req.matrices.size());

	/* Obtain input from request */
	for(int i=0;i<req.matrices.size();i++)
		matrices[i].data = req.matrices[i].data;

	/* Extract target coordinates and map to grid */
	int Px = floor((req.target.x+HSIZE/2)*RES/HSIZE);
	int Py = RES-1-floor((req.target.y+VSIZE/2)*RES/VSIZE); //Inverting y coordinate to match matrix ordering
	ROS_INFO("Point element coordinates: [%d, %d]",Px,Py);

	/* Calculate point pertinences from input landscapes */
	double P[4];
	for(int dir=0;dir<4;dir++) {
    P[dir] = matrices[Px+RES*Py+RES*RES*dir].data;
    ROS_INFO("P[%d]=%f",dir,P[dir]);
  }

	/* Perform mapping of all landscapes into one */
  // i = columns, j = rows
	for(int i=0;i<RES;i++)
	{
		for(int j=0;j<RES;j++)
		{
			landscape[i+RES*j].data =
				(P[0]*matrices[i+RES*j].data +
				 P[1]*matrices[i+RES*j+1*RES*RES].data +
				 P[2]*matrices[i+RES*j+2*RES*RES].data +
				 P[3]*matrices[i+RES*j+3*RES*RES].data)
				*matrices[i+RES*j+4*RES*RES].data;
			if(landscape[i+RES*j].data>max)
				max = landscape[i+RES*j].data;
		}
	}
	/* Attach obtained matrix to response */
	for(int i=0;i<RES*RES;i++)
	{
		landscape[i].data = (landscape[i].data)/max;
		res.landscape.push_back(landscape[i]);
	}


	ROS_INFO("Successfully mapped the pertinences");

	/* Optional: Print matrices */
	ROS_INFO("Matrix generated:");
        for (int j=0;j<RES;j++)
        {
                for (int i=0;i<RES;i++)
                        printf("%3.2f ",landscape[i+j*RES].data);
                printf("\n");
        }

  	return true;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Initialize, assign a node handler and advertise service */
	ros::init(argc, argv, "pertinence_mapping_server");
	ros::NodeHandle n;
	ros::ServiceServer service =
		n.advertiseService("pertinence_mapper", PertinenceMapper);
	ROS_INFO("Pertinence Mapping service active");
	ros::spin();

	return 0;
}
