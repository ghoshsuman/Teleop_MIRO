/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Pertinence Mapper service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/PertinenceMapping.h"
#include <cmath>

/* Constants */
#define H_SIZE 400 // Horizontal map size (in cm)
#define V_SIZE 400 // Vertical map size (in cm)
#define RES 100 // Grid resolution

/* Service function */
bool PertinenceMapper(miro_teleop::PertinenceMapping::Request  &req,
         	      miro_teleop::PertinenceMapping::Response &res)
{
	/* Extract target coordinates and map to grid */ 
	int Px = floor((req.target.x+(H_SIZE/2.00))/H_SIZE);
	int Py = floor((req.target.y+(V_SIZE/2.00))/V_SIZE);
	
	/* Calculate point pertinences from input landscapes */
	double P[4];
	for(int dir=0;dir<4;dir++) P[dir] 
			= req.matrices[Px+RES*Py+RES*RES*dir].data;	

	/* Perform mapping of all landscapes into one */
	for(int i=0;i<=RES;i++)
		for(int j=0;j<=RES;j++)
			res.landscape[i+RES*j].data = 
				(P[0]*req.matrices[i+RES*j].data + 
				 P[1]*req.matrices[i+RES*j+1*RES*RES].data +
				 P[2]*req.matrices[i+RES*j+2*RES*RES].data +
				 P[3]*req.matrices[i+RES*j+3*RES*RES].data)*
				 req.matrices[i+RES*j+4*RES*RES].data;
	
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
