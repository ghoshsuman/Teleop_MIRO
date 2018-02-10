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
#define NZ 5 // Number of zones
#define RES 100 // Grid resolution

/* Service function */
bool PertinenceMapper(miro_teleop::PertinenceMapping::Request  &req,
         	      miro_teleop::PertinenceMapping::Response &res)
{
	/* Input 3-D matrix to be processed (received from master) */
	std_msgs::Float64 matrices[NZ*(RES+1)*(RES+1)];

	/* Landscape matrix to be returned (mapped into an 1-D array) */
	std_msgs::Float64 landscape[(RES+1)*(RES+1)];

	ROS_INFO("Request received from master node");

	/* Obtain input from request */
	for(int i=0;i<req.matrices.size();i++) 
		matrices[i].data = req.matrices[i].data;

	/* Extract target coordinates and map to grid */ 
	int Px = floor((req.target.x+(H_SIZE/2.00))/H_SIZE);
	int Py = floor((req.target.y+(V_SIZE/2.00))/V_SIZE);

	/* Calculate point pertinences from input landscapes */
	double P[4];
	for(int dir=0;dir<4;dir++) P[dir] 
			= matrices[Px+RES*Py+RES*RES*dir].data;	

	/* Perform mapping of all landscapes into one */
	for(int i=0;i<=RES;i++)
		for(int j=0;j<=RES;j++)
			landscape[i+RES*j].data = 
				(P[0]*matrices[i+RES*j].data + 
				 P[1]*matrices[i+RES*j+1*RES*RES].data +
				 P[2]*matrices[i+RES*j+2*RES*RES].data +
				 P[3]*matrices[i+RES*j+3*RES*RES].data)*
				 matrices[i+RES*j+4*RES*RES].data;
	
	/* Attach obtained matrix to response */	
	for(int i=0;i<(RES+1)*(RES+1);i++) 
		res.landscape.push_back(landscape[i]);

	ROS_INFO("Successfully mapped the pertinences");

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
