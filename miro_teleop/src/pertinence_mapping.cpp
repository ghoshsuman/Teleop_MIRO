#include "ros/ros.h"
#include "miro_teleop/PertinenceMapping.h"
#include <cmath>

#define RES 100
#define H_SIZE 400
#define V_SIZE 400

bool PertinenceMapper(miro_teleop::PertinenceMapping::Request  &req,
         	      miro_teleop::PertinenceMapping::Response &res)
{

	double P[4];
	int Px = floor((req.target.x+(H_SIZE/2.00))/H_SIZE);
	int Py = floor((req.target.y+(V_SIZE/2.00))/V_SIZE);

	for(int dir=0;dir<4;dir++) P[dir] 
			= req.matrices[Px+RES*Py+RES*RES*dir].data;	
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pertinence_mapping_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
		n.advertiseService("pertinence_mapper", PertinenceMapper);
	ROS_INFO("Pertinence Mapping service active");
	ros::spin();

	return 0;
}
