#include "ros/ros.h"
#include "miro_teleop/PertinenceMapping.h"

#define SIZE 100

bool PertinenceMapper(miro_teleop::PertinenceMapping::Request  &req,
         	      miro_teleop::PertinenceMapping::Response &res)
{
	for(int i=0;i<SIZE;i++)
		for(int j=0;j<SIZE;j++)
			res.landscape[i+SIZE*j].data = 
				req.matrices[i+SIZE*j].data + 
				req.matrices[i+SIZE*j+1*SIZE*SIZE].data +
				req.matrices[i+SIZE*j+2*SIZE*SIZE].data +
				req.matrices[i+SIZE*j+3*SIZE*SIZE].data +
				req.matrices[i+SIZE*j+4*SIZE*SIZE].data;
	
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
