#include "ros/ros.h"
#include "miro_teleop/SpatialReasoner.h"
#include <cmath>

#define PI 3.14159
#define NZ 5
#define H_SIZE 400
#define V_SIZE 400
#define RES 100

bool SpatialReasoner(miro_teleop::SpatialReasoner::Request  &req,
         	     miro_teleop::SpatialReasoner::Response &res)
{
	double xr = req.center.x;
	double yr = req.center.y;

	double a = req.dimensions[0].data;
	double b = req.dimensions[1].data;

	double xp, yp, xq, yq, xv, yv;
	double ang, beta_min, beta, dist_min, dist;
        double c_ang, s_ang;

	for(int x=0;x<=RES;x++)
	{
		for(int y=0;y<=RES;y++)
		{
			xp = H_SIZE*(x/double(RES));
			yp = V_SIZE*(y/double(RES));
			if((xp>xr)&&(xp<(xr+a))&&(yp>yr)&&(yp<(yr+b)))
                        {
                                for(int dir=0; dir<NZ; dir++) 
				res.matrices[x+y*RES+dir*RES*RES].data=0;
                                break;
                        }
                        for(int dir=0; dir<NZ-1; dir++) // For each direction
                        {
                                ang  = dir*PI/2; // Current angle
                                c_ang = cos(ang);
                                s_ang = sin(ang);

                                beta_min = PI/2; // Initial value of beta
                                dist_min = 1000; // Initial value of dist
                                
				for(int i=0;i<=RES;i++)
                                {
                                        for(int j=0;j<=RES;j++)
                                        {
                                                xq = xr+a*(i/double(RES));
                                                yq = yr+b*(j/double(RES));
                                                xv = xp-xq;
                                                yv = yp-yq;
                                                dist = sqrt(xv*xv+yv*yv);
                                                if(dist==0) beta=0;
                                                else beta = 
						acos((xv*c_ang+yv*s_ang)/dist);
                                                if(beta<beta_min) 
						beta_min = beta;
                                                if(dist<dist_min) 
						dist_min = dist;
                                        }
                                }
                                res.matrices[x+y*RES+dir*RES*RES].data
					       	    = fmax(0,1-(2*beta_min/PI));
                                res.matrices[x+y*RES+4*RES*RES].data = dist_min;
                        }
		}
	}
  	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "spatial_reasoning_server");
	ros::NodeHandle n;
	ros::ServiceServer service = 
		n.advertiseService("spatial_reasoner", SpatialReasoner);
	ROS_INFO("Spatial Reasoning service active");
	ros::spin();

	return 0;
}
