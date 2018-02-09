/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Spatial Reasoner service source code */

/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/SpatialReasoner.h"
#include <cmath>

/* Constants */
#define PI 3.14159
#define NZ 5 // Number of relations (north, south, west, east, distance-to)
#define H_SIZE 400 // Horizontal map size (in cm)
#define V_SIZE 400 // Vertical map size (in cm)
#define RES 100 // Grid resolution

/* Service function */
bool SpatialReasoner(miro_teleop::SpatialReasoner::Request  &req,
         	     miro_teleop::SpatialReasoner::Response &res)
{
	/* Obstacle center obtained from motion capture */
	double xr = req.center.x;
	double yr = req.center.y;
	/* Obstacle dimensions obtained as well */
	double a = req.dimensions[0].data;
	double b = req.dimensions[1].data;

	/* Other definitions */
	double xp, yp, xq, yq, xv, yv;
	double angle, beta_min, beta, dist_min, dist;
        double c_ang, s_ang; // Store cosines and sines to increase performance

	/* For every element P=(x,y) of the grid, compute the pertinences */
	for(int x=0;x<=RES;x++)
	{
		for(int y=0;y<=RES;y++)
		{
			/* Map indices to actual location on the map */
			xp = H_SIZE*(x/double(RES));
			yp = V_SIZE*(y/double(RES));
			/* If P is inside the obstacle, pertinences are null */
			if((xp>xr)&&(xp<(xr+a))&&(yp>yr)&&(yp<(yr+b)))
                        {
                                for(int dir=0; dir<NZ; dir++) 
				res.matrices[x+y*RES+dir*RES*RES].data=0;
                                break;
                        }
			/* Otherwise, compute with respect to each direction */
                        for(int dir=0; dir<NZ-1; dir++)
                        {
                                angle = dir*PI/2; // Direction angle
                                c_ang = cos(angle);
                                s_ang = sin(angle);

                                beta_min = PI/2; // Initial value of beta
                                dist_min = 1000; // Initial value of distance
                                
				/* For every element of the object calculate */
				for(int i=0;i<=RES;i++)
                                {
                                        for(int j=0;j<=RES;j++)
                                        {
						/* Compute beta */
                                                xq = xr+a*(i/double(RES));
                                                yq = yr+b*(j/double(RES));
                                                xv = xp-xq;
                                                yv = yp-yq;
                                                dist = sqrt(xv*xv+yv*yv);
                                                if(dist==0) beta=0;
                                                else beta = 
						acos((xv*c_ang+yv*s_ang)/dist);

                                                /* Find minimum beta and dist */
						if(beta<beta_min) 
						beta_min = beta;
                                                if(dist<dist_min) 
						dist_min = dist;
                                        }
                                }
				/* Update matrices with minimum pertinences */
                                res.matrices[x+y*RES+dir*RES*RES].data
					       	    = fmax(0,1-(2*beta_min/PI));
				/* Update the minimum distance of P to object */
                                res.matrices[x+y*RES+4*RES*RES].data = dist_min;
                        }
		}
	}
  	return true;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Initialize, assign a node handler and advertise service */
	ros::init(argc, argv, "spatial_reasoning_server");
	ros::NodeHandle n;
	ros::ServiceServer service =
		n.advertiseService("spatial_reasoner", SpatialReasoner);
	ROS_INFO("Spatial Reasoning service active");
	ros::spin();

	return 0;
}
