/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/SpatialReasoner.h"
#include <cstdio>
#include <cmath>

/* Definitions */
#define PI 3.14159
#define NZ 5 // Number of relations (north, south, west, east, distance-to)
#define HSIZE 400 // Horizontal map size (in cm)
#define VSIZE 400 // Vertical map size (in cm)
#define RES 40 // Grid resolution

/**
 * Spatial Reasoner Service function.
 * Generates the spatial relation landscape matrices.
 *
 * For each matrix, the workspace is discretized to a RESxRES matrix,
 * each element containing the value corresponding to the pertinence of the
 * spatial relation referrent to the matrix on a certain object, i.e., the 
 * degree of certainty that the point is at SUCH direction with respect to 
 * the object. These values lie bewteen 0 and 1.
 *
 * The "distance-to" relation returns the pertinence with respect to a desired
 * distance range from the object, which can be modified.
 */
bool SpatialReasoner(miro_teleop::SpatialReasoner::Request  &req,
         	     miro_teleop::SpatialReasoner::Response &res)
{
	/* Matrices to be sent back to master (mapped in an 1-D array) */
	std_msgs::Float64 M[NZ*RES*RES];

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

	ROS_INFO("Request received from master node");

	/* For every element P=(x,y) of the grid, compute the pertinences */
	for(int x=0;x<RES;x++)
	{
           for(int y=0;y<RES;y++)
	   {
		/* Map indices to actual location on the map */
		xp = HSIZE/double(2*RES)+HSIZE*(x/double(RES))-HSIZE/2;
		yp = VSIZE/double(2*RES)+VSIZE*(y/double(RES))-VSIZE/2;
		/* If P is inside the obstacle, pertinences are null */
		if((xp>(xr-a/2))&&(xp<(xr+a/2))
					&&(yp>(yr-b/2))&&(yp<(yr+b/2)))
                {
                               for(int dir=0; dir<NZ; dir++) 
				M[x+y*RES+dir*RES*RES].data=0;
                }
		/* Otherwise, compute with respect to each direction */
		else
		{
                       	for(int dir=0; dir<NZ-1; dir++)
                       	{
                               	angle = dir*PI/2; // Direction angle
                               	c_ang = cos(angle);
                               	s_ang = sin(angle);
                               	beta_min = PI/2; // Initial value of beta
                               	dist_min = 1000; // Initial value of distance
                               
				/* For every element of the object calculate */
				for(int i=0;i<RES;i++)
                               	{
                                       	for(int j=0;j<RES;j++)
                                       	{
						// Compute beta and dist
                                               	xq = xr-a/2+a*(i/double(RES));
                                               	yq = yr-b/2+b*(j/double(RES));
                                               	xv = xp-xq;
                                               	yv = yp-yq;
                                               	dist = sqrt(xv*xv+yv*yv);
                                               	if(dist==0) beta=0;
                                               	else beta = 
						acos((xv*c_ang+yv*s_ang)/dist);
                                               	// Find minimum values
						if(beta<beta_min) 
						beta_min = beta;
                                               	if(dist<dist_min) 
						dist_min = dist;
                                       	}
                               	}
				/* Update matrices with minimum pertinences */
                               	M[x+y*RES+dir*RES*RES].data
					= fmax(0,1.0-(2.0*beta_min/PI));
				/* Update the minimum distance of P to object */
                               	M[x+y*RES+4*RES*RES].data 
				= fmin(1,fmax(0,dist_min*exp(-dist_min/60)/20));
                       	}
		}
	   }
	}

	/* Assign matrices to response structure */
	for (int i=0;i<NZ*RES*RES;i++) res.matrices.push_back(M[i]);

	/* Optional: Print matrices */
	for (int dir=0;dir<NZ;dir++) 
	{
		ROS_INFO("Printing Matrix %d:",dir+1);
		for (int i=0;i<RES;i++)
		{
			for (int j=0;j<RES;j++)
				printf("%3.2f ", M[i+j*RES+dir*RES*RES].data);
			printf("\n");
		}		
		printf("\n\n");
	}

	ROS_INFO("Successfully generated landscapes");	

  	return true;
}

/**
 * Spatial Reasoner Service Main function.
 * Initializes and advertises the service.
 */
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

