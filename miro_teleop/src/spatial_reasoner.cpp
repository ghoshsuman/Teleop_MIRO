/* Libraries */
#include "ros/ros.h"
#include "miro_teleop/SpatialReasoner.h"
#include <cstdio>
#include <cmath>

/* Definitions */
#define PI 3.14159
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
	/* Output kernel */
	std_msgs::Float64 M[RES*RES];

	/* Obstacle center obtained from motion capture */
	double xr = req.center.x;
	double yr = req.center.y;
	/* Obstacle dimensions obtained as well */
	double a = req.dimensions[0].data;
	double b = req.dimensions[1].data;
	double dxy = a*a+b*b; // Parameter for the 'near' relation

	/* Other definitions */
	double xp, yp, xq, yq, xv, yv;
	double angle, beta_min, beta, dist_min, dist;

	ROS_INFO("Request received from master node");

	/* Obtain desired relation tag */
	int dir = req.relationship.data; 
	double c_ang = cos(dir*PI/2);
    double s_ang = sin(dir*PI/2);

	/* Set qualifier */
	double q;
	switch (req.qualifier.data)
	{
		case 0: // Weak
			q = 0.5;
			break;
		case 2: // Strong
			q = 2;
		default: // Normal, undefined cases
			q = 1;	
	}

	/* For every element P=(x,y) of the grid, compute the pertinences */
	for(int x=0;x<RES;x++)
	{
       	for(int y=0;y<RES;y++)
	   	{
			/* Map indices to actual location on the map */
			xp = HSIZE/double(2*RES)+HSIZE*(x/double(RES))-HSIZE/2;
			yp = VSIZE/double(2*RES)+VSIZE*(y/double(RES))-VSIZE/2;
			/* If P is inside the obstacle, pertinences are null */
			if((xp>(xr-a/2))&&(xp<(xr+a/2))&&(yp>(yr-b/2))&&(yp<(yr+b/2)))
        		M[x+y*RES].data=0;
			/* Otherwise, compute with respect to direction */
			else
			{
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
                        else beta = acos((xv*c_ang+yv*s_ang)/dist);
                        
						// Find minimum values
						if(beta<beta_min) beta_min = beta;
                        if(dist<dist_min) dist_min = dist;
                   	}
            	}
				/* Update matrices with minimum pertinences */
                if(dir<4) 
					M[x+y*RES].data = pow(fmax(0,1.0-(2.0*beta_min/PI)),q);
                else 
					M[x+y*RES].data = pow(exp(-pow(dist_min,2.0)/(2*dxy)),q);
			}
	   	}
	}

	/* Assign matrices to response structure */
	for (int i=0;i<RES*RES;i++) res.matrices.push_back(M[i]);

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

