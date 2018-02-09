/* University of Genoa - Software Architecture for Robotics (2017/2018) */
/* Project: Teleoperation with MIRO - Mateus Sanches Moura, Suman Ghosh */
/* Command Logic (Master) node source code */

/* Libraries */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "miro_teleop/GestureProcessing.h"
#include "miro_teleop/SpatialReasoner.h"
#include "miro_teleop/PertinenceMapping.h"
#include "miro_teleop/MonteCarlo.h"
#include "rrtstar_msgs/rrtStarSRV.h"
#include "rrtstar_msgs/Region.h"

/* Definitions */
#define RES 100 // Grid resolution
#define NZ 5 // Number of relations (north, south, west, east, distance-to)

/* Global variables */
std_msgs::UInt8 cmd; // Command tag received from interpreter
geometry_msgs::Pose2D obs, robot; // Obstacle and robot positions from mocap
geometry_msgs::Pose gesture; // Gesture information to be processed from mocap

/* Subscriber callback functions */
void getCmd(const std_msgs::UInt8::ConstPtr& msg)
{
	/* Obtain command tag from interpreter */
	cmd.data = msg->data;
	ROS_INFO("Command received from interpreter");
}
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	/* Obtain robot position from mocap */
	robot = *groundpose;
}
void getGesture(const geometry_msgs::Pose::ConstPtr& pose)
{
	/* Obtain gesture information from mocap */
	gesture = *pose;
}
void getObstaclePose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	/* Obtain obstacle position from mocap */
	obs = *groundpose;
}

/* Main function */
int main(int argc, char **argv)
{
	/* Definitions */
	geometry_msgs::Pose2D target, goal; // Target and goal positions
  	geometry_msgs::Vector3 path[1000]; // Trajectory to be published
	std_msgs::Float64 matrices[NZ*(RES+1)*(RES+1)]; // From spatial reasoner
	std_msgs::Float64 landscape[(RES+1)*(RES+1)]; // From pertinence mapping
  	std_msgs::Bool enable; // Controller enable flag
	rrtstar_msgs::Region workspace, goal_reg, obs_reg; // For RRT* algorithm
	double pathsize; // Since RRT* trajectory size is variable
	
	/* Characterize workspace (predefined) */
	workspace.center_x = 0;
  	workspace.center_y = 0;
  	workspace.center_z = 0;
  	workspace.size_x = 400;
  	workspace.size_y = 400;
  	workspace.size_z = 0;

	/* Obstacle dimensions (predefined) */
	std_msgs::Float64 obsdim[2];
  	obsdim[0].data = 5.00;
 	obsdim[1].data = 5.00;

	/* Initialize and assign node handler */
	ros::init(argc, argv, "command_logic");
  	ros::NodeHandle n;

	/* Initialize publishers and subscribers */
	// Publishers to robot controller
	ros::Publisher path_pub = 
		n.advertise<geometry_msgs::Vector3>("path", 1000);
  	ros::Publisher flag_pub = 
		n.advertise<std_msgs::Bool>("enable", 1);
	// Subscriber from command interpreter
	ros::Subscriber sub_cmd = 
		n.subscribe("command", 3, getCmd);
	// Subscribers from motion capture (mocap)
	ros::Subscriber sub_robot = 
		n.subscribe("Robot/ground_pose", 10, getRobotPose);
	ros::Subscriber sub_gesture = 
		n.subscribe("Gesture/pose", 1, getGesture);
	ros::Subscriber sub_obs = 
		n.subscribe("Obstacle/ground_pose", 1, getObstaclePose);

	/* Initialize service clients and handlers */
  	ros::ServiceClient cli_spat = 
	n.serviceClient<miro_teleop::SpatialReasoner>("spatial_reasoner");
  	miro_teleop::SpatialReasoner srv_spat;

  	ros::ServiceClient cli_gest = 
	n.serviceClient<miro_teleop::GestureProcessing>("gesture_processing");
  	miro_teleop::GestureProcessing srv_gest;

  	ros::ServiceClient cli_pert = 
	n.serviceClient<miro_teleop::PertinenceMapping>("pertinence_mapper");
  	miro_teleop::PertinenceMapping srv_pert;

  	ros::ServiceClient cli_mont = 
	n.serviceClient<miro_teleop::MonteCarlo>("monte_carlo");
  	miro_teleop::MonteCarlo srv_mont;

  	ros::ServiceClient cli_rrts = 
	n.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
  	rrtstar_msgs::rrtStarSRV srv_rrts;

	/* Update rate (period) */
  	ros::Rate loop_rate(10);

	// [SIMULATION VALUES]
  	gesture.position.x = 40;
  	gesture.position.y = 40;
  	gesture.position.z = 20;

  	gesture.orientation.w = 0;
  	gesture.orientation.x = 0;
  	gesture.orientation.y = 0;
  	gesture.orientation.z = -1;
 
  	obs.x = 20.00;
  	obs.y = 20.00;
  	obs.theta = 0;

  	robot.x = -10.00;
  	robot.y = -10.00;
  	robot.theta = 0;

      	goal_reg.center_x = goal.x;
 	goal_reg.center_y = goal.y;
  	goal_reg.center_z = 0;
  	goal_reg.size_x = 10;
  	goal_reg.size_y = 10;
  	goal_reg.size_z = 0;
  
  	geometry_msgs::Vector3 init;
  	init.x = robot.x;
  	init.y = robot.y;
  	init.z = 0;

  	obs_reg.center_x = obs.x;
  	obs_reg.center_y = obs.y;
  	obs_reg.center_z = 0;
  	obs_reg.size_x = obsdim[0].data;
  	obs_reg.size_y = obsdim[1].data;
  	obs_reg.size_z = 0;

  	srv_rrts.request.WS = workspace;
  	srv_rrts.request.Goal = goal_reg;
  	srv_rrts.request.Init = init;
  	srv_rrts.request.Obstacles.push_back(obs_reg);
 
	/* Initialization */

	ROS_INFO("Command logic (master) node active");
	ROS_INFO("Initialization: calling spatial reasoner");

	// Set the matrices, by calling spatial reasoner
     	srv_spat.request.center = obs;
	srv_spat.request.dimensions.push_back(obsdim[0]);
	srv_spat.request.dimensions.push_back(obsdim[1]);

	if (cli_spat.call(srv_spat))
      	{
		for (int i=0;i<NZ*(RES+1)*(RES+1);i++)
			matrices[i].data = srv_spat.response.matrices[i].data;      	
		ROS_INFO("Environment landscapes generated succesfully");
	}
      	else
      	{
        	ROS_ERROR("Failed to call spatial reasoner");
        	return 1;
      	}

	/* Main loop */
	while(ros::ok())
	{
		/* Command: look */
   		if(cmd.data==1)
    		{ 
      			// First, call gesture processing service
			ROS_INFO("Calling Gesture Processing service");
		  	srv_gest.request.gesture = gesture;

      			if (cli_gest.call(srv_gest))
      			{
        			target = srv_gest.response.target;
				ROS_INFO("Target obtained: (%f,%f)", 
						target.x, target.y);
      			}
      			else
      			{
        			ROS_ERROR("Failed to call Gesture Processing");
        			return 1;
      			}

      			// Then, call pertinence mapping service
			ROS_INFO("Calling Pertinence Mapping service");
      			srv_pert.request.target = target;
			for (int i=0;i<srv_pert.request.matrices.size();i++)
			srv_pert.request.matrices.push_back(matrices[i]);

			if (cli_pert.call(srv_pert))
      			{
      				for (int i=0;i<(RES+1)*(RES+1);i++)
					landscape[i].data = 
					srv_pert.response.landscape[i].data; 
				ROS_INFO("Landscapes mapped");
			}
      			else
      			{
        			ROS_ERROR("Failed to call Pertinence Mapping");
        			return 1;
      			}

      			// After, call monte carlo service
			ROS_INFO("Calling Monte Carlo Simulation service");
			for (int i=0;i<srv_mont.request.landscape.size();i++)
			srv_mont.request.landscape.push_back(landscape[i]);

      			if (cli_mont.call(srv_mont))
      			{
				goal = srv_mont.response.goal;
 				ROS_INFO("Goal obtained: (%f,%f)",
						  goal.x, goal.y);
     
			}
      			else
      			{
        			ROS_ERROR("Failed to call Monte Carlo service");
        			return 1;
      			}

      			// Finally, call RRT* server and publish path
			ROS_INFO("Calling RRT* Path Planner service");
		      	if(cli_rrts.call(srv_rrts))
      			{
        			ROS_INFO("Path found: Publishing...");
				pathsize = srv_rrts.response.path.size();
        			for(int i=0; i<pathsize; i++)
        			{
          				path[i].x = srv_rrts.response.path[i].x;
          				path[i].y = srv_rrts.response.path[i].y;
          				path[i].z = srv_rrts.response.path[i].z;
          				path_pub.publish(path[i]);
					ROS_INFO("Point %d: (%f,%f)",
							i,path[i].x, path[i].y);
        			}
      			}
       			else
      			{
        			ROS_ERROR("Failed to call RRT* Path Planner");
        			return 1;
      			}
		
			// Reset command	
			cmd.data = 0;
    		}

		/* Command: go */
		if(cmd.data==2)
    		{ 
			// Enable robot control
      			enable.data = true;
      			flag_pub.publish(enable);
      		
			// Reset command
			cmd.data = 0;
    		}

		/* Command: stop */
	    	if(cmd.data==3) 
    		{
			// Disable robot control
      			enable.data = false;
      			flag_pub.publish(enable);
      
			// Reset command
			cmd.data = 0;
    		}

		/* Spin and wait for next period */
    		ros::spinOnce();
    		loop_rate.sleep();

	}

	return 0;

}
