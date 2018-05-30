/* Libraries */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "miro_teleop/GestureProcessing.h"
#include "miro_teleop/SpatialReasoner.h"
#include "miro_teleop/PertinenceMapping.h"
#include "miro_teleop/MonteCarlo.h"
#include "miro_msgs/platform_control.h"
#include "rrtstar_msgs/rrtStarSRV.h"
#include "rrtstar_msgs/Region.h"
#include <iostream>
#include <cmath>
#include <vector>
#include "miro_teleop/Path.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/* Definitions */
#define RES 40 // Grid resolution
#define HSIZE 400
#define VSIZE 400
#define numObs 3

/* Global variables */
ros_cagg_msgs::cagg_tags cmd; // Command tag received from interpreter
geometry_msgs::Pose2D robot, user; // Robot and user positions from mocap
vector<geometry_msgs::Pose2D> obstacles; //Vector of obstacles
vector<geometry_msgs::Point> obsdim; //Vector of object dimensions (populated later manually)
geometry_msgs::Pose gesture; // Gesture information from mocap
string[] relationships = {"right", "behind", "left", "front", "near"}; //Maintain order
string[] qualifiers = {"weak", "normal", "strong"};

/**
 * Subscriber callback function.
 * Obtains command tag from Interpreter node.
 */
void getCmd(const ros_cagg_msgs::cagg_tags::ConstPtr& msg)
{
	cmd.cagg_tags = msg->cagg_tags;
	ROS_INFO("Command received from interpreter");
}

/**
 * Subscriber callback function.
 * Obtains current robot pose from Motion Capture node.
 */
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	robot.x = 100*groundpose->x;
	robot.y = 100*groundpose->y;
	robot.theta = groundpose->theta;
}

/**
 * Subscriber callback function.
 * Obtains current user shoulder pose from Motion Capture node.
 */
void getUserPose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	user.x = 100*groundpose->x;
	user.y = 100*groundpose->y;
	user.theta = groundpose->theta;
}

/**
 * Subscriber callback function.
 * Obtains current gesture pose from Motion Capture node.
 */
void getGesture(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	gesture.orientation = pose->pose.orientation;
	gesture.position.x = 100*pose->pose.position.x;
	gesture.position.y = 100*pose->pose.position.y;
	gesture.position.z = 100*pose->pose.position.z;
}

/**
 * Subscriber callback function.
 * Obtains obstacle pose from Motion Capture node.
 */
void getObstaclePose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	geometry_msgs::Pose2D obs;
	obs.x = 100*groundpose->x;
	obs.y = 100*groundpose->y;
	obs.theta = groundpose->theta;
	obstacles.push_back(obs);
}

/**
 * OpenCV Plot function.
 * Attaches matrix information to an img variable and displays it on screen.
 */

/* Commented due to opencv issues - TODO Uncomment later
void plot(const char* name, float matrix[][RES])
{
	cv::Mat img;
	cv::Mat map(RES, RES, CV_32F, matrix);
	map.convertTo(img, CV_8UC1);
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, img);
	cv::waitKey(0);
}
*/

int generateLandscape(ros::ServiceClient cli_spat, ros::ServiceClient cli_pert,
	miro_teleop::SpatialReasoner srv_spat, miro_teleop::PertinenceMapping srv_pert,
	int state, std_msgs::Float64[] landscape)
{
	std_msgs::Float64[] kernel;
	//Call spatial_reasoner
	if (cli_spat.call(srv_spat))
	{
		float spmat0[RES][RES]; //For opencv plot
		for (int i=0; i<RES*RES; i++)
		{
			kernel[i].data = srv_spat.response.matrices[i].data;
			spmat0[i/RES][i%RES] = kernel[i].data*255;
		}
		// Display landscapes (requires opencv package)
		ROS_INFO("Gesture based landscape generated succesfully");
	}
	else
	{
		ROS_ERROR("Failed to call spatial reasoner");
		return 1;
	}

	//Setting inputs for pertinence map service
	for (int i=0; i<RES*RES; i++)
	{
		srv_pert.request.M1.push_back(landscape[i]);
		srv_pert.request.M2.push_back(kernel[i]);
	}

	//Calling pertinence_mapper
	if (cli_pert.call(srv_pert))
	{
		float pertmatrix[RES][RES]; //For opencv
		for (int i=0; i<RES*RES; i++)
		{
			landscape[i].data = srv_pert.response.MOut[i].data;
			pertmatrix[i/RES][i%RES] = landscape[i].data*255;
		}
		// Verify whether the output is valid
		if(!std::isfinite(landscape[0].data))
		{
			state = 0;
			ROS_INFO("Invalid pertinence mapping");
		}
		else
		{
			state = 2;
			ROS_INFO("Gesture based landscape mapped");
			// Plot using opencv (TODO Uncomment)
			//plot("Mapped landscape", pertmatrix);
		}
	}
	else
	{
		ROS_ERROR("Failed to call Pertinence Mapping");
		return 1;
	}
	srv_pert.request.M1.clear();
	srv_pert.request.M2.clear();
	return state;
}

int main(int argc, char **argv)
{
	/* Definitions */
	geometry_msgs::Pose2D gesture_target, goal; // Gesture target and goal positions
	miro_teleop::Path rrtPath; // Trajectory to be published
	std_msgs::Float64 landscape[RES*RES]; // Final landscape sent to MonteCarlo, updated after each command
	std_msgs::Bool enable; // Controller enable flag
	rrtstar_msgs::Region workspace, goal_reg; // For RRT* algorithm
	geometry_msgs::Vector3 init; // Initial position for the path planner
	double pathsize; // Since RRT* trajectory size is variable
	int state = 0; // Control flag for the "look" command

	enable.data = false;

	/* TODO Define custom obstacle dimesnions*/
	for(int i=0; i<numObs; i++)
	{
		geometry_msgs::Point dim;
		dim.x=60;
		dim.y=60;
		obsdim.push_back(dim);
	}

	/* Initialize and assign node handler */
	ros::init(argc, argv, "command_logic");
	ros::NodeHandle n;

	/* Initialize publishers and subscribers */
	// Publishers to robot controller
	ros::Publisher path_pub =
	n.advertise<miro_teleop::Path>("path", 1);
	ros::Publisher flag_pub =
	n.advertise<std_msgs::Bool>("enable", 1);

	// Subscriber from command interpreter
	ros::Subscriber sub_cmd =	n.subscribe("command", 4, getCmd);
	// Subscribers from motion capture (mocap)
	ros::Subscriber sub_robot =	n.subscribe("Robot/ground_pose", 1, getRobotPose);
	ros::Subscriber sub_gesture =	n.subscribe("Gesture/pose", 1, getGesture);
	ros::Subscriber sub_user =	n.subscribe("User/ground_pose", 1, getUserPose);
	vector<ros::Subscriber> sub_obs;
	for (int i=0; i<numObs; i++)
	{
		sub_obs.push_back(n.subscribe("Obstacle"+i+"/ground_pose", 1, getObstaclePose));
	}

	/* Initialize service clients and handlers */
	ros::ServiceClient cli_spat =	n.serviceClient<miro_teleop::SpatialReasoner>("spatial_reasoner");
	miro_teleop::SpatialReasoner srv_spat;

	ros::ServiceClient cli_gest =	n.serviceClient<miro_teleop::GestureProcessing>("gesture_processing");
	miro_teleop::GestureProcessing srv_gest;

	ros::ServiceClient cli_pert =	n.serviceClient<miro_teleop::PertinenceMapping>("pertinence_mapper");
	miro_teleop::PertinenceMapping srv_pert;

	ros::ServiceClient cli_mont =	n.serviceClient<miro_teleop::MonteCarlo>("monte_carlo");
	miro_teleop::MonteCarlo srv_mont;

	ros::ServiceClient cli_rrts =	n.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
	rrtstar_msgs::rrtStarSRV srv_rrts;

	/* Update rate (period) */
	ros::Rate loop_rate(10);

	/* Characterize workspace region (predefined) */
	workspace.center_x = 0;
	workspace.center_y = 0;
	workspace.center_z = 0;
	workspace.size_x = HSIZE;
	workspace.size_y = VSIZE;
	workspace.size_z = 0;

	srv_rrts.request.WS = workspace; // RRT* request member

	/* Assuming static objects, assign the obstacle region only once */
	for (int i=0; i<numObs; i++)
	{
		rrtstar_msgs::Region obs_reg;
		obs_reg.center_x = obstacles[i].x;
		obs_reg.center_y = obstacles[i].y;
		obs_reg.center_z = 0;
		obs_reg.size_x = obsdim[i].x;
		obs_reg.size_y = obsdim[i].y;
		obs_reg.size_z = 0;
		srv_rrts.request.Obstacles.push_back(obs_reg); // RRT* request member
	}

	ROS_INFO("Command logic (master) node active");
	ROS_INFO("Initialization: calling spatial reasoner");

	//TODO: Here initialize landscape[] by blacking out all obstacles in a white background

	/* Main loop */
	while(ros::ok())
	{
		// We receive a list of relations at a time from Interpreter node
		// Parse each relation to extract "object#", "relationship", "qualifier"
		// Call spatial_reasoner once for each relation to generate one kernel each
		// Call pertinence mapping once for each command to get final landscape

		int taglength=cmd.cagg_tags.size();
		if("reset".equalsIgnoreCase(cmd.cagg_tags[0][0]))
		{
			enable.data = false;
			flag_pub.publish(enable);
		}
		else if("stop".equalsIgnoreCase(cmd.cagg_tags[0][0]))
		{
			enable.data = false;
			flag_pub.publish(enable);
		}
		else if("go".equalsIgnoreCase(cmd.cagg_tags[0][0]) && taglength>1) //Failsafe condition check
		{
			//TODO: Stop Miro's current motion while new path is being computed
			// Or some feedback to show that command is being processed

			// Call gesture processing service
			state=0;
			ROS_INFO("Calling Gesture Processing service");
			srv_gest.request.gesture = gesture;
			if (cli_gest.call(srv_gest))
			{
				target = srv_gest.response.target;
				// Verify if target is valid number
				if(std::isfinite(target.x) &&	std::isfinite(target.y))
				{
					ROS_INFO("Target obtained: (%f,%f)", target.x, target.y);
					state = 1;
				}
				else
					ROS_INFO("Invalid target: please try again");
				// Verify bound conditions
				if(target.x < -HSIZE/2 || target.x > HSIZE/2 || target.y < -VSIZE/2 || target.y > VSIZE/2)
				{
					ROS_INFO("Target out of the bounds");
					state = 0;
				}
			}
			else
			{
				ROS_ERROR("Failed to call Gesture Processing");
				return 1;
			}
			//TODO detect when to use/ignore the gesture data and set useGesture accordingly

			bool useGesture = true;
			if(useGesture)
			{
				//Set inputs for spatial_reasoner
				srv_spat.request.center = target;
				srv_spat.request.dimensions[0] = 1;
				srv_spat.request.dimensions[1] = 1;
				srv_spat.request.relationship = 4; //relation "near"

				state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
			}

			//We send data to spatial_reasoner even when gesture_processing fails
			//Sending info to spatial_reasoner as per Interpreter tags
			// Assuming tags in the form of
			// [["go","quantifier","strictly"](Optional),["go","relation","right"],["go","object","1"]
			int objid = cmd.cagg_tags[taglength-1][2];
			srv_spat.request.center = obstacles[objid-1];
			srv_spat.request.dimensions[0] = obsdim[objid-1].x;
			srv_spat.request.dimensions[1] = obsdim[objid-1].y;
			for (int i=0; i<relationships.size(); i++)
			{
				if(relationships[i].equalsIgnoreCase(cmd.cagg_tags[taglength-2][2]))
				{
					srv_spat.request.relationship = i;
					break;
				}
			}
			if(taglength>2)
			{
				for (int i=0; i<qualifiers.size(); i++)
				{
					if(qualifiers[i].equalsIgnoreCase(cmd.cagg_tags[taglength-3][2]))
					{
						srv_spat.request.qualifier = i;
						break;
					}
				}
			}

			state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);

			if(state==2)
			{
				//Set input for Monte MonteCarlo
				for (int i=0; i<RES*RES; i++)
					srv_mont.request.landscape.push_back(landscape[i]);

				//Call MonteCarlo
				if (cli_mont.call(srv_mont))
				{
					goal = srv_mont.response.goal;
					// Verify if goal returned is valid
					if(goal.x<-HSIZE/2 || goal.x>HSIZE/2 || goal.y<-VSIZE/2 || goal.y>VSIZE/2)
					{
						ROS_INFO("Invalid goal position");
						state = 0;
					}
					else
					{
						ROS_INFO("Goal obtained: (%f,%f)", goal.x, goal.y);
						state = 3;
					}
				}
				else
				{
					ROS_ERROR("Failed to call Monte Carlo service");
					return 1;
				}
				srv_mont.request.landscape.clear();
			}

			// Finally, call RRT* server and publish path
			if(state==3)
			{
				ROS_INFO("Calling RRT* Path Planner service");

				// Initial position is robot current one
				init.x = robot.x;
				init.y = robot.y;
				init.z = 0;

				// Define goal region
				goal_reg.center_x = goal.x;
				goal_reg.center_y = goal.y;
				goal_reg.center_z = 0;
				goal_reg.size_x = 20;
				goal_reg.size_y = 20;
				goal_reg.size_z = 0;

				// Note: workscape and object regions already defined
				srv_rrts.request.Goal = goal_reg;
				srv_rrts.request.Init = init;

				if(cli_rrts.call(srv_rrts))
				{
					ROS_INFO("Path found: Publishing...");
					pathsize = srv_rrts.response.path.size();
					// Obtain trajectory point-by-point
					geometry_msgs::Vector3 point;
					rrtPath.path.clear();
					for(int i=0; i<pathsize; i++)
					{
						point.x = srv_rrts.response.path[i].x;
						point.y = srv_rrts.response.path[i].y;
						point.z = srv_rrts.response.path[i].z;
						// Only x and y coordinates matter
						rrtPath.path.push_back(point);
						ROS_INFO("Point %d: (%f,%f)", i, point.x, point.y);
					}
					path_pub.publish(rrtPath);
					state = 4;
				}
				else
				{
					ROS_ERROR("Failed to call RRT* Path Planner");
					return 1;
				}
				// Enable robot control
				enable.data = true;
				flag_pub.publish(enable);
			 }
		 }
		 /* Spin and wait for next period */
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
	 return 0;
 }
