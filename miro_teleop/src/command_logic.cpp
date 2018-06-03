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
#include "miro_teleop/Path.h"
#include "miro_msgs/platform_control.h"
#include "ros_cagg_msgs/cagg_tags.h"
#include "ros_cagg_msgs/cagg_tag.h"
#include "rrtstar_msgs/rrtStarSRV.h"
#include "rrtstar_msgs/Region.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

/* Definitions */
#define HSIZE 300 // Horizontal map size (in cm)
#define VSIZE 300 // Vertical map size (in cm)
#define RES 30 // Grid resolution
#define numObs 1 // Number of objects

/* Global variables */
geometry_msgs::Pose2D robot, user; // Robot and user positions from mocap
geometry_msgs::Pose gesture; // Gesture information from mocap
std::vector<geometry_msgs::Pose2D> obstacles(numObs); //Vector of obstacles
std::vector<geometry_msgs::Point> obsdim(numObs); // Vector of object dimensions (populated later manually)
std::vector<std::string> command_tag; // Command tags from CAGG
std::string command; // Command associated
ros_cagg_msgs::cagg_tags cmd; // Command tag received from speech recognition
// Relations and qualifiers - Please maintain order
std::string relationships[5] = {"RIGHT", "BEHIND", "LEFT", "FRONT", "NEAR"};
std::string qualifiers[3] = {"SLIGHTLY", "NORMAL", "EXACTLY"};
int taglength = -1; // Size of tags received from CAGG
bool useGesture = false;
double time_threshold = 3; // Max allowed time between gesture and speech
ros::Time lock_time, cmd_time, gesture_time; // Time stamps of the command and stable gesture
bool cmd_received = false; // Auxiliary flag to command
bool lock_param = false; // Auxiliary flag to prevent parameter change

/**
 * Subscriber callback function.
 * Obtains command tag from Interpreter node.
 */
void getCmd(const ros_cagg_msgs::cagg_tags::ConstPtr& msg)
{
	cmd.cagg_tags = msg->cagg_tags;
	ROS_INFO("Command received from interpreter");
	command_tag = cmd.cagg_tags[0].cagg_tag;
	if(command_tag.size()==0) 
	{	
		ROS_INFO("No tag found: command not understood");
		command = "";
	}	
	else
	{
		command = command_tag[0];
		ROS_INFO("Interpreted command: %s", command.c_str());
	 	
	}
	cmd_time = msg->header.stamp;
	// Detect when to use/ignore the gesture data and set useGesture accordingly
	ROS_INFO("Time between gesture and command: %f",abs((gesture_time-cmd_time).toSec()));
	useGesture = (abs((gesture_time-cmd_time).toSec())<time_threshold);
	cmd_received = true;
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
	gesture_time = pose->header.stamp;
}

/* Auxiliar function to obtain object information from mocap */
void addToObsList(const geometry_msgs::Pose2D::ConstPtr& groundpose, int i)
{
	geometry_msgs::Pose2D obs;
	obs.x = 100*groundpose->x;
	obs.y = 100*groundpose->y;
	obs.theta = groundpose->theta;
	obstacles[i-1]=obs;
}
/**
 * Subscriber callback functions:
 * Obtains object pose from Motion Capture node.
 */
void getObstacle1Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 1);
}
void getObstacle2Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 2);
}
void getObstacle3Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 3);
}
void getObstacle4Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 4);
}
void getObstacle5Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 5);
}
void getObstacle6Pose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	addToObsList(groundpose, 6);
}

/**
 * OpenCV Plot function.
 * Attaches matrix information to an img variable and displays it on screen.
 */
 
void plot(const char* name, float matrix[][RES])
{
	cv::Mat img;
	cv::Mat map(RES, RES, CV_32F, matrix);
	map.convertTo(img, CV_8UC1);
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, img);
	cv::waitKey(0);
}

/* Auxiliary function to generate kernels */
int generateLandscape(ros::ServiceClient cli_spat, ros::ServiceClient cli_pert,
miro_teleop::SpatialReasoner srv_spat, miro_teleop::PertinenceMapping srv_pert,
int state, std_msgs::Float64* landscape)
{
	std_msgs::Float64 kernel[RES*RES]; // Output kernel
	// Calling spatial_reasoner
	if (cli_spat.call(srv_spat))
	{
		ROS_INFO("I'm alive!!");
		float spmat0[RES][RES]; //For opencv plot
		for (int i=0; i<RES*RES; i++)
		{
			kernel[i].data = srv_spat.response.matrices[i].data;
			spmat0[i/RES][i%RES] = kernel[i].data*255;
		}
		// Display landscapes (requires opencv package)
		ROS_INFO("Landscape generated succesfully");
		plot("Relation Kernel", spmat0);
	}
	else
	{
		ROS_ERROR("Failed to call spatial reasoner");
		return -1;
	}
	srv_pert.request.M1.clear();
	srv_pert.request.M2.clear();
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
			// Plot using opencv
			plot("Mapped landscape", pertmatrix);
		}
	}
	else
	{
		ROS_ERROR("Failed to call Pertinence Mapping");
		return -1;
	}
	return state;

}

// Function to verify if some point (x,y) is inside any object from a given list
bool isIn(int i, int j)
{
	double cx, cy, dx, dy, x, y;
	x = HSIZE/double(2*RES)+HSIZE*(j/double(RES))-HSIZE/2;
	y = VSIZE/double(2*RES)+VSIZE*((RES-i-1)/double(RES))-VSIZE/2;
	for (int k = 0; k<obstacles.size(); k++)
	{
		cx = obstacles[k].x;
		cy = obstacles[k].y;
		dx = obsdim[k].x;
		dy = obsdim[k].y;
		if((x>(cx-dx/2))&&(x<(cx+dx/2))&&(y>(cy-dy/2))&&(y<(cy+dy/2)))
			return true;
	}
	return false;
}


//Initialize landscape[] by blacking out all obstacles in a white background
void initKernel(std_msgs::Float64* landscape)
{
	for (int i=0; i<RES; i++)
		for (int j=0; j<RES; j++)
			// If point is not inside any obstacle initial value is 1
			// i = row (y); j = column (x)
			if(!isIn(i,j)) landscape[i*RES+j].data = 1;
}

int main(int argc, char **argv)
{


	/* Definitions */
	geometry_msgs::Pose2D target, goal; // Gesture target and goal positions
	geometry_msgs::Vector3 init; // Initial position for the path planner
	miro_teleop::Path rrtPath; // Trajectory to be published
	std_msgs::Float64 landscape[RES*RES]; // Final landscape sent to MonteCarlo, updated after each command
	std_msgs::Bool enable; // Controller enable flag
	rrtstar_msgs::Region workspace, goal_reg; // For RRT* algorithm
	double pathsize; // Since RRT* trajectory size is variable
	int state = 0; // Control flag for the "look" command
	
	enable.data = false; // Robot control is initially off

	/* Initialize and assign node handler */
	ros::init(argc, argv, "command_logic");
	ros::NodeHandle n;

	// Initialize timing variables
	lock_time = ros::Time::now(); 
	cmd_time = ros::Time::now();  
	gesture_time = ros::Time::now(); 


	/* Initialize publishers and subscribers */
	// Publishers to robot controller
	ros::Publisher path_pub = n.advertise<miro_teleop::Path>("path", 1);
	ros::Publisher flag_pub = n.advertise<std_msgs::Bool>("enable", 1);

	// Subscriber from command interpreter
	ros::Subscriber sub_cmd = n.subscribe("CAGG/adapted/semantic_tags", 4, getCmd);

	// Subscribers from motion capture (mocap)
	ros::Subscriber sub_robot =	n.subscribe("Robot/ground_pose", 1, getRobotPose);
	ros::Subscriber sub_gesture = n.subscribe("stable_pose", 1, getGesture);
	ros::Subscriber sub_user =	n.subscribe("User/ground_pose", 1, getUserPose);
	ros::Subscriber sub_obs1 = n.subscribe("Obstacle1/ground_pose", 1, getObstacle1Pose);
	ros::Subscriber sub_obs2 = n.subscribe("Obstacle2/ground_pose", 1, getObstacle2Pose);
	ros::Subscriber sub_obs3 = n.subscribe("Obstacle3/ground_pose", 1, getObstacle3Pose);
	ros::Subscriber sub_obs4 = n.subscribe("Obstacle4/ground_pose", 1, getObstacle4Pose);
	ros::Subscriber sub_obs5 = n.subscribe("Obstacle5/ground_pose", 1, getObstacle5Pose);
	ros::Subscriber sub_obs6 = n.subscribe("Obstacle6/ground_pose", 1, getObstacle6Pose);

	/* Initialize service clients and handlers */
	ros::ServiceClient cli_spat = n.serviceClient<miro_teleop::SpatialReasoner>("spatial_reasoner");
	miro_teleop::SpatialReasoner srv_spat;
	ros::ServiceClient cli_gest = n.serviceClient<miro_teleop::GestureProcessing>("gesture_processing");
	miro_teleop::GestureProcessing srv_gest;
	ros::ServiceClient cli_pert = n.serviceClient<miro_teleop::PertinenceMapping>("pertinence_mapper");
	miro_teleop::PertinenceMapping srv_pert;
	ros::ServiceClient cli_mont = n.serviceClient<miro_teleop::MonteCarlo>("monte_carlo");
	miro_teleop::MonteCarlo srv_mont;
	ros::ServiceClient cli_rrts = n.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
	rrtstar_msgs::rrtStarSRV srv_rrts;

	/* Update rate (period) */
	ros::Rate loop_rate(10);

	/* Characterize workspace region for RRT* (predefined) */
	workspace.center_x = 0;
	workspace.center_y = 0;
	workspace.center_z = 0;
	workspace.size_x = HSIZE;
	workspace.size_y = VSIZE;
	workspace.size_z = 0;
	srv_rrts.request.WS = workspace; // RRT* request member

	/* TODO Define custom obstacle dimesnions */
	for(int i=0; i<numObs; i++)
	{
		// Set some default values
		obstacles[i].x = nan("");
		obstacles[i].y = nan("");
		obstacles[i].theta = nan("");

		obsdim[i].x=60;
		obsdim[i].y=60;
	}

	ros::spinOnce(); // So that obstacle list is populated

	// Initialize kernel
	initKernel(landscape);

	ROS_INFO("Command logic (master) node active");

	/*  Color convention: 0 = none, 1 = red, 2 = green, 3 = blue, 4 = yellow
	 *  Initially the robot will be able to listen to commands (green).
	 *  The CAGG node will trigger the 'processing' (yellow) light.
	 *  If it is not understood, the lights will become (red) for 2 secs.
	 *  Otherwise it'll be (blue) until robot starts moving.
	 *  In either case it returns to (green), accepting new commands.
	 *  When 'reset' is triggered, it turns (red) and blinks.
	 *  When 'stop' is triggered, it turns (blue) and blinks.
	 *  Inconsistent commands are treated as 'reset'.
	 */
	n.setParam("/color_key", 2); // Set initially to green (2)

	/* Main loop */
	while(ros::ok())
	{
		// We receive a list of relations at a time from Interpreter node
		// Parse each relation to extract "object#", "relationship", "qualifier"
		// Call spatial_reasoner once for each relation to generate one kernel each
		// Call pertinence mapping once for each command to get final landscape
		
		// Timing control of the lights
		if(abs((lock_time-ros::Time::now()).toSec())>3) 
		{
			lock_param = false;
			n.setParam("/color_key", 2);
		}
		// Obtain command associated and corresponding tag length
		if(cmd_received)
		{
			taglength = cmd.cagg_tags.size();
			if(taglength == 0) 	// Command not understood - set color red (1)
			{
				if (!lock_param) n.setParam("/color_key", 1);
				// Lock for 3 sec
				lock_param = true;
				lock_time = ros::Time::now();
			}
			else // Command understood - set color blue (3)
			{
				if (!lock_param) n.setParam("/color_key", 3);
			}
			cmd_received = false;
		}
		
		// Work based on command received when callback is executed
		// Reset: user not satisfied (aborted), Stop: user satisfied (done)
		if(command.compare("RESET") == 0) // Stop robot, clear kernel
		{
			enable.data = false;
			flag_pub.publish(enable);
			initKernel(landscape); // Reinitialize kernel
			n.setParam("/color_key", 5); // Set lights to purple (5)
			// Lock for 3 sec
			lock_param = true;
			lock_time = ros::Time::now();
			command = "";
		}
		else if(command.compare("STOP") == 0) // Stop robot, clear kernel
		{
			enable.data = false;
			flag_pub.publish(enable);
			initKernel(landscape); // Reinitialize kernel
			n.setParam("/color_key", 6); // Set lights to white (6)
			// Lock for 3 sec
			lock_param = true;
			lock_time = ros::Time::now();
			command = "";
		}
		else if(command.compare("GO") == 0 && taglength > 1) // Failsafe condition check
		{

			// Initial state
			state=0;
			
			// If a (meaningful) gesture is detected
			if(useGesture)
			{
				// Call gesture processing service
				ROS_INFO("Calling Gesture Processing service");
				srv_gest.request.gesture = gesture;
				if (cli_gest.call(srv_gest))
				{
					target = srv_gest.response.target;
					// Verify if target is valid number
					if(std::isfinite(target.x) && std::isfinite(target.y))
					{
						ROS_INFO("Target obtained: (%f,%f)", target.x, target.y);
						state = 1;
					}
					else
					{
						ROS_INFO("Invalid target: please try again");
					}
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
			
				//Set inputs for spatial_reasoner
				if(state==1)
				{
					srv_spat.request.center = target;
					srv_spat.request.dimx.data = 1;
					srv_spat.request.dimy.data = 1;
					srv_spat.request.relationship.data = 4; //relation "near"

					state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
					if(state == -1)
						return 1;
				}
				else state = 2;
	
				useGesture = false;
			}

			//We send data to spatial_reasoner even when gesture_processing fails
			//Sending info to spatial_reasoner as per Interpreter tags
			// Assuming tags in the form of
			// [["go","quantifier","strictly"](Optional),["go","relation","right"],["go","object","1"]
			int objid = atoi(cmd.cagg_tags[taglength-1].cagg_tag[2].c_str());
			ROS_INFO("Object id: %d",objid);
			srv_spat.request.center = obstacles[objid-1];
			srv_spat.request.dimx.data = obsdim[objid-1].x;
			srv_spat.request.dimy.data = obsdim[objid-1].y;
			for (int i=0; i<5; i++)
			{
				if(relationships[i].compare(cmd.cagg_tags[taglength-2].cagg_tag[2])==0)
				{
					srv_spat.request.relationship.data = i;
					ROS_INFO("Relation found: %s",relationships[i].c_str());					
					break;
				}
			}
			if(taglength>2)
			{
				for (int i=0; i<3; i++)
				{
					if(qualifiers[i].compare(cmd.cagg_tags[taglength-3].cagg_tag[2])==0)
					{
						srv_spat.request.qualifier.data = i;
						ROS_INFO("Qualifier: %s",qualifiers[i].c_str());	
						break;
					}
				}
			}

			srv_spat.request.user = user;
			ROS_INFO("User detected");
			state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
			if(state == -1)
				return 1;
			ROS_INFO("Kernel generated successfully");

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
						command = "RESET";
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
				srv_rrts.request.Obstacles.clear();
				/* Assign the obstacle region for RRT* */
				for (int i=0; i<numObs; i++)
				{
					if(!isnan(obstacles[i].x) || !isnan(obstacles[i].y))
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
				}

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
				// Reset tag length
				taglength = -1;
			 }

		 	n.setParam("/color_key", 2);
		 }
		 
		 
		 /* Spin and wait for next period */
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
	 return 0;
}
