/* Libraries */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "miro_teleop/SpatialReasoner.h"
#include "miro_teleop/PertinenceMapping.h"
#include "miro_teleop/MonteCarlo.h"
#include "miro_teleop/Path.h"
#include "miro_msgs/platform_control.h"
#include "ros_cagg_msgs/cagg_tags.h"
#include "rrtstar_msgs/rrtStarSRV.h"
#include "rrtstar_msgs/Region.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <highgui.h>
#include <opencv/cv.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


/* Definitions */
#define HSIZE 240 // Horizontal map size (in cm)
#define VSIZE 240 // Vertical map size (in cm)
#define RES 24 // Grid resolution
#define numObs 4 // Number of objects
#define GOAL_SIZE 5
#define PI 3.14159

/* Global variables */
geometry_msgs::Pose2D robot, user, gesture; // Robot, user and (stable) gesture positions from mocap
std::vector<geometry_msgs::Pose2D> obstacles(numObs+1); //Vector of obstacles
std::vector<geometry_msgs::Point> obsdim(numObs+1); // Vector of object dimensions (populated later manually)
std::vector<std::string> command_tag; // Command tags from CAGG
std::string command; // Command associated
ros_cagg_msgs::cagg_tags cmd; // Command tag received from speech recognition
std::string relationships[5] = {"RIGHT", "BEHIND", "LEFT", "FRONT", "NEAR"}; // Relations - maintain order
std::string qualifiers[3] = {"SLIGHTLY", "NORMAL", "EXACTLY"}; // Qualifiers - maintain order
ros::Time lock_time, cmd_time, gesture_time; // Time stamps associated with flags
bool useGesture = false, plot_gesture = false, cmd_received = false, lock_param = false, kernel_initialized = false; // Auxiliary flags
double time_threshold = 8; // Max allowed time between gesture and speech (in sec.)
int cmd_count = 0, num_ker = 0; // For logging purposes: command counter in session and number of kernels merged
bool writtenOnLog = false; // For auxiliary log writing fun
ros::Time gestproc_time, mapping_time, montecarlo_time, rrtstar_time;
std::string base_path = "/home/emarolab/ros_ws/src/mmodal_teleop/miro_teleop/log/";
std::string path_to_img = base_path + "img/";

/* Auxiliary functions to convert from number to string */
std::string int_to_str(int num)
{
	 return static_cast<std::ostringstream*>(&(std::ostringstream()<<num))->str();
}
std::string double_to_str(double num)
{
	 return static_cast<std::ostringstream*>(&(std::ostringstream()<<num))->str();
}

/* Logging functions */
const std::string getData(){
	// Returns the current date and time formatted as %Y-%m-%d_%H.%M.%S
	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
	std::string currentDate(mbstr);
	return currentDate;
}

std::string printPath = base_path + "session_" + getData() + ".log"; // Std path to logfile
void writeStrToFile( const std::string &toWrite){
	// Function to write to file a single string
        std::string formattedTime = getData();
	std::ofstream file;
	file.open(printPath.c_str(), std::ofstream::out | std::ofstream::app);
        file << formattedTime << ": " << toWrite << "\n";
	file.close();
}

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
	cmd_received = true; // Set flag to true
}


/* Auxiliar function to obtain object information from mocap */
void addToObsList(const geometry_msgs::Pose2D::ConstPtr& groundpose, int i)
{
	geometry_msgs::Pose2D obs;
	obs.x = 100*groundpose->x;
	obs.y = 100*groundpose->y;
	obs.theta = groundpose->theta;
	obstacles[i-1] = obs;
	//ROS_INFO("FROM MOCAP Obj %d position obtained (%f, %f)",i,obstacles[i-1].x,obstacles[i-1].y);
}

/**
 * Subscriber callback function.
 * Obtains current robot 2D pose from Motion Capture node.
 */
void getRobotPose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	robot.x = 100*groundpose->x;
	robot.y = 100*groundpose->y;
	robot.theta = groundpose->theta;
}

/**
 * Subscriber callback function.
 * Obtains current user body 2D pose from Motion Capture node.
 */
void getUserPose(const geometry_msgs::Pose2D::ConstPtr& groundpose)
{
	// NOTE: Conversion to [cm] is done on addToObsList
	user.x = 100*groundpose->x;
	user.y = 100*groundpose->y;
	user.theta = groundpose->theta;
	addToObsList(groundpose, numObs+1);
}

/**
 * Subscriber callback function.
 * Obtains current gesture position from tracker node.
 */
void getGesture(const geometry_msgs::PointStamped::ConstPtr& gest)
{
	// NOTE: Tracker already gives data in [cm]
	gesture.x = gest->point.x;
	gesture.y = gest->point.y;
	gesture_time = gest->header.stamp;
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

/* Function to verify if some point (x,y) is inside any object from a given list */
bool isIn(int i, int j)
{
	double cx, cy, dx, dy, x, y;
	x = HSIZE/double(2*RES)+HSIZE*(j/double(RES))-HSIZE/2;
	y = VSIZE/double(2*RES)+VSIZE*((RES-i-1)/double(RES))-VSIZE/2;
	for (int k = 0; k<numObs+1; k++)
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

void plotKernel(const char* name, float matrix[][RES])
{
        int x, y, user_x, user_y, dest_x_img, dest_y_img;
        float dest_x_real, dest_y_real, L=50;
        // Create image and associate it to matrix
        cv::Mat img, fin_img;
        cv::Mat map(RES, RES, CV_32F, matrix);
        map.convertTo(img, CV_8UC1);
        std::vector<cv::Mat> channels;
        //cv::Mat g = cv::Mat::zeros(cv::Size(img.rows, img.cols), CV_8UC1);
        channels.push_back(img);
        channels.push_back(img);
        channels.push_back(img);
        cv::merge(channels, fin_img);

        //Plot user orientation
        user_x = (int)((user.x-VSIZE/double(2*RES)+VSIZE/2)/VSIZE*double(RES))*800/double(RES);
        user_y = (RES-(int)((user.y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1)*800/double(RES);
        dest_x_real=user.x+L*cos(user.theta+PI/2);
        dest_y_real=user.y+L*sin(user.theta+PI/2);
        dest_x_img=(int)((dest_x_real-VSIZE/double(2*RES)+VSIZE/2)/VSIZE*double(RES))*800/double(RES);
        dest_y_img=(RES-(int)((dest_y_real-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1)*800/double(RES);

	//Plot objects (blue)
	for(int i=0; i<RES; i++)
		for(int j=0; j<RES; j++)
			if(isIn(j,i))
				fin_img.at<cv::Vec3b>(cv::Point(i,j)) = cv::Vec3b(128,128,0);

	

        if(useGesture)
        {
                // Plot gesture as a purple point
                x = (int)((gesture.x-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES));
                y = RES-(int)((gesture.y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1;
                fin_img.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,255);
		/* Display image
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, fin_img);
		cv::waitKey(0);
		*/
		// Save image on a file
		cv::Mat resized_img;
		cv::resize(fin_img, resized_img, cv::Size(800, 800), 1, 1, cv::INTER_NEAREST);
		cv::arrowedLine(resized_img, cv::Point(user_x, user_y),cv::Point(dest_x_img, dest_y_img), CV_RGB(255, 255, 0), 3, 8, 0, 0.1);		
		std::string cmd_num = int_to_str(cmd_count);
		std::string path_img = path_to_img+"plot_"+getData()+"gesture_cmd_"+cmd_num+".jpg";
		cv::imwrite(path_img, resized_img);
		plot_gesture = true;
        }
	else
	{
		/* Display image
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, fin_img);
		cv::waitKey(0);
		*/
		// Save image on a file
		cv::Mat resized_img;
		cv::resize(fin_img, resized_img, cv::Size(800, 800), 1, 1, cv::INTER_NEAREST);
		cv::arrowedLine(resized_img, cv::Point(user_x, user_y),cv::Point(dest_x_img, dest_y_img), CV_RGB(255, 255, 0), 3, 8, 0, 0.1);		
		std::string cmd_num = int_to_str(cmd_count);
		std::string path_img = path_to_img+"plot_"+getData()+"kernel_cmd_"+cmd_num+".jpg";
		cv::imwrite(path_img, resized_img);
	}
}


/**
 * OpenCV Plot function.
 * Attaches matrix information to an img variable and displays it on screen.
 */ 
void plotPertinence(const char* name, float matrix[][RES], bool plotGoalandPath, geometry_msgs::Pose2D goal, miro_teleop::Path rrtPath)
{	
        int x, y, user_x, user_y, dest_x_img, dest_y_img;
        float dest_x_real, dest_y_real, L=50;
	// Create image and associate it to matrix
	cv::Mat img, fin_img;
	cv::Mat map(RES, RES, CV_32F, matrix);
	map.convertTo(img, CV_8UC1);
	std::vector<cv::Mat> channels;
    	//cv::Mat g = cv::Mat::zeros(cv::Size(img.rows, img.cols), CV_8UC1);
    	channels.push_back(img);
    	channels.push_back(img);
    	channels.push_back(img);
	cv::merge(channels, fin_img);

        //Plot user orientation
        user_x = (int)((user.x-VSIZE/double(2*RES)+VSIZE/2)/VSIZE*double(RES))*800/double(RES);
        user_y = (RES-(int)((user.y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1)*800/double(RES);
        dest_x_real=user.x+L*cos(user.theta+PI/2);
        dest_y_real=user.y+L*sin(user.theta+PI/2);
        dest_x_img=(int)((dest_x_real-VSIZE/double(2*RES)+VSIZE/2)/VSIZE*double(RES))*800/double(RES);
        dest_y_img=(RES-(int)((dest_y_real-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1)*800/double(RES);
        

	//Plot objects (blue)
	for(int i=0; i<RES; i++)
		for(int j=0; j<RES; j++)
			if(isIn(j,i))
				fin_img.at<cv::Vec3b>(cv::Point(i,j)) = cv::Vec3b(128,128,0);

	
        if(plot_gesture)
        {
                // Plot gesture as a purple point
                x = (int)((gesture.x-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES));
                y = RES-(int)((gesture.y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1;
                fin_img.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,255);
		plot_gesture = false;
	}

	if(plotGoalandPath)
	{
                for (int j=0; j<rrtPath.path.size(); j++)
                {
                    x = (int)((rrtPath.path[j].x-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES));
                    y = RES-(int)((rrtPath.path[j].y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1;
                    fin_img.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,255,0);
                }
                x = (int)((goal.x-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES));
                y = RES-(int)((goal.y-HSIZE/double(2*RES)+HSIZE/2)/HSIZE*double(RES))-1;
		fin_img.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);
	}

	/* Display image
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, fin_img);
	cv::waitKey(0);
	*/
	
	// Save image on a file
	cv::Mat resized_img;
        cv::resize(fin_img, resized_img, cv::Size(800, 800), 1, 1, cv::INTER_NEAREST);
	cv::arrowedLine(resized_img, cv::Point(user_x, user_y),cv::Point(dest_x_img, dest_y_img), CV_RGB(255, 255, 0), 3, 8, 0, 0.1);

	std::string cmd_num = int_to_str(cmd_count);
	std::string path_img = path_to_img+"plot_"+getData()+"mapping_cmd_"+cmd_num+".jpg";
	cv::imwrite(path_img, resized_img);
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
		float spmat0[RES][RES]; //For opencv plot
		for (int i=0; i<RES*RES; i++)
		{
			kernel[i].data = srv_spat.response.matrices[i].data;
			spmat0[i/RES][i%RES] = kernel[i].data*255;
		}
		// Display landscapes (requires opencv package)
		ROS_INFO("Landscape generated succesfully");
                plotKernel("Relation Kernel", spmat0);
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
			//pertmatrix[i/RES][i%RES] = landscape[i].data*255;
		}
		// Verify whether the output is valid
		if(!std::isfinite(landscape[0].data))
		{
			state = -1;
			ROS_INFO("Invalid pertinence mapping");
		}
		else
		{
			state = 2;
			ROS_INFO("Landscape mapped");
		}
	}
	else
	{
		ROS_ERROR("Failed to call Pertinence Mapping");
		return -1;
	}
	return state;

}



/* Initialize landscape[] by blacking out all obstacles in a white background */
void initKernel(std_msgs::Float64* landscape)
{
	for (int i=0; i<numObs+1; i++) // DEBUG
	{
		ROS_INFO("\nObstacles ... %d: (%f, %f)",i,obstacles[i].x, obstacles[i].y);
	}

	for (int i=0; i<RES; i++)
		for (int j=0; j<RES; j++)
			// If point is not inside any obstacle initial value is 1
			// i = row (y); j = column (x)
			if(!isIn(i,j)) landscape[i*RES+j].data = 1;
	kernel_initialized = true;
}

// Auxiliary function to check if object data was sent by mocap
bool obstaclesPlaced()
{
	for (int k = 0; k<numObs+1; k++)
	{
		if(isnan(obstacles[k].x) || isnan(obstacles[k].y)) 
			return false;
	}
	return true;
}

// Auxiliary function to write initial setting and header on Log
void writeOnLog()
{
	writeStrToFile(",EXPERIMENT SETTING");
	std::string setting_txt = ",Workspace size: ("+int_to_str(HSIZE)+"  "+int_to_str(VSIZE)+"), Resolution: "+int_to_str(RES)+", Num. of objects: "+int_to_str(numObs);
	writeStrToFile(setting_txt);
	for (int i=0; i<numObs; i++)
	{
		std::string obj_txt = ",Object "+int_to_str(i+1)+" located at ("+double_to_str(obstacles[i].x)+";"+
			double_to_str(obstacles[i].y)+")    Size: ("+double_to_str(obsdim[i].x)+"  "+double_to_str(obsdim[i].y)+")";
		writeStrToFile(obj_txt);
	}
	writeStrToFile(",DATA HEADER");
	writeStrToFile(",Command #, Type, Num. Kernels, Relation, Object ID, Qualifier, Pointing, User Orientation, Initial Pose, Final Pose, Fuzzy Pertinence, Path Found, Gest. Proc. Time, Mapping Time, Montecarlo Time, RRT* Time");

	writtenOnLog = true; 
}

/* Main function */

int main(int argc, char **argv)
{

	/* Definitions */
	geometry_msgs::Vector3 init; // Initial position for the path planner
	miro_teleop::Path rrtPath; // Trajectory to be published
	std_msgs::Float64 landscape[RES*RES]; // Final landscape sent to MonteCarlo, updated after each command
	std_msgs::Bool enable; // Controller enable flag
	rrtstar_msgs::Region workspace, goal_reg; // For RRT* algorithm
	double pathsize; // Since RRT* trajectory size is variable
	int state = 0; // Control flag for the "look" command
	geometry_msgs::Pose2D goal; // Goal position
	
	enable.data = false; // Robot control is initially off

	/* Initialize and assign node handler */
	ros::init(argc, argv, "command_logic");
	ros::NodeHandle n;

	n.param("path_to_pkg", base_path);

	// Plot images on a specific folder within img/
	path_to_img = base_path + "img/session_" + getData() + "/";
	try 
	{
		boost::filesystem::path dirPath(path_to_img);
		boost::filesystem::create_directories(dirPath.parent_path());
	}
	catch(const boost::filesystem::filesystem_error& err) 
	{
		std::cerr << err.what() << std::endl;
	}

	// Initialize time variables
	lock_time = ros::Time::now(); 
	cmd_time = ros::Time::now();  
	gesture_time = ros::Time(0); 

	/* Initialize publishers and subscribers */
	// Publishers to robot controller
	ros::Publisher path_pub = n.advertise<miro_teleop::Path>("path", 1);
	ros::Publisher flag_pub = n.advertise<std_msgs::Bool>("enable", 1);

	// Subscriber from command interpreter
	ros::Subscriber sub_cmd = n.subscribe("CAGG/adapted/semantic_tags", 4, getCmd);

	// Subscribers from motion capture (mocap)
	ros::Subscriber sub_robot = n.subscribe("Robot/ground_pose", 1, getRobotPose);
	ros::Subscriber sub_user = n.subscribe("User/ground_pose", 1, getUserPose);
	ros::Subscriber sub_obs1 = n.subscribe("Obstacle1/ground_pose", 1, getObstacle1Pose);
	ros::Subscriber sub_obs2 = n.subscribe("Obstacle2/ground_pose", 1, getObstacle2Pose);
	ros::Subscriber sub_obs3 = n.subscribe("Obstacle3/ground_pose", 1, getObstacle3Pose);
	ros::Subscriber sub_obs4 = n.subscribe("Obstacle4/ground_pose", 1, getObstacle4Pose);
	ros::Subscriber sub_obs5 = n.subscribe("Obstacle5/ground_pose", 1, getObstacle5Pose);
	ros::Subscriber sub_obs6 = n.subscribe("Obstacle6/ground_pose", 1, getObstacle6Pose);

	// Subscriber from tracking node
	ros::Subscriber sub_gesture = n.subscribe("/stable_gndpose", 1, getGesture);

	/* Initialize service clients and handlers */
	ros::ServiceClient cli_spat = n.serviceClient<miro_teleop::SpatialReasoner>("spatial_reasoner");
	miro_teleop::SpatialReasoner srv_spat;
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

	/* Initialize object variables */
	for(int i=0; i<numObs+1; i++)
	{
		// Set nan as default values
		obstacles[i].x = nan("");
		obstacles[i].y = nan("");
		obstacles[i].theta = nan("");
                // Define custom object dimesnions (already considering robot size)
                obsdim[i].x=50;
                obsdim[i].y=50;
	}
                // Redefine user dimensions (already considering robot size)
		obsdim[numObs].x=5;
		obsdim[numObs].y=5;
	
	//ros::spinOnce(); // So that obstacle list is populated
	/*
	// Initialize kernel
	
	ROS_INFO("Obstacle %d position obtained (%f, %f)",1,obstacles[0].x,obstacles[0].y);
	initKernel(landscape);
	ROS_INFO("Obstacle %d position obtained (%f, %f)",1,obstacles[0].x,obstacles[0].y);
	*/

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
		if(obstaclesPlaced())
		{

			if(!writtenOnLog) writeOnLog();			

			int tagslength = cmd.cagg_tags.size();

			// Redefine log text variables
			std::string cmd_txt = " ";
			std::string ker_txt = " ";
			std::string relation_txt = " ";
			std::string objid_txt = " ";
			std::string qualifier_txt = " ";
			std::string pointing_txt = " ";
			std::string user_txt = " ";
			std::string initpose_txt = " ";
			std::string finalpose_txt = " ";
			std::string pert_txt = " ";
			std::string path_found = " ";
			std::string gestproc_dur = " ";
			std::string mapping_dur = " "; 
			std::string montecarlo_dur = " "; 
			std::string rrtstar_dur = " "; 

			// We receive a list of relations at a time from Interpreter node
			// Parse each relation to extract "object#", "relationship", "qualifier"
			// Call spatial_reasoner once for each relation to generate one kernel each
			// Call pertinence mapping once for each command to get final landscape
		
			// Timing control of the lights
			if(lock_param && abs((lock_time-ros::Time::now()).toSec())>3) 
			{
				lock_param = false;
				n.setParam("/color_key", 2);
			}
			// Obtain command associated and corresponding tag length
			if(cmd_received)
			{
				// Increment command counter and associated string
				cmd_count++;
				cmd_txt = int_to_str(cmd_count);

				int firstTaglength = cmd.cagg_tags[0].cagg_tag.size();
				ROS_INFO("firstTaglength: %d",firstTaglength);
				if(firstTaglength == 0) 	// Command not understood - set color red (1)
				{
					if (!lock_param)
					{ 
						n.setParam("/color_key", 1);
						lock_param = true;
					}				
					// Lock for 3 sec
					lock_time = ros::Time::now();
					command = "";
				}
				else // Command understood - set color blue (3)
				{
					if (!lock_param) n.setParam("/color_key", 3);
				}
			}
		
			if(state==-1)
			{ 
				command = "RESET";
				state = 0;
			}

			// Work based on command received when callback is executed
			// Reset: user not satisfied (aborted), Stop: user satisfied (done)
			if(command.compare("RESET") == 0) // Stop robot, clear kernel
			{
				enable.data = false;
				flag_pub.publish(enable);
				num_ker = 0;
				initKernel(landscape); // Reinitialize kernel
				n.setParam("/color_key", 5); // Set lights to purple (5)
				// Lock for 3 sec
				lock_param = true;
				lock_time = ros::Time::now();
				// Reset command
				command = "";
			}
			else if(command.compare("STOP") == 0) // Stop robot, clear kernel
			{
				enable.data = false;
				flag_pub.publish(enable);
				num_ker = 0;
				initKernel(landscape); // Reinitialize kernel
				n.setParam("/color_key", 6); // Set lights to white (6)
				// Lock for 3 sec
				lock_param = true;
				lock_time = ros::Time::now();
				command = "";
			}
			else if(command.compare("GO") == 0 && tagslength > 1) // Failsafe condition check
			{
				num_ker++; // Increase kernel merge count
				// Disable movement whenever a new command is issued
				enable.data = false;
				flag_pub.publish(enable);
				
				// If it is not initialized, initalize it
				if(!kernel_initialized) initKernel(landscape);

				// Initial state
				state = 0;		
				plot_gesture = false;	
			
				int objid = atoi(cmd.cagg_tags[tagslength-1].cagg_tag[2].c_str());

				// If a (stable) gesture is detected
				ROS_INFO("%d", useGesture);
		                if(useGesture)
				{	
					ROS_INFO("Valid gesture detected at (%f, %f)", gesture.x, gesture.y);
					pointing_txt = "("+double_to_str(gesture.x)+"  "+double_to_str(gesture.y)+")";
					srv_spat.request.center = gesture;
					srv_spat.request.dimx.data = 1;
					srv_spat.request.dimy.data = 1;
					srv_spat.request.relationship.data = 4; // Relation: "near"
					state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
					useGesture = false;

				}
				else
				{
					ROS_INFO("No valid gesture detected");
					srv_spat.request.center = obstacles[objid-1];
					pointing_txt = "Not detected";
					// Relation: "near" FOR OBJECT
					srv_spat.request.dimx.data = 1;
					srv_spat.request.dimy.data = 1;
					srv_spat.request.relationship.data = 4;
					state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
				}	
				
				gestproc_time = ros::Time::now();			
				gestproc_dur = double_to_str((gestproc_time-cmd_time).toSec());

				if(state == -1) return 1;
				else state = 2;


				//We send data to spatial_reasoner even when gesture_processing fails
				//Sending info to spatial_reasoner as per Interpreter tags
				// Assuming tags in the form of
				// [["go","quantifier","strictly"](Optional),["go","relation","right"],["go","object","1"]
				
				ROS_INFO("Object id: %d",objid);
				objid_txt = int_to_str(objid);
				srv_spat.request.center = obstacles[objid-1];
				srv_spat.request.dimx.data = obsdim[objid-1].x;
				srv_spat.request.dimy.data = obsdim[objid-1].y;
				for (int i=0; i<5; i++)
				{
					if(relationships[i].compare(cmd.cagg_tags[tagslength-2].cagg_tag[2])==0)
					{
						srv_spat.request.relationship.data = i;
						ROS_INFO("Relation found: %s",relationships[i].c_str());
						relation_txt = relationships[i].c_str();	
						break;
					}
				}

				qualifier_txt = "NORMAL"; // Default qualifier if none is said
				if(tagslength>2)
				{
					for (int i=0; i<3; i++)
					{
						if(qualifiers[i].compare(cmd.cagg_tags[tagslength-3].cagg_tag[2])==0)
						{
							srv_spat.request.qualifier.data = i;
							ROS_INFO("Qualifier: %s",qualifiers[i].c_str());	
							qualifier_txt = qualifiers[i].c_str();				
							break;
						}
					}
				}

				srv_spat.request.user = user;
				user_txt = double_to_str(user.theta*(180/PI))+" deg.";
				state = generateLandscape(cli_spat, cli_pert, srv_spat, srv_pert, state, landscape);
				mapping_time = ros::Time::now();
				mapping_dur = double_to_str((mapping_time-gestproc_time).toSec());

				if(state == -1)
					return 1;
				ROS_INFO("Kernel generated successfully");

				if(state==2)
				{
					initpose_txt = "("+double_to_str(robot.x)+"  "+double_to_str(robot.y)+")";

					//Set input for Monte MonteCarlo
					for (int i=0; i<RES*RES; i++)
						srv_mont.request.landscape.push_back(landscape[i]);

					//Call MonteCarlo
					if (cli_mont.call(srv_mont))
					{
						goal = srv_mont.response.goal;
						double pert_val = srv_mont.response.pert_value.data;
						pert_txt = double_to_str(pert_val);
						// Verify if goal returned is valid
						if(isnan(goal.x) && isnan(goal.y))
						{
							ROS_INFO("Invalid goal position");
							finalpose_txt = "Invalid";
							state = -1;
						}
						else
						{
							ROS_INFO("Goal obtained: (%f,%f)", goal.x, goal.y);
							finalpose_txt = "("+double_to_str(goal.x)+"  "+double_to_str(goal.y)+")";
							state = 3;
						}
					}
					else
					{
						ROS_ERROR("Failed to call Monte Carlo service");
						return 1;
					}
					srv_mont.request.landscape.clear();
					montecarlo_time = ros::Time::now();
					montecarlo_dur = double_to_str((montecarlo_time-mapping_time).toSec());
				}
				


				// Finally, call RRT* server and publish path
				if(state==3)
				{
					ROS_INFO("Calling RRT* Path Planner service");
					srv_rrts.request.Obstacles.clear();
					/* Assign the obstacle region for RRT* */
					for (int i=0; i<numObs+1; i++)
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
					//ros::spinOnce();
					init.x = robot.x;
					init.y = robot.y;
					init.z = 0;
					ROS_INFO("Robot position before calling RRT %f %f",init.x, init.y);

					// Define goal region
					goal_reg.center_x = goal.x;
					goal_reg.center_y = goal.y;
					goal_reg.center_z = 0;
					goal_reg.size_x = GOAL_SIZE;
					goal_reg.size_y = GOAL_SIZE;
					goal_reg.size_z = 0;

					// Note: workscape and object regions already defined
					srv_rrts.request.Goal = goal_reg;
					srv_rrts.request.Init = init;

					if(cli_rrts.call(srv_rrts))
					{
						pathsize = srv_rrts.response.path.size();
						if(pathsize < 3)
						{
							ROS_INFO("Path not found: Resetting...");
							state=0;
							path_found = "no";				
						}
						else
						{
							ROS_INFO("Path found: Publishing...");
							path_found = "yes";
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
							// Plot using opencv final landscape with goal position
							float pertmatrix[RES][RES]; //For opencv
							for (int i=0; i<RES*RES; i++)
							{
								pertmatrix[i/RES][i%RES] = landscape[i].data*255;
							}
		                                        plotPertinence("Mapped landscape", pertmatrix, true, goal, rrtPath);
						}
						
					}
					else
					{
						ROS_ERROR("Failed to call RRT* Path Planner");
						return 1;
					}
					// Enable robot control
					enable.data = true;
					flag_pub.publish(enable);
					rrtstar_time = ros::Time::now();
					rrtstar_dur = double_to_str((rrtstar_time-montecarlo_time).toSec());
				 }
			
				n.setParam("/color_key", 2);
		 	}

			if(cmd_received)
			{
				ker_txt = int_to_str(num_ker);
				// Log results
				std::string log_txt = ","+cmd_txt+", "+command+", "+ker_txt+", "+relation_txt+
				", "+objid_txt+", "+qualifier_txt+", "+pointing_txt+", "+user_txt+
				", "+initpose_txt+", "+finalpose_txt+", "+pert_txt+", "+path_found+
				", "+gestproc_dur+", "+mapping_dur+", "+montecarlo_dur+", "+rrtstar_dur;
				writeStrToFile(log_txt);
				cmd_received = false; // Set flag to false
			}
			command = "";
		 }
		 
		 /* Spin and wait for next period */
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
	 return 0;
}
