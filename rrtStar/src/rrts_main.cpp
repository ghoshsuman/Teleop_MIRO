#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <list>
#include <string>
#include <ros/ros.h>
#include <vector>
#include <rrtstar_msgs/rrtStarSRV.h>
#include <rrtstar_msgs/Region.h>
#include <geometry_msgs/Vector3.h>
#include <time.h>

#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;



typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

int publish_Tree_Regions (string time_start, planner_t& planner, System& system);
int publishTraj (string time_start, planner_t& planner, System& system, State &, float* goalCenter);

bool generatePath(rrtstar_msgs::rrtStarSRV::Request &req, rrtstar_msgs::rrtStarSRV::Response &res){
//! request Values:

	cout<<"**********"<<endl;
	float x[6];
	x[0]=req.WS.center_x;
	x[1]=req.WS.center_y;
	x[2]=req.WS.center_z;
	x[3]=req.WS.size_x;
	x[4]=req.WS.size_y;
	x[5]=req.WS.size_z;
    cout<<"WS: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<" "<<x[5]<<" " <<endl;

	float y[6];
	y[0]=req.Goal.center_x;
	y[1]=req.Goal.center_y;
	y[2]=req.Goal.center_z;
	y[3]=req.Goal.size_x;
	y[4]=req.Goal.size_y;
	y[5]=req.Goal.size_z;
    cout<<"Goal: "<<y[0]<<" "<<y[1]<<" "<<y[2]<<" "<<y[3]<<" "<<y[4]<<" "<<y[5]<<" " <<endl;

	float z[3];
	z[0]=req.Init.x;
	z[1]=req.Init.y;
	z[2]=req.Init.z;
    cout<<"InitialPoint: "<<z[0]<<" "<<z[1]<<" "<<z[2]<<endl;

	for (int i=0; i< req.Obstacles.size(); ++i){
	      const rrtstar_msgs::Region &data = req.Obstacles[i];
	      cout<<"req.Obstacles("<<i<<")"<<data.center_x<<" "<<data.center_y<<" "<<data.center_z<<" "<<data.size_x<<" "<<data.size_y<<" "<<data.size_z<<" "<<endl;
    }
//! rrtStar Method:

    planner_t rrts;
    // Create the dynamical system
    System system;

    //variables:
    float gamaValue=1.5;
    int NoIteration=40000;
    geometry_msgs::Vector3 pathState;

      // Three dimensional configuration space
    system.setNumDimensions (2);

      // Define the operating region
    system.regionOperating.setNumDimensions(3);
    system.regionOperating.center[0] = req.WS.center_x;
    system.regionOperating.center[1] = req.WS.center_y;
    //system.regionOperating.center[2] = req.WS.center_z;
    system.regionOperating.size[0] = req.WS.size_x;
    system.regionOperating.size[1] = req.WS.size_y;
    //system.regionOperating.size[2] = req.WS.size_z;

      // Define the goal region
    system.regionGoal.setNumDimensions(3);
    system.regionGoal.center[0] =req.Goal.center_x;
    system.regionGoal.center[1] =req.Goal.center_y;
    //system.regionGoal.center[2] =req.Goal.center_z;
    system.regionGoal.size[0] = req.Goal.size_x;
    system.regionGoal.size[1] = req.Goal.size_y;;
    //system.regionGoal.size[2] = req.Goal.size_z;

    float goalCenter[2];
    goalCenter[0]=system.regionGoal.center[0];
    goalCenter[1]=system.regionGoal.center[1];
    //goalCenter[2]=system.regionGoal.center[2];


  	// Setup the root vertex

      // Add the system to the planner
    rrts.setSystem (system);

      // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();
    State &rootState = root.getState();
    rootState[0] = req.Init.x;
    rootState[1] = req.Init.y;
   // rootState[2] = req.Init.z;

    // Define the obstacle region
    if (system.obstacles.size()>0)
    	system.obstacles.clear();

	for (int i=0; i< req.Obstacles.size(); ++i){
		const rrtstar_msgs::Region &obs = req.Obstacles[i];
	    region *obstacle;
	    obstacle = new region;
	    obstacle->setNumDimensions(3);
	    obstacle->center[0] =obs.center_x;
	    obstacle->center[1] =obs.center_y;
	//    obstacle->center[2] =obs.center_z;
	    obstacle->size[0] = obs.size_x;
	    obstacle->size[1] = obs.size_y;
	  //  obstacle->size[2] = obs.size_z;
	    system.obstacles.push_front (obstacle);  // Add the obstacle to the list
	}


      // Initialize the planner
    rrts.initialize ();

      // This parameter should be larger than 1.5 for asymptotic
      //   rather than exploration in the RRT* algorithm. Lower
      //   optimality. Larger values will weigh on optimization
      //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (gamaValue);

    clock_t start = clock();
    // Run the algorithm for 10000 iteartions
	for (int i = 0; i < NoIteration; i++)
    	rrts.iteration ();
    clock_t finish = clock();

    cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;

	list<double*> stateList;
    rrts.getBestTrajectory (stateList);
    cout<<"stateList.size(): "<<stateList.size()<<endl;

    //! add init state to returning path
//    if (stateList.size()>0){
        pathState.x=rootState[0] ;
        pathState.y=rootState[1];
        if (system.getNumDimensions() > 2)
        	pathState.z=rootState[2];
        else
        	pathState.z=0.0;
        res.path.push_back(pathState);
//    }

    //! if a path based on rrtstar found, add it to returning path:

    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        double* TrajState = *iter;
        pathState.x=TrajState[0];
        pathState.y=TrajState[1];

        if (system.getNumDimensions() > 2)
        	pathState.z=TrajState[2];
        else
        	pathState.z=0.0;
        delete [] TrajState;
        res.path.push_back(pathState);
    }

    //! add final goal to the found path:
	pathState.x=goalCenter[0];
	pathState.y=goalCenter[1];
	if (system.getNumDimensions() > 2)
		pathState.z=goalCenter[2];
	else
		pathState.z=0.0;
	res.path.push_back(pathState);


//    for (int i=0;i<stateList.size();i++){
//		cout<<"pitt_call.objectFeature["<<i<<"][0]: "<<pitt_call.objectFeature[i][0]<<" "<<pitt_call.objectFeature[i][1]<<endl;
//		obstacle.center_x=pitt_call.objectFeature[i][0];
//		obstacle.center_y=pitt_call.objectFeature[i][1];
//		obstacle.center_z=pitt_call.objectFeature[i][2];
//		obstacle.size_x=pitt_call.objectFeature[i][3];
//		obstacle.size_y=pitt_call.objectFeature[i][4];
//		obstacle.size_z=pitt_call.objectFeature[i][5];
//		res..push_back(obstacle);
//	}
	char stringTime[20];

	sprintf(stringTime, "%d", ((double)(start))/CLOCKS_PER_SEC);

    publish_Tree_Regions(stringTime,rrts, system);
    publishTraj (stringTime,rrts, system,rootState, goalCenter);

     return true;
  }



int main (int argc, char** argv) {
	// IMPORTANT:

	// 1- define safety factor of the obstacles here;

	ros::init(argc, argv, "rrtstar");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("rrtStarService",generatePath);

    cout << "*****************" << endl;
    cout << "RRTstar is alive: " << endl;

    ros::spin();
    return 1;
}

int publishTraj ( string stringTime,planner_t& planner, System& system,State & initState, float * goalCenter) {

	const char* DataLogPath	="/home/nasa/Datalog/rrtStar";
	string DataLogPath2		="/home/nasa/Datalog/rrtStar";
	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream Myfile1;

	cout<<"stringTime: "<<stringTime<<endl;
	Myfile1.open ((DataLogPath2+"/"+stringTime+"_Trajectory.txt").c_str(),ios::trunc);

    cout << "Publishing trajectory -- start" << endl;

    vertex_t& vertexBest = planner.getBestVertex ();

    //cout<<"planner.getBestVertexCost(): "<<planner.getBestVertexCost()<<endl;

    if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        return 0;
    }

    list<double*> stateList;
    planner.getBestTrajectory (stateList);
    cout<<"stateList.size(): "<<stateList.size()<<endl;

    //! insert initial state to the response path:
//    if (stateList.size()>0){
	Myfile1 <<initState[0]<<" "<<initState[1]<<" ";
	if (system.getNumDimensions() > 2)
		Myfile1 <<initState[2]<<"\n";
	else
		Myfile1 <<0.0<<"\n";
//    }

	//! insert found trajectory to the response path:
    int stateIndex = 0;
    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        double* stateRef = *iter;
        cout<<stateRef[0]<<" "<<stateRef[1]<<" ";
    	Myfile1 <<stateRef[0]<<" "<<stateRef[1]<<" ";
    	//cout<<"system.getNumDimensions(): "<<system.getNumDimensions()<<endl;
        if (system.getNumDimensions() > 2){
        	cout<<stateRef[2]<<"\n";
        	Myfile1 <<stateRef[2]<<"\n";
        }
        else{
           	cout<<0.0<<"\n";
           Myfile1 <<0.0<<"\n";
        }
        delete [] stateRef;

        stateIndex++;
    }

    //! insert final state to the response path:
	Myfile1 <<goalCenter[0]<<" "<<goalCenter[1]<<" ";
	if (system.getNumDimensions() > 2)
		Myfile1 <<goalCenter[2]<<"\n";
	else
		Myfile1 <<0.0<<"\n";

    Myfile1.close();
    cout << "Publishing trajectory -- end" << endl;
    return 1;
}

int publish_Tree_Regions (string stringTime,planner_t& planner, System& system) {

	const char* DataLogPath	="/home/nasa/Datalog/rrtStar";
	string DataLogPath2		="/home/nasa/Datalog/rrtStar";
	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream Myfile1,Myfile2,Myfile3;
//	char stringIter[10];
//	sprintf(stringIter, "%d", iterator);

	Myfile1.open ((DataLogPath2+"/"+stringTime+"_Vertices.txt").c_str(),ios::trunc);
	Myfile2.open ((DataLogPath2+"/"+stringTime+"_Edges.txt").c_str(),ios::trunc);
	Myfile3.open ((DataLogPath2+"/"+stringTime+"_Regions.txt").c_str(),ios::trunc);

    cout << "Publishing the tree -- start" << endl;

    bool plot3d = (system.getNumDimensions() > 2);

   int num_vertices = planner.numVertices;

   //!PUBLISH VERTICES

    if (num_vertices > 0) {

        int vertexIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();

//            cout<<vertexIndex<<" "<<stateCurr[0]<<" "<<stateCurr[1];
            Myfile1 <<stateCurr[0]<<" "<<stateCurr[1];
            if (plot3d){
//            	cout<<" "<<stateCurr[2]<<endl;
            	Myfile1 <<" "<<stateCurr[2]<<"\n";
            }
             else{
//            	cout<<" "<<0.0<<endl;
            	Myfile1 <<" "<<0.0<<"\n";
             }
            vertexIndex++;

        }

    }
    else {
    	cout<<"num_vertices <= 0"<<endl;
    }

    //! PUBLISH EDGES
    if (num_vertices > 1) {

        int num_edges = num_vertices - 1;

        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

            vertex_t &vertexCurr = **iter;

            vertex_t &vertexParent = vertexCurr.getParent();

            if ( &vertexParent == NULL )
                continue;

            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();

//            cout<<edgeIndex<<" "<<stateParent[0]<<" "<<stateParent[1]<<" ";
            Myfile2 <<stateParent[0]<<" "<<stateParent[1]<<" ";
            if (plot3d){
//            	cout<<stateParent[2]<<" ";
            	Myfile2 <<stateParent[2]<<" ";
            }
            else{
//            	cout<<0.0<<" ";
            	Myfile2 <<0.0<<" ";
            }
//            cout<<stateCurr[0]<<" "<<stateCurr[1]<<" ";
            Myfile2 <<stateCurr[0]<<" "<<stateCurr[1]<<" ";

            if (plot3d){
//            	cout<<stateCurr[2]<<endl;
                Myfile2 <<stateCurr[2]<<"\n";
            }
            else{
//            	cout<<0.0<<endl;
            	Myfile2 <<0.0<<"\n";
            }
            edgeIndex++;
        }

    }
    else {
       	cout<<"num_vertices <= 1"<<endl;
    }
    Myfile1.close();
    Myfile2.close();

    cout << "Publishing the tree -- end" << endl;

    cout << "Publishing the Regions -- start" << endl;
    // First Line of Regions is Operating Region
    // Second Line of Region is Goal Region
    // From third Line is the obstacles Regions

	Myfile3 <<system.regionOperating.center[0]<<" "<<system.regionOperating.center[1]<<" "<<system.regionOperating.center[2]<<" ";
	Myfile3 <<system.regionOperating.size[0]<<" "<<system.regionOperating.size[1]<<" "<<system.regionOperating.size[2]<<"\n";

	Myfile3 <<system.regionGoal.center[0]<<" "<<system.regionGoal.center[1]<<" "<<system.regionGoal.center[2]<<" ";
	Myfile3 <<system.regionGoal.size[0]<<" "<<system.regionGoal.size[1]<<" "<<system.regionGoal.size[2]<<"\n";

    for (list<region*>::iterator iter = system.obstacles.begin(); iter != system.obstacles.end(); iter++) {
    	region *obstacle= *iter;

    	Myfile3 <<obstacle->center[0]<<" "<<obstacle->center[1]<<" "<<obstacle->center[2]<<" ";
    	Myfile3 <<obstacle->size[0]<<" "<<obstacle->size[1]<<" "<<obstacle->size[2]<<"\n";
    }

    Myfile3.close();
    cout << "Publishing the Regions -- end" << endl;



    return 1;
}
