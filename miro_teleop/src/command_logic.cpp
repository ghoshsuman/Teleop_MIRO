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

std_msgs::UInt8 cmd;

void interpreterCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  cmd.data = msg->data;
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_logic");

  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<geometry_msgs::Vector3>("path",1000);
  ros::Publisher flag_pub = n.advertise<std_msgs::Bool>("enable",1);

  ros::Subscriber sub_cmd = n.subscribe("command", 1, interpreterCallback);

  ros::ServiceClient cli_gest = 
	n.serviceClient<miro_teleop::GestureProcessing>("gesture_processing");
  miro_teleop::GestureProcessing srv_gest;

  ros::ServiceClient cli_rrts = 
	n.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
  rrtstar_msgs::rrtStarSRV srv_rrts;

  ros::Rate loop_rate(10);

  geometry_msgs::Pose2D obs_c, robot, target;
  geometry_msgs::Pose gesture;
  geometry_msgs::Vector3 path[1000]; 
  std_msgs::Bool enable;

  gesture.position.x = 40;
  gesture.position.y = 40;
  gesture.position.z = 20;

  gesture.orientation.w = 0;
  gesture.orientation.x = 0;
  gesture.orientation.y = 0;
  gesture.orientation.z = -1;
 
  srv_gest.request.gesture = gesture;
  
  obs_c.x = 20.00;
  obs_c.y = 20.00;
  obs_c.theta = 0;

  robot.x = -10.00;
  robot.y = -10.00;
  robot.theta = 0;

  std_msgs::Float64 obsdim[2];
  obsdim[0].data = 5.00;
  obsdim[1].data = 5.00;

  rrtstar_msgs::Region workspace;
  workspace.center_x = 0;
  workspace.center_y = 0;
  workspace.center_z = 0;
  workspace.size_x = 100;
  workspace.size_y = 100;
  workspace.size_z = 0;
  
  rrtstar_msgs::Region goal;
  goal.center_x = target.x;
  goal.center_y = target.y;
  goal.center_z = 0;
  goal.size_x = 10;
  goal.size_y = 10;
  goal.size_z = 0;
  
  geometry_msgs::Vector3 init;
  init.x = robot.x;
  init.y = robot.y;
  init.z = 0;

  rrtstar_msgs::Region obs;
  obs.center_x = obs_c.x;
  obs.center_y = obs_c.y;
  obs.center_z = 0;
  obs.size_x = obsdim[0].data;
  obs.size_y = obsdim[1].data;
  obs.size_z = 0;

  srv_rrts.request.WS = workspace;
  srv_rrts.request.Goal = goal;
  srv_rrts.request.Init = init;
  srv_rrts.request.Obstacles.push_back(obs);
 
  while(ros::ok())
  {

    if(cmd.data==1) // Look
    { 
      // Call gesture processing server
      if (cli_gest.call(srv_gest))
      {
        target = srv_gest.response.target;
        ROS_INFO("Target found: (%f,%f,%f)", target.x, target.y, target.theta);
      }
      else
      {
        ROS_ERROR("Failed to call gesture processing service");
        return 1;
      }

      // Call pertinence mapping server

      // Call monte carlo server

      // Call RRT* server
      if(cli_rrts.call(srv_rrts))
      {
        ROS_INFO("Path found\n");
        for(int i=0; i<srv_rrts.response.path.size(); i++)
        {
          path[i].x = srv_rrts.response.path[i].x;
          path[i].y = srv_rrts.response.path[i].y;
          path[i].z = srv_rrts.response.path[i].z;
          ROS_INFO("[%f] [%f] [%f]\n", path[i].x, path[i].y, path[i].z);
        }
      }
      cmd.data = 0;
    }

    if(cmd.data==2) // Go
    { 
      // Publish path
      ROS_INFO("Publishing path...");
      for(int i=0; i<srv_rrts.response.path.size(); i++) 
        path_pub.publish(path[i]);
      // Then enable robot control
      enable.data = true;
      flag_pub.publish(enable);
      cmd.data = 0;
    }

    if(cmd.data==3) 
    {
      enable.data = false;
      flag_pub.publish(enable); // Stop
      cmd.data = 0;
    }  

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;

}
