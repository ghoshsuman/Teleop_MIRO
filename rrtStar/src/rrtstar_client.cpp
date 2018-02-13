#include "ros/ros.h"
#include <rrtstar_msgs/rrtStarSRV.h>
#include <rrtstar_msgs/Region.h>
#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrtstar_client");

  ros::NodeHandle n;
  ros::ServiceClient client = 
	n.serviceClient<rrtstar_msgs::rrtStarSRV>("rrtStarService");
  rrtstar_msgs::rrtStarSRV srv;
  rrtstar_msgs::Region obs;
  
  srv.request.WS.center_x = 0;
  srv.request.WS.center_y = 0;
  srv.request.WS.center_z = 0;

  srv.request.WS.size_x = 400;
  srv.request.WS.size_y = 400;
  srv.request.WS.size_z = 0;

  srv.request.Goal.center_x = 50;
  srv.request.Goal.center_y = 50;
  srv.request.Goal.center_z = 0;

  srv.request.Goal.size_x = 10;
  srv.request.Goal.size_y = 10;
  srv.request.Goal.size_z = 0;

  srv.request.Init.x = -50;
  srv.request.Init.y = -50;
  srv.request.Init.z = 0;

  obs.center_x = 20;
  obs.center_y = 30;
  obs.center_z = 0;

  obs.size_x = 5;
  obs.size_y = 5;
  obs.size_z = 0;

  srv.request.Obstacles.push_back(obs);

  if (client.call(srv))
  {
    std::cout << "Path found" << std::endl;
    for(int i=0;i<srv.response.path.size();i++)
    {
	std::cout << srv.response.path[i].x << " ";
	std::cout << srv.response.path[i].y << " ";
	std::cout << srv.response.path[i].z << " ";
	std::cout << std::endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service rrtstar");
    return 1;
  }

  return 0;
}
