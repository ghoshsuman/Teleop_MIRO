#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <vector>

#define MIRO_WHEEL_TRACK_MM 280

std::vector<geometry_msgs::Vector3> path;

void ctlCallBack(const geometry_msgs::Vector3::ConstPtr& point)
{
	path.push_back(*point);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle n;

  ros::Publisher ctl_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber path_sub = n.subscribe("path", 1000, ctlCallBack);

  ros::Rate loop_rate(10);

  ROS_INFO("Robot controller active\n");

  bool enable = true;
  geometry_msgs::Twist vel;
  geometry_msgs::Vector3 p;
  double wl, wr, dr, dtheta;

  while (ros::ok())
  {

    if(enable && !path.empty())   
    {
      p = path.front();
      path.erase(path.begin());

      wl = p.x;
      wr = p.y;

      dr = (wl+wr)/2.0;
      dtheta = (wr-wl)/(MIRO_WHEEL_TRACK_MM/1000.0);

      vel.linear.x = dr;
      vel.angular.z = dtheta;

      ctl_pub.publish(vel);
      ROS_INFO("Current reference: [%f], [%f]\n",p.x,p.y);
      ROS_INFO("Command velocity: [%f], [%f]\n",dr,dtheta);

    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
