#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#define MIRO_WHEEL_TRACK_MM 280

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_controller");

  ros::NodeHandle n;

  ros::Publisher ctl_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate loop_rate(10);

  ROS_INFO("Robot controller active\n");

  while (ros::ok())
  {
    bool enable = true;
    geometry_msgs::Twist vel;

    double wl = -0.1;
    double wr = 0.05;

    double dr = (wl+wr)/2.0;
    double dtheta = (wr-wl)/(MIRO_WHEEL_TRACK_MM/1000.0);

    vel.linear.x = dr;
    vel.angular.z = dtheta;

    ctl_pub.publish(vel);
    ROS_INFO("Command velocity: [%f], [%f]\n",dr,dtheta);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
