#include "ros/ros.h"
#include "std_msgs/UInt8.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpreter");

  ros::NodeHandle n;

  ros::Publisher interpreter_pub = n.advertise<std_msgs::UInt8>("command", 1);

  ros::Rate loop_rate(10);

  ROS_INFO("Interpreter service active\n");

  while (ros::ok())
  {
    std_msgs::UInt8 msg;

    msg.data = 1;

    interpreter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
