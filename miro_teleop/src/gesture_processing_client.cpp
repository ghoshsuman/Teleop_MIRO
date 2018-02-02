#include "ros/ros.h"
#include "miro_teleop/GestureProcessing.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gesture_processing_client");

  ros::NodeHandle n;
  ros::ServiceClient client = 
	n.serviceClient<miro_teleop::GestureProcessing>("gesture_processing");
  miro_teleop::GestureProcessing srv;
  
  geometry_msgs::Pose TestPose;
  TestPose.position.x = 10;
  TestPose.position.y = 10;
  TestPose.position.z = 20;
  TestPose.orientation.w = 0;
  TestPose.orientation.x = 1;
  TestPose.orientation.y = 1;
  TestPose.orientation.z = -1;

  srv.request.gesture = TestPose;
  if (client.call(srv))
  {
    ROS_INFO("Target found: (%f,%f,%f)", srv.response.target.x, 
		       srv.response.target.y, srv.response.target.theta);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
