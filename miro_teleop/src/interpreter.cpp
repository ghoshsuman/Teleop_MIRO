/* Libraries */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

/**
 * Command Interpreter Node main function.
 * Obtains speech from user and translate them into commands.
 *
 * The command strings input by the user (typed in the terminal) are mapped to 
 * flags (integer values) and sent to the Command Logic Node.
 *
 * If sentences are received, parsing the text is necessary, which must be done
 * by calling the Speech Recognition Node (currently in development).
 */
int main(int argc, char **argv)
{
	/* Definitions */
 	std_msgs::String msg; // Command tag associated and published.
 	std::string cmd; // Command string parsed by speech recognition.

	/* Initialize and assign node handler */
	ros::init(argc, argv, "interpreter");
	ros::NodeHandle n;

	/* Initialize publisher */
	ros::Publisher cmd_pub = 
		n.advertise<std_msgs::String>("command", 1);

	/* Update rate (period) */
  	ros::Rate loop_rate(10);

 	ROS_INFO("Command Interpreter node active");

	/* Main loop */
	while (ros::ok())
	{
		/* Receiving input from the user */
		std::cout << "Awaiting command: ";
		std::cin >> cmd;

		msg.data = cmd;
		cmd_pub.publish(msg);
		ROS_INFO("Sent command: [%s] to master", cmd.data());

		/* Spin and wait for next period */
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
