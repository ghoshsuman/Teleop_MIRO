/* Libraries */
#include "ros/ros.h"
#include "std_msgs/UInt8.h"
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
 	std_msgs::UInt8 msg; // Command tag associated and published.
 	std::string cmd; // Command string parsed by speech recognition.

	/* Initialize and assign node handler */
	ros::init(argc, argv, "interpreter");
	ros::NodeHandle n;

	/* Initialize publisher */
	ros::Publisher cmd_pub = 
		n.advertise<std_msgs::UInt8>("command", 1);

	/* Update rate (period) */
  	ros::Rate loop_rate(10);

 	ROS_INFO("Command Interpreter node active");

	/* Main loop */
	while (ros::ok())
	{
		/* Receiving input from the user */
		std::cout << "Awaiting command: ";
		std::cin >> cmd;

		/* Command-tag mapping */
		if (!cmd.compare("look")) msg.data = 1;
		else if (!cmd.compare("go")) msg.data = 2;
		else if (!cmd.compare("stop")) msg.data = 3;
		else msg.data = 0;

		/* Publish only in case something meaningful was received */
		if(msg.data>0)
		{
			cmd_pub.publish(msg);
			ROS_INFO("Sent command: [%s] to master",cmd.data());
		}

		/* Spin and wait for next period */
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
