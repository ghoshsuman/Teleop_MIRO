# mmodal_teleop
Multi-Modal Qualitative Teleoperation with Speech and Gestures

**Objective:**
ROS-based software architecture that drives the MIRO robot from Consequential Robotics to a given position, given a set of spatial relations with respect to referenced objects in the workspace, and a pointing gesture. We use as feedback the information provided by a visual tracking system, in this case the Motion Capture by OptiTrack.

**Publication:**
https://www.researchgate.net/publication/328806816_A_Scalable_Architecture_to_Design_Multi-modal_Interactions_for_Qualitative_Robot_Navigation_XVIIth_International_Conference_of_the_Italian_Association_for_Artificial_Intelligence_Trento_Italy_November


**Dependencies:**
Install the ross_cagg_miro_teleop (https://github.com/EmaroLab/ros_cagg_miro_teleop) packages and all its sub-dependencies.

**Installation and experiment preparation:**
- Clone the master branch of mmodal_teleop repository inside the `src` folder of your `catkin_ws`
- The following 2 OpenCV steps are **optional**. You can either follow them, or comment out relevant code in `miro_teleop/src/command_logic.cpp`
    - (optional) Install OpenCV following these instructions: https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
    - (optional) Set OpenCV path corresponding to your system and add dependency in `miro_teleop/CMakeLists.txt`
- Run `catkin_make` from inside `catkin_ws`
- Download MIRO MDK and MIRO app from http://labs.consequentialrobotics.com/miro/mdk/
- Turn on MIRO. Run the app and connect with MIRO, to find MIRO's IP. Ensure your machine is on the same network as MIRO.
- Edit the `~/.profile` file in root@<MIRO's IP>, set ROS_MASTER_IP with machine IP running the roscore. Source it.
- Run/Enable bridge on the app. You might need to do this at frequent intervals if MIRO stops responding.
- In the Motive software for Mocap Optitrack, enable Data Streaming and set broadcast IP to the machine running the roscore.
- Setup up markers in the Motion Capture area. Create Rigid Bodies in Motive (first Robot, then User, Gesture and Obstacles in order), after aligning required local axes with global axes (the user must be pointing north while the robot and the gesture tool must be aligned towards east of the absolute frame).
- Set the microphone to enable user speech.
- Launch the application using `roslaunch miro_teleop miro_teleop.launch`. A web page will appear, with the voice recognition API interface. Plase verify whether ROS Bridge is connected. If not, it is necessary to restart the application. Otherwise, click on the microphone button to enable speech.

**Experimental procedure:**
- When MIRO turns green lights on, it means that the user is able to issue a command, in the form 'go + qualifier(optional) + relation + object id' (e.g.: 'go strictly in front of object number one'). He/she should point to the desired position at the moment of speech. During the command processing MIRO's lights turn yellow. If the command was understood by the speech recognition node, it will turn MIRO's lights blue and proceed with goal computation, otherwise MIRO turns red to indicate that the command was not clear. When the processing is finished, the robot starts moving towards the goal, and turns the lights back to green.
- The robot halts once the goal is reached, within a small tolerance. If the user is satisfied with such position, he/she can say 'done' and the robot will acknowlegde the command by turning on white lights. Otherwise, the user may continue giving new commands to add relations to the system, unless he/she wants to abort the mission by saying 'reset' (the robot will turn purple lights and stop to acknowledge that). In both the 'done' and 'reset' cases the mapping is cleared and all the previous commands are forgotten.
- Issuing new commands can be done asynchronously in between actions. However, if the new command gives an inconsistent map, MIRO body will turn purple (reset) lights and stop.
- Keep an eye on the Motive screen. If rigid bodies are not being tracked properly, you might need to create rigid bodies again with proper alignment.


**Accomplishments:**

MiRo and obstacle in the motion capture area with markers on them.

- Pointing gesture on the ground is used to calculate a goal-point where MiRo has to reach.
- Voice command - look (for now typed in the workstation) is used to orient MiRo towards the goal.
- Montecarlo simulation is used to generate an approximate point near the goal-point.
- RRT* planner is used to generate a path from MiRo to the montecarlo given goal-point.
- Voice command - go (for now typed in the workstation) is used to move MiRo towards the goal, as it avoids the obstacle.
- MiRo reaches the goal position.
- MiRo can also be stopped by a voice command - stop (for now typed in the workstation).

 
**Modules (files) in the system:**

    File 'miro_teleop/src/command_logic.cpp' - Main Command Logic node.
    File 'miro_teleop/src/interpreter.cpp' - Interpreter node.
    File 'miro_teleop/src/gesture_processing.cpp' - Gesture Processing server.
    File 'miro_teleop/src/monte_carlo.cpp' - Monte Carlo Simulation server.
    File 'miro_teleop/src/pertinence_mapping.cpp' - Pertinence Mapping server.
    File 'miro_teleop/src/spatial_reasoner.cpp' - Spatial Reasoner server.
    File 'miro_teleop/src/robot_controller.cpp' - Robot Controller node.
    File 'miro_teleop/srv/GestureProcessing.srv' - Gestutre Processing service
    File 'miro_teleop/srv/MonteCarlo.srv' - Monte Carlo Simulation service
    File 'miro_teleop/srv/PertinenceMapping.srv' - Pertinence Mapping service
    File 'miro_teleop/srv/SpatialReasoner.srv' - Spatial Reasoner service
    File 'miro_teleop/srv/rrtStarSRV.srv' - RRT* service
    File 'miro_teleop/msg/Path.msg' - msg used to transfer trajectory from Command Logic to Robot Controller
    File 'rrtStar/src/rrts_main.cpp' - RRT* server
    File 'mocap_optitrack-master/launch/mocap.launch' - Launches mocap node
    File 'miro_teleop/launch/miro_teleop.launch' - Launches the whole application including mocap nodes
    File 'run_miro_teleop' - Shell script to run all nodes in separate terminals
    File 'doc/html/index.html' - Doxygen generated documentation

 
**System scope**
 - This system currently runs only on rectangular workspaces covered by the Motion Capture, and only with MIRO. However, the modular approach allows one to substitute the sensing and actuacting components without harming the overall functioning of the architecture.
