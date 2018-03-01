# Teleop_MIRO
Software Architecture: Teleoperation with MIRO using gesture and speech
Using Motion Capture for localization and gesture recognition

Objective:
 

Accomplishments:

MiRo and obstacle in the motion capture area with markers on them.

    Pointing gesture on the ground is used to calculate a goal-point where MiRo has to reach.
    Voice command - look (for now typed in the workstation) is used to orient MiRo towards the goal.
    Montecarlo simulation is used to generate an approximate point near the goal-point.
    RRT* planner is used to generate a path from MiRo to the montecarlo given goal-point.
    Voice command - go (for now typed in the workstation) is used to move MiRo towards the goal, as it avoids the obstacle.
    MiRo reaches the goal position.
    MiRo can also be stopped by a voice command - stop (for now typed in the workstation).

 

Modules (files) in the system:

    File 'bla-bla' - does bla-bla. Takes input bla-bla … gives output bla-bla

 

Limitations of the system:

 

How to run the application:
