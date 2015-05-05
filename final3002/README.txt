Make sure the nav2d package is downloaded with dependencies
In a new terminal window:
   $roslaunch final3002 turtle_sim.launch

In a new terminal window:
   $rosservice call /StartMapping 3

wait for "response: 0" in terminal

   $rosservice call /StartExploration 2

wait for "response: 0" in terminal

watch as the robot explores the map autonomously
