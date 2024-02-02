#final_project_env

To spawn the dynamic obstacles:

`rosrun final_project dynamic_obstacles.py`
** The objective of the final project is to put into practice different concepts that have been acquired during
 the lab activities using the simulation environment Gazebo, and ROS. This is an individual project. The
 project will be done on a custom world that we have created to provide you with enough challenge and
 distinctive features to let you decide between different techniques to implement your solution. The project
 gives a total of 100 points for what we call basic tasks. In addition to those tasks, we have defined a set
 of tasks (that build upon the basic tasks) that will be considered for a total of up to 30 points, giving
 a total of 130 weighted as shown in the syllabus. A top level description of the tasks that you need to
 accomplish is included as follows:
 • Task 1- Autonomous Mapping: For this task you will need to use the gmapping library from the
 navigation stack as in lab 4; however, for this task it is NOT allowed to use the keyboard or fixed
 waypoints to map the space. Instead, you need to implement an algorithm to make turtlebot cover as
 much area as possible. You are free to use the full navigation stack to move the robot, but the way
 in which automatically decide which waypoint to visit next, should be your own solution. This task
 will be graded based on the coverage (visited grid cells / unvisited grid cells) of the map created by
 you algorithm.
 • Task 2- Navigation with Static Obstacles: In this task you need to use an A∗ path planner in the
 same way that you did for lab 4. You will be able to use the AMCL node for localization, but the
 path planning, and path following algorithms must be part of your solution, which is almost a copy
 of what you did for lab 4, the only difference is the world that you will be testing on. The map to
 solve the path planning can be taken from task 1. To grade this activity, we will spawn the robot in a
 random position and send a goal pose, you need to make sure that the turtlebot trajectory is collision
 free.
 • Task 3- Navigation with Dynamic Obstacles: As the name suggests, in this task we will include
 dynamic obstacles to the world used in task 2. In order to successfully complete this task you will
 need to add an obstacle avoidance strategy to your previous path planner, and path follower. The
 obstacles will have colors and shape that are easy to detect with respect to the world background.
 The grading of this activity will be based on the same elements that were graded for task 2, but we
 will additionally consider collisions with the obstacles.**
