#turtle for finding optimal path and avoiding obstacle 

Pathfinding_turtle is a package for learning turtlesim more interactively. 

#user guid 

There are two ways to play this path finding and obstacle avoiding turtle:

1. roslaunch pathfinding_turtle pathfinding_turtle_node
2. in command line, run "roscore", "rosrun turtlesim turtlesim_node"  and "rosrun pathfinding_turtle pathfinding_turtle_node" in three seperate terminal

After launching the turtle, you have to give the full path of obstacle position file and cost map file. The line of obstacle position file is defined by four integer number, the x of point1, y of point1, x of point2 and y of point2. The cost map is designed for simulating social navigation of robot, e.g. a human is in the environment of a robot. 



