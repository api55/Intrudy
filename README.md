Intrudy
========

### The System ##########


Intrudy is a intrusion detection system created by **Pablo Aponte** and **Ilya Manyugin** using python and ROS. It can be defined as Finite State Machine with four states.

1.    Surveillance : In this state Intrudy is stationary (i.e., the command given on each iteration is {0, 0}, that is, both the linear and angular speeds are set to zero), and the movement detection threshold in the target tracking module is set to a lower value, in order to aid early target detection.
2.    Following: In this state Intrudy is moving, following a target, and the effective command is dependent on both the angle of the target relative to Intrudy’s front and the distance between the Intrudy and the target.
3.    Re-Acquisition: The FSM transits to this state from the Following state if the Tracking Module reports a target loss (i.e., the return value of the track method is None). In this state Intrudy is still moving towards the place where the target was last detected, simultaneously trying to re-acquire the target. If Intrudy fails to re- acquire target within 5 time steps (a configurable parameter of Intrudy specified in config.py) since the target loss, it transits to the Surveillance state. If the target was successfully re-acquired, it transits to the Following state.
4.    Collision: A collision detection algorithm ensures that all the range values retrieved from the SICK S300 laser scanner are greater than 0.15 m. If this assertion doesn’t hold, we say that we detected a collision with an obstacle, and Intrudy transits into the Collision state. In this state Intrudy runs at each time step the collision detection algorithm in order to check, whether the obstacle has changed its location. Therefore, if Intrudy collides with a stable object, it cannot resolve the collision by itself, and the only way to make it operable again is for the user to move the robot away from the obstacle.

### Movement Detection ######

In the Intrudy project we implement and widely use a technique called the ICP Optimization. The main idea of the ICP Optimization is to use the ICP algorithm and previous knowledge about the target (its position), in order to reduce the search space in the newly obtained scan.

Given two scans s_0 and s_1 , we apply the ICP algorithm using both of them, changing s_0 in such a way that the static objects (such as walls, chairs, etc.) remain in the same position in both scans, while the displaced objects (e.g., a person walking) are the ones that change between scans. We get the transformation T as the output of the ICP algorithm, and we apply this transformation to the s_0 scan and the previously obtained target position p_t . Applying the transformation T to the previous target position t_p , we get the estimation of the target in the next scan p_t' = T (p_t ). This estimation is saved for later use in a class variable.

### Tracking #######

Given two range scans, we truncate the values to the maximum range. Additionaly the scans are converted from a robot-centric polar coordinate system into Cartesian coordinate system. If Intrudy is not in surveillance mode the ICP optimization of the first scan is done and, at the same time, the previously detected target, if any, is moved accordingly to this ICP optimization. This optimization is done in the first scan so that we can also recalculate the position of the previously obtained target.

After the optimization, the corresponding points are found in the two sets (scans). These are the points that are considered to have been a subject to transformation due to the target movement. We consider all the n points of the ‘new’ scan S_1 and compute the Euclidean distance E_d between each point from S_1 and all the points from the set S_0 . If all the Euclidean distances are greater than a configurable parameter alpha, then the point p_S_1 ,i is considered an ‘outlier’. This approach allows us to ignore the noise encountered in the data that is introduced by the resolution of the laser scanner.

Once we found the outliers, it is possible to group them or discard them. The outliers will be group depending on their differences towards the robot. They must have a difference with the same signed to be able to be grouped. This groups will be consider the most probable targets.

Once the most probable targets are found, we use a heuristic to determine which target is the best, and, therefore, the actual target the algorithm returns. The heuristic selects that target, which is the closest one to the previous target location, determined in the previous run. If there is no previous target location, the algorithm will select the target consistion of the greatest number of points.

If the robot is in either the following or the re-aquisition state, it will not use the full range of its lasers, but instead only a 90° window centered in the previous target location. While in this states, the target position will be also updated (transformed) to account for the robots movements.

### Notes ##########

This program was tested in robots created by the university of Bonn, that consist of a laptop connected to a Roomba robot. This program was tested using Ubuntu 10.04.2 and ROS Electric. Later on it was tested with ROS Groovy in the simulator and it worked.

For more information about the details, requirements, how to use them and the results of our experiment take a look into [the manual](https://github.com/api55/Intrudy/raw/master/doc/programmers_manual/programmers_manual.pdf). 
