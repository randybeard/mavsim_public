The objective of this assignment is to implement several of the path planning
algorithms described in this chapter. Skeleton code for this chapter is given
on the website. The file createWorld.m creates a map similar to those
shown in figures 12.11 and 12.12. The file drawEnvironment.m draws
the map, the waypoint path, and the current straight-line or orbit that is
being followed. The file path planner.m contains a switch statement for
manually choosing between different path planning algorithms. The sample
code contains the file planRRT.m for planning straight line paths through
a synthetic urban terrain.

12.1 Using planRRT.m as a template, create
planRRTDubins.m and modify path planner.m so that it calls
planRRTDubins.m to plan Dubins paths through the map. Modify
planRRTDubins.m to implement the RRT algorithm based on Du-
bins paths and the associated smoothing algorithm. Test and debug
the algorithm on the guidance model given in equation (9.18). When
the algorithm is working well on the guidance model, verify that it
performs adequately for the full six-DOF model.
