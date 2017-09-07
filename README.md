# menelaus
A single quadcopter follows a group of friendly targets who periodically share their waypoints and positions. 
The quadcopter attempts to keep the targets in camera view as much as possible. 
However, the quadcopter also positions itself such that it can remain in place when possible to save energy while tracking the targets.

The project is called "menelaus" because the quadcopter follows its target just as Menelaus followed after Helen to Troy.
However, Helen was not a particularly friendly target to the pursuer, so the comparison fails. 

## overview
Much work has been done on target tracking with unmanned aerial vehicles.
However, the typical vehicle is a fixed wing with limited hovering or cruising ability. 
Also, the targets are not typically sharing positional information with the tracking vehicle.
This project explores the use of a quadcopter as a tracking vehicle whose targets are part of its multi-robot team and share information. 
Of interest is how such information can be used for autonomous path planning for the quadcopter that minimizes movement sinch hovering uses less energy than moving. 
Rather than constantly "tail-chase" the targets, the quadcopter should predict the path of the targets and select a place to hover while continueing to pan and tilt the camera.  

The quadcopter should be able to follow a variety of platforms and in a variety of terrain types. 
Heterogeniety of targets and terrains affect the path prediction. 
Machine learning should be used to enable to the quadcopter to track more effectively through more accurate path predictions. 

An additional feature would be to specify some point or area as an additional non-vehicle target. If possible, the quadcopter continues tracking the target vehicles while also keeping the stationary targets in view.
This would be of use to a remote operator of vehicles who wants to not only see the vehicle(s) from a good angle, but also certain objects or locations in the environment. 
Another idea is to keep the alitude set by the user rather than the autonomous quadcopter. It is reasonable that a user wants to view the targets, but also some amount of area around the targets to be truly useful. 
Or even a completely separate autonomous behavior that sends the altitude to this following behavior, based on some application-specific criteria. 

Generally, the project should be useful for a variety of multi-robot team applications.
But specifically, it is intended to be of use in the marine disaster recovery domain.
Multiple EMILY ERS robots can be better utilized if the first responders have an overhead view of the vehicles, rather than the elongated (and potentially occluded) view from the ship or shore. 
The ability to add non-vehicle targets would allow the first responders to see not only the vehicles, but have a view of the victim's location such as a capsizing ship. 

Since the quadcopter is using communication for following and not the camera, resources are free to use the camera for other vision tasks such as helping the targets with collision avoidance. 

## plan

The system is developed in increments. 
Each step could be its own behavior, progressing toward more sophisticated behaviors. 

1.  **basic following (done)**

	The quadcopter can tail-chase a group of targets.

2. **pan and tilt (done)**

	The quadcopter faces toward the centroid of the targets' next waypoint
 	and tilts the camera to keep their current position centroid in view. 

3. **basic hover (done)**

	Instead of tail-chasing, the quadcopter hovers behind the targets. 
	When the targets are leaving the camera footprint, the quadcopter catches up and hovers again. 
	If the targets are often staying in a confined space, this may be sufficient for some applications. 

	At this stage, I did experiments to see the advantage of using the next waypoint as heading instead of 
	simply using current direction of the targets. There was an advangate as the targets would face the wrong direction at
	times as they navigated over very hilly terrain. This would cause the quadcopter to need to readjust more often.  

4. **simple predictive hover (in progress)**

	The quadcopter uses the current speed of the targets and their next waypoints to create a very simple path prediction.
	The quadcopter finds a location to hover that keeps this path in view the longest before having to reposition.

5. **add stationary targets**

	The quadcopter could keep stationary points in view. A large object to keep in view could be represented as a bounding box of four points. 

6. **sophisticated prediction hover**

	Use machine learning to incorporate the actual movement characteristics of the various targets.
	For example, the paths of the EMILY ERS are far from straight, holonomic lines. Rather they swerve significantly. 
	Also could incorporate terrain characteristics. 


## environment 

The system in being developed using the Morse simulator, which makes it easy to work with a variety of robots in a 3D environment with terrain. 

Since each robot and the GCS are run with independent python scripts, using a subscriber-publisher message passing system with the REDIS NoSQL database.
MORSE is a server where I can connect to the robots, but then I can only use the functions in MORSE to work with the robots.
Where MORSE might have a "status" function with the current position of the robot, I define a tailored status function for the particular robot with other information such as "next waypoint" and "destination waypoint". 
So to keep things coordinates, only a particular robot's python script can call MORSE functions related to that robot. Other robots (and the GCS) can send messages to the robot to request information and send tasks to the robot. 
I worry that the above sentances make little sense, but I will provide a diagram soon that I hope will clarify. 

## robots

Currently two robots are defined:

- QCOORD: The protagonist, this is the "quadcopter coordinator" who will keep the targets in view as best as it can. 

- MARISA: This is a UGV (specifically an ATRV). Named to differentiate from the USV EMILY which is the real motivating robot for the research. Choose to work with UGVs for simulation since they are well supported in MORSE. Once I have everything working for the MARISA robots, will try to simulate USVs comparable to the EMILY. 


## installation and use

Will add soon. 

