# robot_localization
## Project Goals
What was the goal of your project?
Our goal for this project was to accurately locate our robot's location and position on a predetermined map using real-world LiDAR data. In order to accomplish this, we used a particle filter. More broadly, a particle filter is an algorithmic approach used to accurately decide on one interpretation of uncertain range data.  

## Approach
On a high level, our particle filter generates XXX virtual 'particles' that act as explorers on the occupancy grid. Each particle starts in a random location on the virtual map, and can compute the same 'sensor measurements' that a robot would receive if it were located in that position on the map. When the robot moves, each particle virtually performs the same actions as the robot. After each movement of the robot and particles, the particles with the most similar 'sensor measurements' to the actual measurements taken by the robot are the best candidates. The particles reporting less-similar 'measurements' are moved to a location near the best candidates, with some random noise, and the process repeats. Eventually, the robot's movements will cause all the virtual location of the best-candidate particles to concentrate very closely to the actual location of the robot.

## Design Decisions
We chose to use numpy for mathematics tools like binomial distributions. We did not choose to use matrix multiplication, but we would recommend using matrix multiplication to update particle locations (x,y, and orientation). (see Lessons Learned)

## Challenges
We struggled to complete our function that re-generated particles after evaluating the best particles. Re-generation using a scaled-down version of variance for the x, y, and theta values 

## Future Improvements
If we had more time, we would refactor our code to use NumPy and matrix multiplication to perform particle updates. we would add 'moving object' constraints to the particle filter. For example, a person walking could be filtered out by merging odometry data and LiDAR data and removed if it traveled over the course of the scans

## Lessons Learned
TODO: Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.
