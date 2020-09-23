# Warmup Project
## Nathan Estill

### Square

The first problem is to create a node that moves the Neato in a 1m by 1m square. I did this by using the odometry of the Neato. There are essentially two states: driving forward and turning. Before the first state starts, the location of the Neato is recorded. During the driving forward state, a message to drive forward is continuously sent to the Neato. Every tick, the distance the Neato has moved is calculated using the odometry and the previously recorded position. If the distance is greater than 1 meter, it records the current orientation and moves to the turning state. During the turning state, a message to turn is continuously sent to the Neato. Every tick, the degree the Neato has turned is calculated using the odometry and the previously recorded orientation. If the degree is greater than 90 degrees, it records the current position and moves back to the driving forward state. Thus, the steps repeat, causing the Neato to move in a square.

### Wall Follow

The next problem is to create a node that follows the wall using the LIDAR sensor. For each tick, an average of all scans from 30 degrees to 60 degrees are averaged to form the front scan, and all scans from 120 degrees to 150 degrees are averaged to form the back scan. If the front scan average is larger than the back scan, then the robot will move forward and slightly to the right. If the front scan average is less than the back scan, then the robot will move forward and slightly to the left. The respective actions will eventually make the robot follow along the wall.

### Person Follow

This problem is to follow a person that is in front of the robot. To find the person, the smallest distance from the scan is translated to Cartesian coordinates in the Neato reference frame. The X coordinate is fed into the linear velocity. This results in a faster speed if the person is farther and slower as it approaches the person. The Y coordinate is fed into the angular velocity. This results in the robot turning towards the person, lowering the Y coordinate until the person is directly ahead of the robot. Both of these combined allow the robot to turn towards the person while moving forward as well.

### Obstacle Avoidance

The penultimate problem is to create a node that will navigate through obstacles in order to reach a goal. To do this, I used the concept of a potential field. Each point in the laser scan that is within a certain range causes a vector to be applied to the robot away from it. The goal that is chosen applies a large vector towards it, even from far away. These vectors are summed up to get hte vector that the Neato will follow, using the same vector to twist command as the Person Follow section. In this way, the robot will naturally stray away from obstacles and go towards the goal.  
