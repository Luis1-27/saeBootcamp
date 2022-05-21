### Week 4: Advanced Emergency Braking 

### Instructions

The callback function in sae_aeb.py needs to be updated to specify which controller to run. Uncomment the function you wish to use in the code and comment out the other function. The two functions are dist_control and TTC_control

Make sure you have the cloned and created a symbolibc link for the following f1/10 repository. It contains the world and robot used for this simulation.
https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public

Once the above two items are complete you'll want to run the following commands:
$ roslaunch automatic_emergency_braking aeb.launch
$ 

#### Getting the Index

This function is used to calculate the ray index corresponding to angle 0(i.e directly in front of the vehicle)

Using the equation A = a + (n-1)*d where d is the increment for each index, n represents the index we are trying to find, a is the starting angle of the lidar FOV, and capital A
is the angle of the ray (zero) that we are trying to find.

In our code we will calculate a range using an offset of angle minimum. Then calculate the angle range of indices per radian. Then calcualate the index using the product of the angle range and the indices per radian. This will give us the index value we are looking for which contains the distance to the nearest object(wall) in front of our vehicle.
#### Getting the Distance

Normalization of the angle is needed to the exact range index for the specified angle. Angle given in degrees is converted to radians and then subtracted angle_min for offsetting. Also calculated range indices per angle. Finally multiplied the calculated angle above and range indices per angle to get the index. This index will be in ratio and so flooring to the below nearest integer will get an exact integer. This index corresponds to the desired angle for measuring the distance in scan data.
To be more certain, a set of more samples are taken around the desired angle and all those distances are averaged to get more accurate result. 

#### Dist_Controller

This function calculates the distance to collision error, which is used to slow the robot to a stop before it hits the wall. The distance error is calculated by subtracting the distance threshold (how close the robot can get to the wall)from the distance determined from the get_distance function (see section "Getting the Distance") The distance error is multiplied by a constant to calulate the velocity that is published to the robot. As the robot gets closer to the wall, the distance error decreases, which decreases the velocity. The robot slows down as the distance from the wall decreases and comes to a complete stop when the distance threshold is reached.

#### TTC_Controller

This function calculates the time to collision error, which is used to slow the robot to a stop before it hits the wall. The time error is calulated by taking the distance from the lidar topic and dividing by the odometry derived speed and subracting this quantity by the global time threshold. The if statement stores the previous state of the vehicle within time threshold to avoid noisy behavior. If time is greater than 5 seconds we want to maintain max speed if the time to collision is within 5 seconds then then start breaking. We print the time error and time to collision as well as the Vehicle velocity.Ultimately we then publish this message to a topic which is subscribed by the F1/10 simulator.

#### Preferable controller

The time collision controller is a more practical approach in real-time applications because at high speeds there may not be enough time to stop based on the distance calculation.
The time based controller considers velocity which therefore makes it more practical. 
