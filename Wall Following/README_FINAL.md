### Week 4: Advanced Emergency Braking 

### Instructions

Make sure you have the cloned and created a symbolibc link for the following f1/10 repository. It contains the world and robot used for this simulation.
https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public

Once the above two items are complete you'll want to run the following commands:
$ roslaunch wall_following wall_following.launch

#### Getting the Index

The get_index function is used to calculate the ray index corresponding to angle 0(i.e directly in front of the vehicle)

Using the equation A = a + (n-1)*d where d is the increment for each index, n represents the index we are trying to find, a is the starting angle of the lidar FOV, and capital A
is the angle of the ray (zero) that we are trying to find.

In our code we will calculate a range using an offset of angle minimum. Then calculate the angle range of indices per radian. Then calcualate the index using the product of the angle range and the indices per radian. This will give us the index value we are looking for which contains the distance to the nearest object(wall) in front of our vehicle.

#### Getting the Average Distance

The avg_distance function calculates average distance based on the index provided from the get_index function. Normalization of the angle is needed to the exact range index for the specified angle. Angle given in degrees is converted to radians and then subtracted angle_min for offsetting. Also calculated range indices per angle. Finally multiplied the calculated angle above and range indices per angle to get the index. This index will be in ratio and so flooring to the below nearest integer will get an exact integer. This index corresponds to the desired angle for measuring the distance in scan data.
To be more certain, a set of more samples are taken around the desired angle and all those distances are averaged to get more accurate result. 

#### Calculating the Distance Error

The distance function calculates the distance the vehicle is from the wall. Based on the calulated distance an error is calculated by performing "desired_value - actual_value". The distance from the wall is determined by performing geometry. The needed angle is calculated using the distance from the wall for 90 degrees from the front vehicle and 30 degrees fro the front of the vehicle. This function can be used to determine the distance and error from a wall either on the right or left of the vehicle.

#### Follow the Center Line

The follow_center function calls the distance function twice. Once for the right wall and another time for the left wall. After the error is calculated for both walls, the error for the center line is determined by subtracting the right wall error from the left wall error. This new center line error is then used as an input when the control function is called in the callback function.

#### Controller Function

The control function is used to control the vehicle. This entails turning left, turning right, and changing the velocity. Based on if the error is negative or positive, the vehicle will turn left or right. The steering angle is determined by using a PID controller. The error is multiplied by a calibratable value, added to an integral term, and also added to a derivative term. Based on if the steering angle is positive or negative, the vehicle will turn left or right. The error being positive or negative plays a big factor in if the vehicle turns left or right.
The steering angle is multiplied by another calibratable value to determine the velocity of the value. The as the steering angle increases, the velocity decreases, so the turns can be made.
