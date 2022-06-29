### Week 10: Pure Pursuit (Carrot Planner)

### Instructions
Make sure you have the cloned and created a symbolibc link for the following f1/10 repository. It contains the world and robot used for this simulation.
https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public

Be sure run chmod u+x pure_pursuit.py and waypoint_logger.py to ensure the scripts are executable.

To run code, first select which map you want to do. Open the launch files and change the track in both.

You need to make a set of points first. To do so run

<code>roslaunch pure_pursuit purepursuit_points_logger.launch </code>

after you have teleoped a full lap, hit control c. This will create a file called new_file.csv in the waypoints folder. 

Next run 

<code>roslaunch pure_pursuit purepursuit_control.launch </code>

**NOTE: If you get some errors about the arrows, please comment out the if statement starting at line #213**

### Pure Pursuit Interpolation
Pure Pursuit works by selecting a waypoint that is always one lookahead distance away from your current position. However, in the list of waypoints, there is probably not a point that is exactly one lookahead distance away. So you must interpolate between a set of points to find one that is. 

The first step is to take two waypoints that are closest to you. Create a line that connects the points together. Next, draw a circle with your current position being the origin. The radius of the circle is the lookahead distance. The line that connected the two waypoints will now intersect the circle (in most likely 2 places). These are the roots. Both of these points are exactly one look ahead distance away from the current position. To find these points, you must first insert equation (1) into equation (2). Next you will get a long quadratic function. You can group the variables together so that the quadratic function is in x. Once you have that done, you can then solve for the roots using $-b \pm \sqrt{-b^2-4ac} \over 2a$.

$y_{root} = mx_{root} - (mx_1 - y_1) (1)$

$(x-x_c)^2 + (y-y_c)^2 = {l_d}^2 (2)$

The points have now been interpolated.

### Pure Pursuit Controller

The controller itself has two separate controllers. The first controller is for imaginary solutions while the second utilizes the Pure Pursuit algorithm. If there is no solution to the interpolation above, it is assumed that there is an imaginary solution. When this is the case, the next coordinates used are the next ones in the list of waypoints. 

#### Imaginary Solution
If the interpolation calculation does not return a solution, the next waypoints are used for the target points. To move the vehicle to these points, a simple P Controller is used. 

To determine the steering angle, you must solve for $\alpha$ which is the angle of the current position relative to the target point. 

$\alpha = \tan^{-1}({targety - y_c \over targetx - x_c}) - \gamma$

Where $\gamma$ is the yaw (angle of vehicle relative to the global x axis).

Once the angle is found, it is multiplied by a tuned constant and passed to the driving controller. The velocity can also be tuned using the distance from the next waypoint.

#### Look Ahead Controller

After the target lookahead points have been interpolated, the angle is found just as it is above. There is a second step for the steering angle however. You must find $\delta$. This value is the final steering angle with constant velocity. It also uses the lookahead distance and the wheelbase to calculate the steering angle. 

$\delta = \tan^{-1}({2\sin(\alpha)\*W_B \over l_d})$

After the vehicle makes it through every waypoint, you have successfully completeed the track!

