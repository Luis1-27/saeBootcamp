### Week 6: Lane Keeping

### Instructions

Make sure you have the cloned and created a symbolibc link for the following f1/10 repository. It contains the world and robot used for this simulation.
https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public

Once the above two items are complete you'll want to run the following command:

$ roslaunch lane_keeping lane_keep.launch

#### Canny Function

The canny function used openCV to extract the data the camera captures and export it as an image. The lines are extracted as COLOR_BGR2GRAY.

#### Region of Interest

The region_of_intrest function crops the image produced from the canny function. Four coordiates are input into an array. These will be the four corners of a rectangle. This rectangle is masked over the image to crop it to the specific wanted region.

#### Extract Hough Lines and Center Line

The Hough Lines are extracted using the get_line_points and average_slope_intercept functions. These functions calculate two points using y=mx+b and then create a line based on those two points. Based on if the slope is positive or negative, the lines will be classified as being on the left or right lane. This is performed constantly and will create many lines. All of the left lane lines will be averaged to output a single line to represent the left lane. The same is done for the right lane.

The center line is determined by taking the average of the left lane line and the right lane line.

#### Controller

The controller for this script will adjust the steering angle of the vehicle based on the error of the desired centerline angle vs the actual centerline angle. The angle of the actual centerline is calculated taking the inverse tangent of the slope of the centerline. This angle is subtracted from the desired angle to calcualte the error. The error is then multiplied by a calibratable value to determine the steering angle. If the actual angle is greater than the desired angle then the vehicle wil turn right. If the actual angle is less than the desired angle then the vehicle wil turn left.

