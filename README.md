# Package Usage

In order to use the package, please place it in your ROS Noetic catkin workspace source folder

In a terminal, cd to your catkin workspace and run 
```bash 
catkin_make
```
After the package is built, source the build using
```bash 
source devel/setup.bash
```
To execute the launch file, run
```bash
roslaunch vision_challenge locator.launch
```
This will automatically launch both the configuration node, solver node, as well as RVIZ

To display the frames in RVIZ, please be sure to add 'TF' to your displays section and set the fixed frame in global setting to the 'world' frame
Sample images of the RVIZ screen can be found in the results.png and results2.png

## Approach
My approach to this problem was a straightforward triangulation approach.

Initially, the configuration node loads the camera_intrinsics yaml file as well as initializing the configuration messages. The configuration node also displays the world frame, robot1_cam, and robot2_cam in RVIZ while calling the solve_position service to communcate with the solver node. Meanwhile the solver begins computing the location of the bird using triangulation between both cameras.

Utilizing the intrinsic camera properties provided, the solver node creates the intrinsic camera matrix K. As we have a linear pinhole camera, the principal point of the camera will be at the center, which can be obtained by taking half of the camera resolution width and camera resolution height. The x and y focal length are both set to 270 and a pixel density of 1 is assumed. However, these can all be changed thorugh the Camera Intrinsics YAML file.

Next, the solver node creates the extrinsic matrix for each camera based on their respective rotations from Y, which is a rotation around the Z-axis (as it is a yaw rotation). The translation components were simply the current camera coordiantes as the world origin is at (0,0,0) and the coordiantes given are the camera center.

Now having both the intrinsic camera matrix, K, as well as ther respective extrinsic matrixes for each camera, I multiplied both matrices together to get the Projective Camera matrix for each drone (P1 and P2). Using the Projective Matrices and the pixel coordinates of the recognized object, we construct a matrix A consisting of a linear system of equations based on the epipolar lines through each camera. In order to find the 3D point location of the object, we would like to see where these epipolar lines intersect. Through SVD decompostition we set Ax = b and solve for the intersection. The resulting 4 dimensional point then is normalized by its last component (the scaling factor) to get our 3D point representation.

Upon computing the location of the bird, the solver node sends a validation service request to the configuration node to validate the answer. In order to perform validation, the config node reprojects the location received from the solver node to each of the cameras to obtain the pixel coordinate representation once again. The reprojection is done by multiplying the location of the bird by each camera's respective projection matrix. This results in a 3D vector which we similarly normalize by divided the whole vector by the third component. Once the pixel reprojections are calculated, we find the norm between the actual pixel values (d1 and d2) and our reprojected pixel values, ensuring they are within a certain threshold. If the values are within the threshold, a validation success is sent to the solver node which then prints the calculated bird location to ROS LOG. The configuration node then displays the tf frame for the bird in RVIZ.
