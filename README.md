# KU AIRS: SPARK Autonomous Vehicle Project

Check this [link](https://eneserciyes.github.io/projects/spark/) for the blog post describing this project and the video of SPARK in simulation and on parkour. 

## Project Structure
SPARK code base was written on ROS Noetic using Python and C++. The code is structured into packages with specific responsibilities. The repository also includes driver packages for the sensor suite. 

### Packages
 
- *controllers:* Contains PID and Stanley controller for waypoint following
- *hardware_interface:* Interface to Arduino microcontrollers in the car. Accepts messages controlling steering, throttle, brake and gearshift commands. 
- *joy_controller:* Contains the code for controlling SPARK with a joystick
- *lane_detector:* Implementation of LaneNet instance segmentation model for lane detection. Modified for the use case and includes trained models on TUSimple dataset augmented with our custom real and synthetic data. 
- *lidar_cam_calibration:* Sensor fusion package for fusing LIDAR and camera data to localize traffic signs in 3D. 
- *lidar_following:* Contains the navigation stack using LIDAR mapping.
- *localization:* Contains the localization stack using RTK-GPS data.

- *planners:* Contains finite state machine based behavioural planner and cubic spiral lattice planner for parking and waypoint navigation
- *spark:* Contains launch files for system startup
- *spark_description:* Contains environment and robot descriptions for use with Rviz and Gazebo
- *spark_msgs:* Contains all custom message and service definitions.
- *visualization:* Rviz visualization scripts
- *yolov5:* YoloV5 implementation and ROS node trained on custom Turkish traffic signs dataset for real time traffic sign and light detection. 



<!-- KU AIRS: SPARK is a team founded in 2020 in Koç University, Istanbul with the goal of providing a place where students who wishes to go beyond courses to create significant projects can gather. First major project of SPARK team was building an autonomous car for Teknofest Robotaxi competition. Among 30 finalists, SPARK became one of the 5 teams that was able to complete the parkour.  -->

