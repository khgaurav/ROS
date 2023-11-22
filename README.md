# ROS
This repository consists of all the codes made on ROS. It contains:
## Codes
- <b>ball_detector</b>: Using hough circles, colour detection, and elimination
<br><br> **Issues** <br>
 1. Detects a bottom of a green bottle as ball. *Reason -* Hough circles.
 2. Does not work in bright sunlight. *Reason -* overexposure of ball.
---
- <b>Obstacle detector</b>: using PCL from realsense camera. Segment the largest plane from PCL and mark it as ground. Search the remaining part for obstacles
<br><br> **Errors** <br>
 1. Small range *Reason -* Limitation of such cameras. *Solution -* Use LiDAR
 2. Suseptible to noise, especially with sand in the air
---
- <b>rs_driver</b>: Intel Realsense access and publish point cloud without using Realsense ROS wrapper.
---
- <b>skid_velocity_control</b>: Convert linear and angular velocity to right and left motor commands for skid steered vehicles. Uses IMU data to give commands until the required velocity is matched
