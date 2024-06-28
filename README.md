# 3D-Mapping-using-2d-LiDAR
 A 3D Mapping Algorithm that generates PointCloud2 Messages which can be used with OctoMap or Cartographer to visualise 3D Voxels on Rviz/Rviz2
In order to use these files these are the steps that must be followed:
1. Check the PIN Configuration Used for the PWM in the Rasberry Pi
2. Launch the reference.py # You may need to use sudo pigpiod for a LINUX based OS
3. Launch rot_ros2.py for continuous rotation of the servo 
4. For Displaying the coordinates you may use cod_ros2.py or for direct publishing you should use pcl2.py

Key Notes: I used a Rasberry Pi 4 with an MG996R Servo and RP LiDAR A1-M8R6 that pucblishes the laser scan topic under the name \scan so be careful with your own naming standards.
A platform was also designed that allowed the LiDar to be stuck on so that the servo rotates it.