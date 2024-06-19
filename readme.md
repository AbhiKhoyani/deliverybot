# DeliveryBot for DFL 
### (Partially done!!)

This project contains the necessary moduels for deliverybot. Deliverybot has 2 RGBD camera mounted on body of Turtlebot3-burger. RGBD camera's horizontal field of view is 84\degree. Considerung this, both the camera are mounted with yaw with +/-42\degree to maximize the field-of-view and minimum overlap. The constraint in this mounting approach is that there will be minimal overlaping with both camera's FOV, which makes extrinsic calibration process of both camera with each other difficult. I'm planning to reduce the yaw angle to have suufcient overlapping to include target object for calibration but for now, I've followed this configuration for the rest of the application.

TODO: As of now, I was able to finish this assignment partially only considering various reasons. Out of five main component of the assignment, I was able to build a robot with 2 RGBD camera, calculate intrinsic calibration, perform mapping using SLAM and calculate cost-map using it, and finally navigation. Extrinsic calibration, various test cases to check all of the above task, and CICD build pipeline is remained. For the SLAM, and navigation also, I was able to verify it partially because of the limitation of my machine. As, when I'm launching robot, gazebo, rviz, altogether with SLAM ododmetry and mapping task, RGBD driver is publishing 6-8 frames only `image_raw` and `/depth/image_raw` [See this Issue](https://github.com/introlab/rtabmap_ros/issues/1054) which fails `rgbd_sync` node so further pipeline won't work at all if there's no enough images. However, as per the code, SLAM and navigation both should work with better system as I tried to run them using wheel odometry.

To build the project:
```
> cd ROS2
> git clone https://github.com/AbhiKhoyani/deliverybot.git src/deliverybot
> colcon build --packages-selected deliverybot
```


This bot has the following modules incorporated.
1. **Main Robot**: As described above, main robot with 2 RGBD camera. Check description file in `./urdf/deliverybot.urdf.xacro` and `./models/model.sdf`. I've treid to include Intel Realsense D435 RGBD camera, but it's driver may not allow multiple camera so I've used depth camera drivers from `gazebo_ros_pkgs`. To launch the robot in gazebo and rviz run `ros2 launch deliverybot deliverybot_house.launch.py`, that will launch the robot in Turtlebot3_House world and show respective rviz with camera, pointcloud, TF frame, and robot description.

2. **Intrinsic Calibration**: For now, intrinsic calibration is designed to perform interactively. During test designed, it can be changed to run without user input, by saving images first from respective image topic and then run `cameracalibrator` offline as defined in [tarfile_calibration.py](https://github.com/ros-perception/image_pipeline/blob/humble/camera_calibration/scripts/tarfile_calibration.py).
    - To run, intrinsic calibration interactively, launch `intrinsic_calibrate_left.launch.py` in one terminal, which will spawn deliveryBot and a checker-board and then run `./scripts/calibrate_left.sh` in another terminal.  `calibrate_left.sh` scripts will move checker-board in various position in FOV of left camera with different angles. Once, all the poses of checker-board are captured in `cameracalibrator` node, `CALIBRATE` button will be activated and pressing it will calculate the intrinsic parameters of the left camera. Press `SAVE` button to save params in `/tmp/calibrationdata.zip` file.
    - Repeat the same procedure for the right camera.
    - #TODO: For making this procedure automatic to make it suitable it for CI/CD pipeline, a single script file shall be written that will first launch `intrinsic_calibrate_left.launch.py` with given delay run `./scripts/calibrate_left.sh` once complete, measure the difference of the intrinsic values calculated by cameracalibrator and `/camera/camera_info` topic in a test.   

3. **Extrinsic Calibration**: *Not implemented yet!* Started with [multi_lidar_calibration](https://github.com/AbhiKhoyani/multi_lidar_calibration_ros2) repo, but as I've set RGBD camera to have minimum overlapping, It won't work for now. I'll have to increase the overlapping FOV and then use this. Ran out of time.

4. **Mapping using RTABMAP**: See `./launch/rtabmap.launch.py` file. Here, I'm basically, syncing RGB and depth image using `rgbd_sync` node for both of the camera and feeding those data to `rgbd_odometry` and `rtabmap_slam` for `/odom` and `/map` frame data. Utlizing two camera by setting `rgbd_cameras:=2` in parameters.

5. **Navigation using Nav2**: Refer `./launch/nav2_bringup.launch.py` Haven't made much modification in params file yet for RGBD use cases! Shall be work fine with LaserScan data.

6. **Dockerfile**: Finally, have prepared a Dockerfile that will be able to prepare an image of ROS2-Humble with all the necessary dependencies and code for the deliverybot. This Dockerfile can be utilized to prepare CI/CD build pipeline with the respective action test cases for each of the above task.