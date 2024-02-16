Requirements: Isaac Sim , ROS Noetic
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-rosserial ros-noetic-sick-tim
ros-noetic-odom2tf ros-noetic-nmea-messages ros-noetic-pointcloud-to-lasescan
ros-noetic-pointgrey-camera-driver ros-noetic-catkin-simple
```

Importing Jackal UGV to Isaac Sim

Import URDF

Prepare URDF file of a skid steer vehicle. 

Launch Isaac Sim . In the toolbar click Isaac Utils, then go to Workflows, then URDF importer. If there is no URDF Importer, click Window on the toolbar and click Extensions. In the new window, search for ISAAC SIM URDF IMPORTER and toggle the slider near DISABLED label.
In URDF Importer tab, uncheck Fix Base Link box and change Joint Drive Type to Velocity

the file selection dialog box, navigate to the desired folder, and select the desired URDF file.

Click the Import button to add the robot to the stage.

That creates the prim of your skid steer vehicle in the simulated environment. One step that is frequently required is to check the articulation root of the mobile robot. It is frequently assigned to the whole prim, while for imported vehicles it is better to assign the Articulation Root for the base_link frame.
Click on the prim of your vehicle in the Stage Tab and scroll down in the Property Tab to the Physics Tab and click on the red cross to the Articulation Root. Do not do it while simulation is active, because it crashes Isaac Sim.
Then Right click on the base_link , click Add->Physics->Articulation Root

The robot is ready to be connected to ROS
	
Graphs for SLAM in ROS
To control robot in Isaac Sim via ROS , we need to build graphs  to control velocity of the wheels

![image1](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/3c84e666-5334-4687-adba-f70c5a3c9f4c)

Fig 1. Complete twist control of skid steer vehicle in Isaac Sim


That tutorial is very close to twist control of the differential robot in Isaac Sim, the only difference is that you need more joints to control - the velocity on front and back wheels is going to be the same. You can check the validity of your graph via teleop_twist_control or rqt_gui.
The other thing that we need is odometry and transform nodes, lidar and camera. Tick Reset On Stop box in every Isaac Read Simulation Time OmniGraphNode to avoid time conflict between ROS nodes.

![image2](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/1f5ba06a-bc09-48f5-a7e1-46f693c5ddc0)

Fig 2.Odometry and Transformation node
![image3](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/7abf1ee5-e005-4d1f-8a1e-947a37795d17)

Fig 3. Lidar node
Lidar must be connected to the existing frame of the robot.
![image4](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/9810790f-9cd1-4436-82da-e5deac126a48)

Fig 4. Camera node with RGB and Depth cameras.

Physics scene

The other problem that you will encounter is that the robot does not act like a real robot or robot in GazeboSim. It cannot reach its destination in Isaac Sim, circling around it. The fix is to fine tune the physics scene in Isaac Sim. Click on physicsScene in the Stage tab and change parameters in Scene and Advanced subtabs to the shown below. You can increase time Steps Per Second more, but it slows the performance. 
Another part is to assign a material to the skid-steer vehicle’s wheels. Create a material, and assign it to every wheel link of the mobile robot. That would fix rocking in motion.

![image5](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/2de319ff-594d-444e-93cd-7506c7924cf2)
![image6](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/4a8f06f7-db9f-472a-b291-6d8c7a44ecd7)
![image7](https://github.com/ALARIS-NU/jackal-ugv-isaac-sim/assets/63298970/6878b002-9645-4a40-a120-a0993f4130bb)




Fig 5. The material parameters.
