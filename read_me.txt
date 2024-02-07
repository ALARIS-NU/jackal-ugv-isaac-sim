source devel/setup.bash

## December 20 2023 instruction Issac

# 1) load robot description
roslaunch jackal_description description.launch 

# 2) Launch Issac simulation and start it (with front RGB-D camera 15degrees)

# 3) Start AMCL service for navigation
roslaunch jackal_navigation amcl_demo.launch map_file:=`rospack find human_detection`/map/generated_map.yaml

# 4) Paddle Paddle human detection network
source paddle_v2/bin/activate

rosrun human_detection bw_human_w_camera_info.py

# 5) Start GUI
rosrun human_detection gui.py
