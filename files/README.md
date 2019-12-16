
### [Download Files](https://minhaskamal.github.io/DownGit/#/home?url=https://github.com/joshuawisc/movo-control/tree/master/files)

## Velcoity Controller
To run the velocity controller, run `rosrun uw_test movo_vel_controller.py` on the Movo along with the RelaxedIK node

## Mimicry Control
The ar_mover script contains a class with various functions for moving the various parts of the robot. Instantiating the class and calling the function will perform some behavior according to the function. eg. Follow AR tag with arm, follow AR tag with base, move head etc.

To run the mimicry_control, run the following nodes:
```shell
rosrun relaxed_ik load_info_file.py  
roslaunch relaxed_ik relaxed_ik_julia.launch
rosrun mimicry_control controller_publisher.py
rosrun mimicry control ee_pose_goal_pub_both.py
rosrun uw_test movo_vel_controller.py
```

Then click play on the unity project on Windows while both controllers are connected. If only one controller is connected it will still work with only the right arm.
