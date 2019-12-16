# Final Report

## Results

My main achievement was getting mimicry control of both the arms of the Movo using the Vive controllers. I used the mimicry control package from Danny and Luis that worked on other robots and I was able to adapt it to work on the Movo with both arms. For the mimicry control to work I also created a velocity controller that works along with relaxedIK to move the arms. This controller is a standalone ROS node and can, therefore, be used by anyone who wants to control the arms without worrying about how to actually move it.

The mimicry control works by taking Vive controller positions from a project in unity (in windows), sending it to the Movo’s Linux machine over an ethernet connection and using relaxedIK to get joint angle solutions. The arms are then moved according to the solutions using the velocity controller. Running the program gives full 6DOF control of both arms of the Movo simultaneously. The arms also do not collide with each other due to the collision detection code in the velocity controller.

The velocity controller is a ROS node that takes joint angles solutions for both arms from relaxedIK and, using the current arm positions, determines how to move the arms using angular velocities. The controller also checks for collisions and doesn’t move the arm if it believes it will reach a collision state. One limitation is that the collision detector is a bit conservative and sometimes has a false-positive result. The node is reusable by anyone who wants to control the arms using relaxedIK. All the user had to do is publish end effector positions to relaxedIK while running the relaxedIK and velocity controller nodes in the background.

The files included are:
- **uw_test:** package containing source for the velocity controller as well as another ros script that does AR tracking and moves other parts of the robot
- **relaxed_ik_files folder:** Contains RelaxedIK Config folder and start_here.py file
- **mimcricry_control_scripts folder:** Contains two scripts that are changed from the original mimicry_control package. The rest of the package is still needed.
All the files are also in their respective packages on the Movo’s PC.

## Work

Most of the work I did was related to arm control on the Movo robot. I started with getting position and orientation control of the right arm using the built-in MoveIt! solver. I got the arm to follow an AR tag in the 3D space and also point towards the AR tag, although not accurately. I then worked on setting up RelaxedIK on the Movo. I also created a simple velocity controller that only controlled the right arm and did not have any collision detection. I was able to get the right arm to follow the AR tag using RelaxedIK movements. I worked a bit on getting the arm to point at the AR tag but that was not very accurate either.

I also worked on calibrating the right arm with respect to the Movo’s camera but I wasn’t able to create an accurate calibration process. I built a basic framework for the calibration but I needed a more accurate way of determining the arm’s position from the camera. I also tested out the internal kinematic data given out by the Movo and I found that it was accurate.

One of the hardest parts of working on the Movo was dealing with all the troubles the Movo had. Many of the problems were unpredictable and seemed to happen randomly. As we used the Movo more, more issues started to show up and it was frustrating to work with them. I spent a lot of time testing out what worked and didn’t work and trying to find the reasons behind the issues.

Unfortunately, while running one of my programs I broke the right arm’s gripper cuff. The right arm was moved by my velocity controller which caused it to hit the left arm and break. After that, I initially used the arms without the grippers or the cuffs and then later added the collision detection to the Movo. Maybe if I had added the collision detection before that, this could have been prevented.

## Lessons Learned

On the technical side, I learned a lot about robot control and teleoperation. I learned how to use velocity control and IK solvers to move the arms. I got a better understanding of transformations between coordinate frames and getting the right positions and orientations to send to the solver. Even though I didn’t complete the calibration, I learned a lot about the process of calibration and how to get an accurate transformation function. I got more comfortable with ROS and all the features that it has. I learned a bit about unity scripting and how the positions from the Vive controller were being sent and decoded as end effector position. Helping Pragathi with her study helped me learn about how studies are run and the process behind them. It was interesting to see the final paper at the end.

I also learned a lot about how to work better and more efficiently. I got better at having more quantifiable and goals. Having more concrete goals allowed me to focus better and also stick to a timeline as best as possible. I also learned how to collaborate better with other people. Since Sayem also worked on the Movo this semester we had a lot of overlapping parts and by splitting the work to our strengths we were able to finish the work more effectively.

## Self-Evaluation

Overall, I am very happy with how the project turned out. I feel like I have progressed from the beginning to the end of the semester and have achieved an interesting goal. I also think my work will be useful to other people in the lab and can be used directly or built upon further for other purposes. Something different I would do next time is discussing my goals and my plan more thoroughly before getting started on the work and maybe asking for more feedback along the way. The advice I would give to someone else is that if you have an issue, someone else at the lab has probably dealt with something similar and you could ask them for help on the matter.
