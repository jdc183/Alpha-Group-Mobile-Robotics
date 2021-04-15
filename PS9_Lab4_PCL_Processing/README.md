# PS8: Navigation Enhancements in the Lab Report
This is the upload for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

## Notes 
```
roslaunch baxter_gazebo baxter_world.launchbaxter simulation
roslaunch baxter_launch_files baxter_playfile_nodes.launch 
```
canned moves/ positions that are generically useful, ready to take more predefined trajectories 		and execute them. Prepositions arms. nothing in rviz has to change. try not to hit the table or 		get in the camera view. grab stuff from the top

```
roscd baxter_playfile_nodes/ 
```
easy to construct hand moves from these play files. have to close gazebo to run these because 	they can conflict. run with the command:
```
rosrun baxter_playfile_nodes baxter_playback shy.jsp (example for the shy.jsp file)
```

## instructions on how to write jsp play files: lecture 17 at 39:32 - 59:27
(we have to write these jsp files to instruct the robot to go to the correct poses based off of the blocks)
	rostopic echo /robot/joint_states
		cannot depend on output order being consistent
	rosrun baxter_playfile_nodes get_and_save_jintervals
		gives value of left and right arms in the preferred order for the jsp files
			only concerned about the right arm
		recorded as text files with time it was saved
			look inside of it with: more filename.txt
				joint values with the correct order and appended value of 1 at the end
			copy and paste into a jsp file
			make sure your arrival times are geometrically increasing
			
## Using the bag files (lecture 17 1:00:00 - 1:03:42)
physical mary joint angles given in arm#.bag (where # is actually a number)
	rosbag play arm#.bag -l
	rostopic list
	rostopic echo /robot/joint_states
		check positions with name matches in the order we care about to find the values we want. 
		match up simulators pose to physical robots joint states and then play back a simulation of 		the point cloud display, then we will have something that looks identical to the real robot 			and 	then 	vary the values of the transform publisher	to try to reconcile the calibration of 			the vision 	with the calibration of the kinematics.  The ttransform has to be right and these 			two things must agree before we can try to pick up anything.
			
## Operation (lecture 17 at 32 min) 
roslaunch baxter_gazebo baxter_world.launch 
roslaunch baxter_launch_files baxter_playfile_nodes.launch 
			
rosrun baxter_playfile_nodes baxter_playback block_one.jsp
rosrun baxter_playfile_nodes baxter_playback block_two.jsp
rosrun baxter_playfile_nodes baxter_playback block_three.jsp
rosrun baxter_playfile_nodes baxter_playback block_four.jsp
rosrun baxter_playfile_nodes baxter_playback block_five.jsp
rosrun baxter_playfile_nodes baxter_playback block_six.jsp
rosrun baxter_playfile_nodes baxter_playback block_seven.jsp
rosrun baxter_playfile_nodes baxter_playback block_eight.jsp
rosrun baxter_playfile_nodes baxter_playback block_nine.jsp
rosrun baxter_playfile_nodes baxter_playback block_ten.jsp

```

Alternatively, you can run each of the nodes, as well as the Gazebo simulator, from separate terminal tabs or windows. ```
```
