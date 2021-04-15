# PS8: Navigation Enhancements in the Lab Report
This is the upload for the alpha group (Nicole Graf, Joseph Cressman, and Andrew Capelli).

## Notes 
roslaunch baxter_gazebo baxter_world.launch
baxter simulation
roslaunch baxter_launch_files baxter_playfile_nodes.launch 
canned moves/ positions that are generically useful, ready to take more predefined trajectories and execute them. Prepositions arms. nothing in rviz has to change. try not to hit the table or get in the camera view. grab stuff from the top
roscd baxter_playfile_nodes/ 
easy to construct hand moves from these play files. have to close gazebo to run these because they can conflict. run with the command:
	rosrun baxter_playfile_nodes baxter_playback shy.jsp (example for the shy.jsp file)


## Operation lecture 17 at 32 min 
roslaunch baxter_gazebo baxter_world.launch 
roslaunch baxter_launch_files baxter_playfile_nodes.launch 





```

Alternatively, you can run each of the nodes, as well as the Gazebo simulator, from separate terminal tabs or windows. ```
```
