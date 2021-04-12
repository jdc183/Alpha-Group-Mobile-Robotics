# table_transform

Example code to find the transform for a table top from pcd images of a clear surface,
as well as code that uses this transform to find points above the tabletop.

## Example usage
Given a PCD file of a surface of interest, cleared of objects, run:

`rosrun table_transfo find_table_frame`
this will prompt for a PCD filename (so you may want to cd to this directory, or copy the PCD file to this package).

The code will write out a launch file for the transform of the table frame to camera frame.  The launch file will be written to the directory from which this code is run.  (So it would be easiest to copy the PCD file to this package, then run the code from this package, resulting in the launch file being written to this package).

Then, run the launch file to establish the computed transform on tf.

Then run the program:
`rosrun table_transform find_points_above_table`
This will prompt for a PCD file name of interest (so you may want to run this node from
a directory that contains files of interest).  The files should be PCD snapshots of
objects of interest sitting on the previously-identified tabletop.

Results can be viewed in rviz.

This program may be converted to subscribe to a pointcloud topic from the camera,
and it could be specialized to recognize objects of interest and compute their
coordinates in the table frame.  A service interface would be appropriate.


    
