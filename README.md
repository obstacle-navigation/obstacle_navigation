Obstacle Navigation
===================
This is the source code for the UT Austin FRI project by Victor Yap, Eysa Lee, and Daniel Cheng, in the BWI research stream.

Project Description
-------------------
This project involves the detection and tracking of moving obstacles at varying depths using a Kinect sensor.

How to Run
----------
<b>Dependencies:</b> OpenNI, OpenCV

This node reads data from '/camera/depth/image' and publishes to '/obsnav/depth_blobs/threshold' and '/obsnav/depth_blobs/blobs'.

To start this node, run
<pre><code>rosrun obstacle_navigation depth_blobs</code></pre>

The topic '/obsnav/depth_blobs/threshold' displays a depth threshold image. By default, it shows objects between 0 and 1 meters of the Kinect sensor. As of now, this range can only be changed by modifying the 'depth_blobs.cpp' source code and recompiling. The variables 'slice_count' and 'slice_size' in lines 16 and 17 determine the number of depth slices and their precision. The slice that is displayed to '/obsnav/depth_blobs/threshold' is chosen in line 182.

Information about the largest blob in each slice can be found in the topic '/obsnav/depth_blobs/blobs', which uses a custom blobs message.

Future Improvements
-------------------
Possible future improvements include:
* Tracking moving obstacles
* Using motion information at each depth to update the cost map used for autonomous navigation

Additional Info
---------------
Additional information about this project can be found on the <a href="http://farnsworth.csres.utexas.edu/bwi/index.php/CS378/Obstacle_Navigation">wiki page</a>.
