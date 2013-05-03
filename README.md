Obstacle Navigation
-------------------
This is the source code for the UT Austin FRI project by Victor Yap, Eysa Lee, and Daniel Cheng, in the BWI research stream.

Project Description
===================
This project involves the detection and tracking of moving obstacles at varying depths using a Kinect sensor. We hope to be able to improve on path planning by giving a wider margin of avoidance to moving obstacles.

How to Run
==========
<b>Dependencies:</b> OpenNI, OpenCV

This node reads data from <pre><code>/camera/depth/image</code></pre> and publishes data to <pre><code>/obsnav/depth_blobs/threshold</code></pre> and <pre><code>/obsnav/depth_blobs/blobs</code></pre>.

To start this node, run
    rosrun obstacle_navigation depth_blobs

By default, <pre><code>/obsnav/depth_blobs/threshold</code></pre> show the depth threshold image for objects within 1 meter of the Kinect sensor. As of now, this can only be changed by modifying the <pre><code>depth_blobs.cpp</code></pre> source code and recompiling. The variables <pre><code>slice_count</code></pre> and <pre><code>slice_size</code></pre> in lines 16 and 17 determine the number of depth slices and their precision. The slice that is displayed to <pre><code>/obsnav/depth_blobs/threshold</code></pre> is chosen in line 182.

Information about the largest blob in each slice is in <pre><code>/obsnav/depth_blobs/blobs</code></pre>, which uses a custom blobs message.

Additional Info
===============
Additional information about this project can be found on the <a href="http://farnsworth.csres.utexas.edu/bwi/index.php/CS378/Obstacle_Navigation">wiki page</a>.
