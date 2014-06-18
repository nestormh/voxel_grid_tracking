Voxel and Particle Filter based object tracking
===============================================

Method able to detect the location and estimate the motion of obstacles, using just a 3D point cloud and odometry information as input. For that, we do a simplification of the world based on voxels, supported by a particle filter based 3D object segmentation and motion estimation scheme. This allows a fast and reliable object detection, for which we will know also their motion speed and directions.

Although we have focused our tests on the use of 3d point clouds obtained from a pair of images, a key feature of this approach is that we do not need color information, allowing the use of many different sensor types able to generate a point cloud, or even a combination of them. In fact, we have implemented our approach in a modular way, allowing changing information sources (both the input point cloud and the odometry information) without modifying anything.

