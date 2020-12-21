# Get a dense Depth map from sparse LiDAR point clouds

This repository contains a short python script to get a dense depth map from a LiDAR point cloud for a known camera. The original code is written in matlab and is from balcilar(https://github.com/balcilar/DenseDepthMap).

In dense_map.py is the function to generate the dense depth map (depth_map) and in main.py the code is used on the KITTI dataset.

depth_map gets the projected LiDAR point cloud, the size of the camera image and the grid size. A grid size of 4 means a 9x9 neighbourhood is used and weighted depth information is calculated according to the distance of the neighbourhood. 

![](https://github.com/BerensRWU/DenseMap/blob/main/image/depth_maps.png)
