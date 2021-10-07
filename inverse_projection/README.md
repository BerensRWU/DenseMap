# Inverse projection
Script to get a 3D point cloud from the depth map.
Ref: 
  * https://github.com/darylclimb/cvml_project/blob/master/projections/inverse_projection/
  * https://github.com/MankaranSingh/Auto-Depth/tree/master/inverse_projection/

Explanation:
  * https://medium.com/@daryl.tanyj/inverse-projection-transformation-c866ccedef1c
  * https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f

## Dependencies
openCV is used to load the RGB image 
```
pip install opencv-python
```
open3d is used for visualising the point clouds
```
pip install open3d
```
If you are only interested in the point cloud numpy is enough.
## How to use
```
python inverse_project.py
```
