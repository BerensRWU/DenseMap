import cv2
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
from geometry_utils import *

cur_id = 0

root = "data/"
image_dir = os.path.join(root, "image_2")
depth_dir = os.path.join(root, "depth_map")
## Load images
rgb = cv2.cvtColor(cv2.imread(os.path.join(image_dir, "%06d.png" % cur_id)), cv2.COLOR_BGR2RGB)

# Depth is stored as float32 in meters
depth = np.load(os.path.join(depth_dir, "%06d.png" % cur_id))
depth = depth.reshape(depth.shape[0], depth.shape[1])
# Get intrinsic parameters
height, width, _ = rgb.shape
K = intrinsic_from_fov(height, width, 105)  # +- 45 degrees
K_inv = np.linalg.inv(K)

# Get pixel coordinates
pixel_coords = pixel_coord_np(width, height)  # [3, npoints]

# Apply back-projection: K_inv @ pixels * depth
cam_coords = K_inv[:3, :3] @ pixel_coords
cam_coords = cam_coords * depth.flatten()
rgb = rgb.reshape((height*width, 3)).transpose()

# Limit points to 150m in the z-direction for visualisation
rgb = rgb[:, np.where(cam_coords[2] <= 95)[0]]
cam_coords = cam_coords[:, np.where(cam_coords[2] <= 95)[0]]

np.save('coords', cam_coords)
np.save('texture', rgb)

def custom_draw_geometry(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

def change_background_to_black(vis):
        opt = vis.get_render_option()
        #opt.background_color = np.asarray([0.1, 0.1, 0.1])
        return False    

# Visualize
pcd_cam = o3d.geometry.PointCloud()
pcd_cam.points = o3d.utility.Vector3dVector(cam_coords.T[:, :3])
# Flip it, otherwise the pointcloud will be upside down
pcd_cam.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
colors = np.ones(cam_coords.T[:, :3].shape)
#pcd_cam.colors  = o3d.utility.Vector3dVector(colors.astype('float64'))
pcd_cam.colors  = o3d.utility.Vector3dVector(rgb.T.astype('float64') / 255.0)

### open3d > 0.10.0 ubuntu 18
# cl, ind = pcd_cam.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
# inlier_cloud = pcd_cam.select_by_index(ind)

#cl1, ind1 = pcd_cam.remove_radius_outlier(nb_points=5, radius=0.5)
### open3d 0.7.0 ubuntu 16.04
# cl, ind = o3d.geometry.statistical_outlier_removal(pcd_cam, nb_neighbors=20, std_ratio=2.0)
# inlier_cloud = pcd_cam.select_by_index(ind) not found in 0.7.0

o3d.visualization.draw_geometries_with_animation_callback([pcd_cam], change_background_to_black)
