# ROS and PCL preprocess for creating 3D PointCloud ML dataset

This is the Pipeline to create 3D PointCloud Dataset for Machine learning from Lidar. 

Mainly using ROS, this includes realsense-lidar, capture depth-image, crate PCD(Point Clod Data), PCL preprosecing, meshlab annotation .etc .

# Version
- Ubuntu 20.04
- ROS noetic

## Setup
```zsh
# realsense related pkg install
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
$ sudo apt update && sudo apt upgrade
$ sudo apt-get install librealsense2-dkms librealsense2-utils
$ sudo apt-get install librealsense2-dev librealsense2-dbg
$ git clone <this repogitory>
$ sudo apt-get install ros-noetic-ddynamic-reconfigure
$ sudo apt-get install ros-noetic-realsense2-camera

# catkin make
cd ~/catkin_ws
#catkin_make clean
#catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make

# install python pkg
pip3 install open3d trimish
```

## Network Environment and Entire pipeline

<b>Network Environment</b> 

<img src="https://user-images.githubusercontent.com/48679574/161230708-b3bb5b9a-262c-4726-8236-87286e25f390.png" width="400px">

<b>Entire pipeline</b>

<img src="https://user-images.githubusercontent.com/48679574/161230782-f59f0f0a-7c83-48fe-93dc-31fc61e172df.png" width="650px">



# PCL Preprocess

<b>raw PCD => filter => planar_segmenter => downsampler => clusterer</b>

<img src="https://user-images.githubusercontent.com/48679574/161231084-a455dd3e-d695-41a2-a6e8-9c7ae9c1a6c1.png" width="200px"><img src="https://user-images.githubusercontent.com/48679574/161231104-fa4338d0-e88a-4e34-b2d5-b6f551950d36.png" width="200px"><img src="https://user-images.githubusercontent.com/48679574/161231116-a115ed70-098e-4a2f-a501-0ea991285744.png" width="200px"><img src="https://user-images.githubusercontent.com/48679574/161231121-2f8ae135-a7ec-4543-9d75-adb2c5dae582.png" width="200px"><img src="https://user-images.githubusercontent.com/48679574/161231137-6de017c5-91ea-4579-a0d3-91280e090d03.png" width="200px">


# MeshLab

after PCL preprocessing, convert ply to mesh and load mesh by [MeshLab](https://www.meshlab.net) for annotation to create ML 3dPointCloud Dataset.

```zsh
# ubuntu MeshLab install
sudo apt update
sudp apt install meshlab
```
<b>convert PLY to Mesh</b>

```python
import os, sys, trimesh
import numpy as np
import open3d as o3d

def o3d_create_mesh(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    pcd.estimate_normals()
    # estimate radius for rolling ball
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist   

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd,
            o3d.utility.DoubleVector([radius, radius * 2]))

    # create the triangular mesh with the vertices and faces from open3d
    tri_mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                            vertex_normals=np.asarray(mesh.vertex_normals))
    trimesh.convex.is_convex(tri_mesh)
    return tri_mesh

def main(pcd_path, ply_path):
    polygon_mesh = o3d_create_mesh(pcd_path)
    polygon_mesh_ = trimesh.exchange.ply.export_ply(polygon_mesh, encoding='ascii')
    output_file = open(ply_path, "wb+")
    output_file.write(polygon_mesh_)
    output_file.close()

if __name__=='__main__':
    pcd_path, ply_path=str(sys.argv[1]), str(sys.argv[2])
    main(pcd_path, ply_path)
```

# Create 3D PointCloud ML Dataset after Meshlab

Export OFF file format by Meshlab and load by numpy and keras to use ML dataset

```zsh
cd Meshlab
# numpy load
pyhton3 load_mesh.py <off file>

# Keras load
python3 keras_load_mesh.py <off file>
```

# References
- [Point cloud classification with PointNet](https://keras.io/examples/vision/pointnet/)
- [github-pointnet](https://github.com/charlesq34/pointnet)
- [Point Cloud Library tools](https://github.com/PointCloudLibrary/pcl/tree/master/tools)
