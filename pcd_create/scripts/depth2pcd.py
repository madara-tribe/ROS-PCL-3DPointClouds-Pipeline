#!/usr/bin/env python3

from bag2depth import arg_parsar 
import open3d as o3d
import matplotlib.pyplot as plt
import glob, os

def plot_pcd(rgbd_image):
    plt.subplot(1, 2, 1)
    plt.title('Redwood grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('Redwood depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()

def main(out_dir):
    pcd_dir = os.path.join(out_dir, "pcds")
    if not os.path.isdir(pcd_dir):
        os.makedirs(pcd_dir)
    color_raw = glob.glob(os.path.join(out_dir, "rgb/*.jpg"))
    color_raw.sort()
    depth_raw = glob.glob(os.path.join(out_dir, "depth/*.png"))
    depth_raw.sort()
    count = 0
    for rgb, dep in zip(color_raw, depth_raw):
        rgbimg = o3d.io.read_image(rgb)
        depthimg = o3d.io.read_image(dep)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbimg, depthimg)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.io.write_point_cloud(os.path.join(pcd_dir, "open3d_{}.pcd".format(count)), pcd)
        count += 1
        print('saving {} pcd file'.format(count))
    print('total pcd files are ', count)
    plot_pcd(rgbd_image)
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    args = arg_parsar()
    main(args.output_path)

