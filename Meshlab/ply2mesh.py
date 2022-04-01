import os, sys
import numpy as np
import open3d as o3d
import trimesh


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
    if len(sys.argv) < 2:
        print('python3 ~.py <pcd_path> <ply_path>')
    else:
        pcd_path, ply_path=str(sys.argv[1]), str(sys.argv[2])
        main(pcd_path, ply_path)

