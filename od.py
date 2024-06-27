import open3d as o3d
import numpy as np
# 加载
def load_normals_from_txt(file_path):
    """
    从文本文件中加载点云及其对应的法向量
    :param file_path: 文本文件路径
    :return: 点云和法向量，形状分别为 (N, 3)
    """
    # data = np.loadtxt(file_path)
    data = np.loadtxt(file_path, delimiter=',')
    points = data[:, :3]
    normals = data[:, 3:6]
    return points, normals

def visualize_normals_with_open3d(points, normals):
    """
    使用 Open3D 可视化点云及其对应的法向量
    :param points: 点云，形状为 (N, 3)
    :param normals: 法向量，形状为 (N, 3)
    """
    # 创建 Open3D 点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    
    # 设置点云颜色为蓝色
    pcd.paint_uniform_color([0, 0, 1])
    
    # 创建法向量的线段对象
    line_points = []
    lines = []
    colors = []
    for i, (point, normal) in enumerate(zip(points, normals)):
        start = point
        end = point + (normal / np.linalg.norm(normal)) * 0.01 # 归一化并调整法向量长度
        line_points.append(start)
        line_points.append(end)
        lines.append([2 * i, 2 * i + 1])
        colors.append([1, 0, 0])  # 红色c

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.array(line_points)),
        lines=o3d.utility.Vector2iVector(np.array(lines)),
    )
    line_set.colors = o3d.utility.Vector3dVector(np.array(colors))

    # 创建坐标系
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

    # 可视化点云和法向量
    o3d.visualization.draw_geometries([pcd, line_set])

file_path = 'airplane_0016.txt' 
# file_path = 'chair_0015_normals.txt' 
points, normals = load_normals_from_txt(file_path)
visualize_normals_with_open3d(points, normals)
