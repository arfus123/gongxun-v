import numpy as np
import cv2
import matplotlib.pyplot as plt

# 相机内参
fx = 768.6145
fy = 769.0561
cx = 681.7078
cy = 372.4581

# 相机画面尺寸
width = 1280
height = 720

# 畸变参数
k1 = 0.1105
k2 = -0.1373
p1 = 0.0005
p2 = -0.0013
k3 = 0.0178

# 相机位置和方向
camera_height = 0.20  # 相机距离地面0.20米
tilt_angle = 30  # 倾角30度
tilt_rad = np.radians(tilt_angle)

# 计算旋转矩阵 R
R = np.array([
    [1, 0, 0],
    [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
    [0, np.sin(tilt_rad), np.cos(tilt_rad)]
])

# 计算平移向量 T
T = np.array([0, 0, camera_height])

# 创建相机内参矩阵 K
K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
])

# 计算每个像素点到地面的距离
distances = np.zeros((height, width))

for y in range(height):
    for x in range(width):
        # 像素坐标到归一化设备坐标
        u = (x - cx) / fx
        v = (y - cy) / fy
        
        # 齐次坐标
        pixel_point = np.array([u, v, 1])
        
        # 在相机坐标系中
        camera_point = np.linalg.inv(K) @ pixel_point
        
        # 计算地面上的点
        # z = 0 (地面)
        z_ground = 0
        # 计算相机坐标系中对应的 x 和 y
        scale = (z_ground - T[2]) / camera_point[2]
        ground_point = scale * camera_point
        
        # 将地面点转换到世界坐标系
        world_point = R @ ground_point + T
        
        # 计算与相机在地面上的垂直投影的距离
        distance = np.sqrt((world_point[0])**2 + (world_point[1])**2)
        
        # 如果 z <= 0（不在地面上），则置为 0
        if world_point[2] > 0:
            distances[y, x] = distance
        else:
            distances[y, x] = 0

# 可视化结果
plt.imshow(distances, cmap='hot')
plt.colorbar(label='Distance to Ground (m)')
plt.title('Distance from Camera Projection to Ground')
plt.xlabel('Pixel X')
plt.ylabel('Pixel Y')
plt.show()
