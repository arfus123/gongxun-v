import numpy as np
import matplotlib.pyplot as plt

def main():
    # 相机参数
    fx = 781.1682
    fy = 780.9376
    cx = 681.8543
    cy = 375.8645
    width = 1280
    height = 720
    
    distanceCam2Ground = 0.2  # 相机到地面的距离

    # 畸变参数
    k1 = 0.1218
    k2 = -0.1440
    p1 = -0.0001145
    p2 = -0.00055488
    k3 = -0.0222

    degree = 120
    # 计算旋转矩阵R（绕X轴120度）
    theta = np.radians(degree)  # 转换为弧度
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    R = np.array([
        [1, 0, 0],
        [0, cos_theta, sin_theta],
        [0, -sin_theta, cos_theta]
    ])

    # 去畸变函数（简化版，实际应用中可能需要更精确的迭代）
    def undistort(xd, yd, k1, k2, p1, p2, k3, eps=1e-6, max_iter=20):
        x = xd
        y = yd
        for _ in range(max_iter):
            r2 = x**2 + y**2
            r4 = r2 * r2
            r6 = r4 * r2
            radial = 1 + k1 * r2 + k2 * r4 + k3 * r6
            x_new = (xd - (2 * p1 * x * y + p2 * (r2 + 2 * x**2))) / radial
            y_new = (yd - (p1 * (r2 + 2 * y**2) + 2 * p2 * x * y)) / radial
            if np.abs(x_new - x) < eps and np.abs(y_new - y) < eps:
                break
            x, y = x_new, y_new
        return x, y

    # 初始化距离矩阵
    distance_map = np.zeros((height, width))

    # 预先计算sin(60)和cos(60)
    sin60 = np.sin(theta)
    cos60 = np.cos(theta)

    # 遍历像素并计算地面交点距离
    for v in range(height):
        for u in range(width):
            # 转换为归一化坐标
            xd = (u - cx) / fx
            yd = (v - cy) / fy

            # 去畸变
            try:
                x_undist, y_undist = undistort(xd, yd, k1, k2, p1, p2, k3)
            except:
                distance_map[v, u] = 0
                continue

            # 计算方向向量的Z分量（世界坐标系）
            # 这里的系数示例中使用了 cos(120) = -0.5, sin(120) = sin60
            dir_world_z = sin60 * y_undist + cos60  # -0.5 对应 cos(120)

            if dir_world_z >= 0:
                # 表示无法与地面(z=0)相交或相交在相机后方
                distance_map[v, u] = 0
                continue

            # 计算比例s，使得Z = 0
            s = -distanceCam2Ground / dir_world_z

            # 计算地面交点 (P_world_x, P_world_y)
            # 其中 -0.5 * y_undist 对应 cos(120)*y_undist, sin60 对应 sin(120)
            P_world_x = s * x_undist
            P_world_y = s * (cos60 * y_undist - sin60)

            # 计算地面距离：相对于相机垂直投影(0,0,0)的距离
            distance_map[v, u] = np.sqrt(P_world_x**2 + P_world_y**2)

    # 使用matplotlib显示
    plt.figure(figsize=(8, 4))
    plt.imshow(distance_map, cmap='jet', interpolation='nearest')
    plt.title("Distance Map (m)")
    plt.colorbar(label='Distance (meters)')
    plt.xlabel("u (pixels)")
    plt.ylabel("v (pixels)")
    plt.show()

    # 保存CSV
    # np.savetxt("distance_map.csv", distance_map, delimiter=",")
    # print("distance_map.csv saved.")

if __name__ == "__main__":
    main()