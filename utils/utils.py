import numpy as np

def rotate_x(angle):
    """
    绕X轴旋转的旋转矩阵.
    
    参数:
    - angle: 旋转角度，单位为弧度
    
    返回:
    - 旋转矩阵 (3x3)
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])

def rotate_y(angle):
    """
    绕Y轴旋转的旋转矩阵.
    
    参数:
    - angle: 旋转角度，单位为弧度
    
    返回:
    - 旋转矩阵 (3x3)
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[ c,  0,  s],
                     [ 0,  1,  0],
                     [-s,  0,  c]])

def rotate_z(angle):
    """
    绕Z轴旋转的旋转矩阵.
    
    参数:
    - angle: 旋转角度，单位为弧度
    
    返回:
    - 旋转矩阵 (3x3)
    """
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[ c, -s,  0],
                     [ s,  c,  0],
                     [ 0,  0,  1]])

def vector2matrix(xyzrxryrz):
    x, y, z, rx, ry, rz = xyzrxryrz
    theta = np.linalg.norm([rx, ry, rz])
    if theta < 1e-16:
        R = np.eye(3)
    else:
        kx = rx / theta
        ky = ry / theta
        kz = rz / theta
        K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, -1] = [x, y, z]
    return T

def matrix2vector(T):
    x, y, z = T[:3, -1]
    R = T[:3, :3]
    traceR = R[0][0] + R[1][1] + R[2][2]
    if traceR >= 3 - np.finfo(float).eps:
        rxryrz = [0.0, 0.0, 0.0]
    else:
        theta = np.arccos((traceR - 1) / 2)
        kx = (R[2][1] - R[1][2]) / (2 * np.sin(theta))
        ky = (R[0][2] - R[2][0]) / (2 * np.sin(theta))
        kz = (R[1][0] - R[0][1]) / (2 * np.sin(theta))
        rxryrz = [theta * kx, theta * ky, theta * kz]

    return [x, y, z] + rxryrz


# # 示例：绕X轴、Y轴和Z轴旋转30度
# angle = np.radians(30)  # 将角度转化为弧度

# rotation_matrix_x = rotate_x(angle)
# rotation_matrix_y = rotate_y(angle)
# rotation_matrix_z = rotate_z(angle)

# print("绕X轴旋转的旋转矩阵:")
# print(rotation_matrix_x)

# print("\n绕Y轴旋转的旋转矩阵:")
# print(rotation_matrix_y)

# print("\n绕Z轴旋转的旋转矩阵:")
# print(rotation_matrix_z)