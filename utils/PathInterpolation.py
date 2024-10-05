import numpy as np

class Quaternion:
    def __init__(self, q):
        self.q = q  # q = [qx, qy, qz, qw]

    def __mul__(self, other):
        w1, x1, y1, z1 = self.q[3], self.q[0], self.q[1], self.q[2]
        w2, x2, y2, z2 = other.q[3], other.q[0], other.q[1], other.q[2]
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return Quaternion([x, y, z, w])

    def normalize(self):
        norm = np.linalg.norm(self.q)
        self.q = self.q / norm
        return self

    @staticmethod
    def dot(q1, q2):
        return np.dot(q1.q, q2.q)

    @staticmethod
    def slerp(q1, q2, t):
        dot = Quaternion.dot(q1, q2)
        if (dot < 0.0):
            q1 = Quaternion([-v for v in q1.q])
            dot = -dot

        if dot > 0.9995:
            result = (1 - t) * np.array(q1.q) + t * np.array(q2.q)
            return Quaternion(result).normalize()

        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s1 = np.sin(theta_0 - theta) / sin_theta_0
        s2 = sin_theta / sin_theta_0

        result = s1 * np.array(q1.q) + s2 * np.array(q2.q)
        return Quaternion(result)

    @staticmethod
    def from_rotation_vector(rvec):
        """Convert a rotation vector (Rodrigues' rotation formula) to quaternion."""
        theta = np.linalg.norm(rvec)
        if theta < 1e-6:
            return Quaternion([0, 0, 0, 1])
        axis = rvec / theta
        half_theta = theta / 2
        qw = np.cos(half_theta)
        qx, qy, qz = axis * np.sin(half_theta)
        return Quaternion([qx, qy, qz, qw])

    def to_rotation_vector(self):
        """Convert quaternion back to rotation vector."""
        theta = 2 * np.arccos(self.q[3])
        sin_half_theta = np.sqrt(1 - self.q[3]**2)
        if sin_half_theta < 1e-6:
            return np.array([0, 0, 0])
        axis = np.array(self.q[:3]) / sin_half_theta
        return axis * theta

def smooth_velocity(distance, max_velocity, acceleration):
    t_accel = max_velocity / acceleration
    d_accel = 0.5 * acceleration * t_accel**2
    if distance < 2 * d_accel:
        t_accel = np.sqrt(distance / acceleration)
        t_total = 2 * t_accel
    else:
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / max_velocity
        t_total = 2 * t_accel + t_cruise
    return t_total,t_accel

def linear_interpolate(start, end, t):
    return (1 - t) * start + t * end

def plan_6dof_trajectory(start_pose, target_pose, max_velocity, max_acceleration, control_frequency):
    x_start, y_start, z_start, rx_start, ry_start, rz_start = start_pose
    x_end, y_end, z_end, rx_end, ry_end, rz_end = target_pose

    # Convert rotation vectors to quaternions (this is the modification)
    start_quat = Quaternion.from_rotation_vector([rx_start, ry_start, rz_start])
    end_quat = Quaternion.from_rotation_vector([rx_end, ry_end, rz_end])

    distance = np.linalg.norm([x_end - x_start, y_end - y_start, z_end - z_start])
    angle_distance = Quaternion.dot(start_quat, end_quat)

    time_total_translation , = smooth_velocity(distance, max_velocity, max_acceleration)
    time_total_rotation = smooth_velocity(angle_distance, max_velocity, max_acceleration)

    time_total = max(time_total_translation, time_total_rotation)
    num_steps = int(time_total * control_frequency)
    trajectory = []

    for step in range(num_steps):
        t = step / num_steps
        x = linear_interpolate(x_start, x_end, t)
        y = linear_interpolate(y_start, y_end, t)
        z = linear_interpolate(z_start, z_end, t)

        # Quaternion interpolation (SLERP)
        interp_quat = Quaternion.slerp(start_quat, end_quat, t)
        rvec = interp_quat.to_rotation_vector()  # Convert back to rotation vector

        pose = [x, y, z, float(rvec[0]), float(rvec[1]), float(rvec[2])]
        trajectory.append(pose)

    return trajectory


if __name__=="__main__":    
    # Example Usage
    start_pose = [0, 0, 0, 0, 0, 0]  # [x, y, z, Rx, Ry, Rz (rotation vector)]
    target_pose = [1, 1, 1, np.pi/2, np.pi/2, np.pi/2]  # [x, y, z, Rx, Ry, Rz (rotation vector)]
    max_velocity = 0.5
    max_acceleration = 0.2
    control_frequency = 100
    import time
    t1 = time.time()
    trajectory = plan_6dof_trajectory(start_pose, target_pose, max_velocity, max_acceleration, control_frequency)
    t2 = time.time()
    print("Time elapsed: ", t2 - t1)
    print(len(trajectory))  # List of poses [x, y, z, Rx, Ry, Rz] along the trajectory