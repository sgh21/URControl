import numpy as np

def t_curve_interpolation(start_joint, target_joint, max_vel, acc,blend =0.01,control_frequency=125):
    """
    T-curve interpolation in joint space.
    
    Args:
    - start_joint: list of starting joint angles
    - target_joint: list of target joint angles
    - max_vel: maximum velocity
    - acc: maximum acceleration
    - control_frequency: frequency of control in Hz
    
    Returns:
    - trajectory: list of joint angles, velocities, and accelerations at each time step
    """
    joint_diff = np.array(target_joint) - np.array(start_joint)
    total_distance = np.abs(joint_diff)
    
    # Time to accelerate to max velocity
    t_acc = max_vel / acc
    d_acc = 0.5 * acc * t_acc**2
    
    # Check if the distance is large enough to reach maximum velocity
    if total_distance < 2 * d_acc:
        # If not, we need to adjust for a triangular velocity profile
        t_acc = np.sqrt(total_distance / acc)
        t_total = 2 * t_acc
        max_vel_reached = acc * t_acc
    else:
        # Full T-curve
        d_cruise = total_distance - 2 * d_acc
        t_cruise = d_cruise / max_vel
        t_total = 2 * t_acc + t_cruise
        max_vel_reached = max_vel

    num_steps = int(t_total * control_frequency)
    dt = 1 / control_frequency
    trajectory = []
    
    for step in range(num_steps + 1):
        t = step * dt
        if t < t_acc:
            # Accelerating phase
            pos = 0.5 * acc * t**2
            vel = acc * t
            accel = acc
        elif t < (t_total - t_acc):
            # Constant velocity phase
            pos = d_acc + max_vel_reached * (t - t_acc)
            vel = max_vel_reached
            accel = 0
        else:
            # Decelerating phase
            t_dec = t - (t_total - t_acc)
            pos = total_distance - 0.5 * acc * t_dec**2
            vel = acc * (t_total - t)
            accel = -acc
        
        # Interpolate joint position based on progress
        progress = pos / total_distance
        joint_angles = start_joint + progress * joint_diff
        
        trajectory.append((joint_angles.tolist(), vel, accel,blend))
    
    return trajectory


if __name__=="__main__":
    # Example usage
    start_joint = [0, 0, 0]  # Starting joint angles
    target_joint = [1, 0.5, -0.5]  # Target joint angles
    max_vel = 0.5  # Maximum velocity
    acc = 0.2  # Maximum acceleration
    control_frequency = 100  # Control frequency in Hz

    trajectory = t_curve_interpolation(start_joint, target_joint, max_vel, acc, control_frequency)

    # Print trajectory (joint angles, velocity, acceleration)
    for step in trajectory:
        joint_angles, velocity, acceleration = step
        print(f"Joint Angles: {joint_angles}, Velocity: {velocity:.4f}, Acceleration: {acceleration:.4f}")
