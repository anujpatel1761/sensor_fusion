#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Complete 3D Extended Kalman Filter Implementation for KITTI Dataset
----------------------------------------------------------------

This script implements a 3D Extended Kalman Filter for sensor fusion of GPS and IMU data
from the KITTI dataset. It includes comprehensive data parsing, full 3D state estimation,
and extensive visualization capabilities.

Main Features:
- Full 3D state estimation (position, orientation, velocity)
- Actual IMU measurements (accelerometer and gyroscope)
- GPS position updates with accuracy-based measurement noise
- Adaptive process noise based on motion dynamics
- Comprehensive visualization and analysis tools
- Proper timestamp handling from dataset

Author: [Your Name]
Date: January 2025
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec

class OXTSData:
    """
    Parser for KITTI OXTS data format.
    
    This class handles parsing of the OXTS data files from the KITTI dataset,
    providing structured access to all sensor measurements including GPS, IMU,
    and accuracy metrics.
    
    Attributes:
        lat (float): Latitude in degrees
        lon (float): Longitude in degrees
        alt (float): Altitude in meters
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians
        yaw (float): Yaw angle in radians
        vn (float): Velocity towards north in m/s
        ve (float): Velocity towards east in m/s
        vf (float): Forward velocity in m/s
        vl (float): Leftward velocity in m/s
        vu (float): Upward velocity in m/s
        ax (float): X-axis acceleration in m/s²
        ay (float): Y-axis acceleration in m/s²
        az (float): Z-axis acceleration in m/s²
        wx (float): Angular rate around X-axis in rad/s
        wy (float): Angular rate around Y-axis in rad/s
        wz (float): Angular rate around Z-axis in rad/s
        pos_accuracy (float): Position accuracy in meters
        vel_accuracy (float): Velocity accuracy in m/s
    """
    
    def __init__(self, line):
        """
        Initialize OXTSData object from a line of OXTS data.
        
        Args:
            line (str): Space-separated line of OXTS measurements
        """
        vals = [float(x) for x in line.strip().split()]
        
        # Position data (lat, lon, alt)
        self.lat = vals[0]
        self.lon = vals[1]
        self.alt = vals[2]
        
        # Orientation (Euler angles)
        self.roll = vals[3]   # Roll  (rad) - positive = left side up
        self.pitch = vals[4]  # Pitch (rad) - positive = front down
        self.yaw = vals[5]    # Yaw   (rad) - positive = counter clockwise
        
        # Velocities (in different frames)
        self.vn = vals[6]     # Velocity towards north
        self.ve = vals[7]     # Velocity towards east
        self.vf = vals[8]     # Forward velocity
        self.vl = vals[9]     # Leftward velocity
        self.vu = vals[10]    # Upward velocity
        
        # Accelerations
        self.ax = vals[11]    # Acceleration in x
        self.ay = vals[12]    # Acceleration in y
        self.az = vals[13]    # Acceleration in z
        
        # Angular rates (gyroscope)
        self.wx = vals[17]    # Angular rate around x
        self.wy = vals[18]    # Angular rate around y
        self.wz = vals[19]    # Angular rate around z
        
        # Accuracy metrics
        self.pos_accuracy = vals[23]  # Position accuracy
        self.vel_accuracy = vals[24]  # Velocity accuracy

def latlon_to_xyz(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """
    Convert geodetic coordinates to local ENU (East-North-Up) coordinates.
    
    Args:
        lat, lon (float): Position in degrees
        alt (float): Altitude in meters
        lat_ref, lon_ref (float): Reference position in degrees
        alt_ref (float): Reference altitude in meters
        
    Returns:
        tuple: (x, y, z) coordinates in meters relative to reference point
    """
    # Earth's radius in meters
    R = 6378137.0
    
    # Convert to radians
    lat, lon = np.radians(lat), np.radians(lon)
    lat_ref, lon_ref = np.radians(lat_ref), np.radians(lon_ref)
    
    # Compute differences
    d_lat = lat - lat_ref
    d_lon = lon - lon_ref
    d_alt = alt - alt_ref
    
    # Local coordinates (ENU)
    x = d_lon * R * np.cos(lat_ref)  # East
    y = d_lat * R                    # North
    z = d_alt                        # Up
    
    return x, y, z

# class ExtendedKalmanFilter3D:
#     """
#     3D Extended Kalman Filter for GPS/IMU fusion.
    
#     State vector: [x, y, z, roll, pitch, yaw, vx, vy, vz]
    
#     Attributes:
#         x (np.array): State vector
#         P (np.array): State covariance matrix
#         Q (np.array): Process noise covariance
#         R (np.array): Measurement noise covariance
#     """
    
#     def __init__(self, init_state, init_cov):
#         """
#         Initialize the 3D EKF.
        
#         Args:
#             init_state (np.array): Initial state vector (9,)
#             init_cov (np.array): Initial state covariance (9,9)
#         """
#         self.x = init_state
#         self.P = init_cov
        
#         # Initial process and measurement noise matrices
#         self.Q = np.eye(9) * 0.1  # Will be updated adaptively
#         self.R = np.eye(3) * 1.0  # Will be updated based on GPS accuracy
    
#     def get_rotation_matrix(self, roll, pitch, yaw):
#         """
#         Compute 3D rotation matrix from Euler angles.
        
#         Args:
#             roll, pitch, yaw (float): Euler angles in radians
            
#         Returns:
#             np.array: 3x3 rotation matrix
#         """
#         # Compute trig functions
#         cr, cp, cy = np.cos([roll, pitch, yaw])
#         sr, sp, sy = np.sin([roll, pitch, yaw])
        
#         # Individual rotation matrices
#         R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
#         R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
#         R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
#         # Combined rotation matrix
#         R = R_z @ R_y @ R_x
#         return R
    
#     def predict(self, imu_data, dt):
#         """
#         EKF prediction step with gravity compensation
#         """
#         """
#         EKF prediction step using IMU measurements.
        
#         Args:
#             imu_data (OXTSData): IMU measurements
#             dt (float): Time step in seconds
#         """
#         # Extract current state components
#         # Extract current state
#         roll, pitch, yaw = self.x[3:6]
#         vx, vy, vz = self.x[6:9]
        
#         # Get rotation matrix
#         R = self.get_rotation_matrix(roll, pitch, yaw)
        
#         # Remove gravity from accelerometer readings
#         # Note: In KITTI, az is positive upward and includes gravity
#         acc_body = np.array([imu_data.ax, imu_data.ay, imu_data.az - 9.81])
        
#         # Transform acceleration to world frame
#         acc_world = R @ acc_body
        
#         # IMU measurements
#         gyro = np.array([imu_data.wx, imu_data.wy, imu_data.wz])
        
#         # State prediction
#         pos_next = self.x[0:3] + R @ np.array([vx, vy, vz]) * dt
#         att_next = self.x[3:6] + gyro * dt
#         vel_next = self.x[6:9] + acc_world * dt
        
#         x_pred = np.concatenate([pos_next, att_next, vel_next])
        
#         # Jacobian computation remains the same
#         F = np.eye(9)
#         F[0:3, 6:9] = R * dt
        
#         # Covariance prediction
#         self.P = F @ self.P @ F.T + self.Q
#         self.x = x_pred
    
#     def update_gps(self, gps_data):
#         """
#         EKF update step using GPS measurements.
        
#         Args:
#             gps_data (OXTSData): GPS measurements including position and accuracy
#         """
#         # Measurement matrix for position updates
#         H = np.zeros((3, 9))
#         H[0:3, 0:3] = np.eye(3)
        
#         # GPS measurement
#         z = np.array([gps_data.x, gps_data.y, gps_data.z])
        
#         # Update measurement noise based on GPS accuracy
#         self.R = np.eye(3) * (gps_data.pos_accuracy ** 2)
        
#         # Innovation computation
#         z_pred = H @ self.x
#         y = z - z_pred
        
#         # Kalman gain computation
#         S = H @ self.P @ H.T + self.R
#         K = self.P @ H.T @ np.linalg.inv(S)
        
#         # State and covariance update
#         self.x = self.x + K @ y
#         self.P = (np.eye(9) - K @ H) @ self.P



class ExtendedKalmanFilter3D:
    def __init__(self, init_state, init_cov):
        self.x = init_state
        self.P = init_cov
        
        # Tuned process noise values
        pos_std = 0.5    # position uncertainty
        ori_std = 0.1    # orientation uncertainty
        vel_std = 1.0    # velocity uncertainty
        
        self.Q = np.diag([pos_std]*3 + [ori_std]*3 + [vel_std]*3)
        self.R = np.eye(3) * 1.0
    
    def get_rotation_matrix(self, roll, pitch, yaw):
        cr, cp, cy = np.cos([roll, pitch, yaw])
        sr, sp, sy = np.sin([roll, pitch, yaw])
        
        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        return R_z @ R_y @ R_x
    
    def transform_velocity_body_to_world(self, v_body, roll, pitch, yaw):
        """Transform velocity from body to world frame."""
        R = self.get_rotation_matrix(roll, pitch, yaw)
        return R @ v_body
    
    def transform_velocity_world_to_body(self, v_world, roll, pitch, yaw):
        """Transform velocity from world to body frame."""
        R = self.get_rotation_matrix(roll, pitch, yaw)
        return R.T @ v_world
    
    def predict(self, imu_data, dt):
        # Current state
        roll, pitch, yaw = self.x[3:6]
        v_world = self.x[6:9]
        
        # Get rotation matrix
        R = self.get_rotation_matrix(roll, pitch, yaw)
        
        # Compensate for gravity in accelerometer readings
        # KITTI IMU data includes gravity, so we subtract it
        acc_body = np.array([imu_data.ax, imu_data.ay, imu_data.az - 9.81])
        
        # Transform body acceleration to world frame
        acc_world = R @ acc_body
        
        # Angular rates (gyro)
        gyro = np.array([imu_data.wx, imu_data.wy, imu_data.wz])
        
        # State prediction
        pos_next = self.x[0:3] + v_world * dt + 0.5 * acc_world * dt**2
        att_next = self.x[3:6] + gyro * dt
        vel_next = v_world + acc_world * dt
        
        # Combine predictions
        x_pred = np.concatenate([pos_next, att_next, vel_next])
        
        # Jacobian
        F = np.eye(9)
        F[0:3, 6:9] = np.eye(3) * dt  # Position affected by velocity
        
        # Update state and covariance
        self.P = F @ self.P @ F.T + self.Q
        self.x = x_pred
    
    def update_gps(self, gps_data):
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)
        
        z = np.array([gps_data.x, gps_data.y, gps_data.z])
        
        # Use actual GPS accuracy for measurement noise
        acc = max(gps_data.pos_accuracy, 0.5)  # minimum 0.5m accuracy
        self.R = np.eye(3) * (acc ** 2)
        
        # Innovation
        z_pred = H @ self.x
        y = z - z_pred
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(9) - K @ H) @ self.P




def plot_comprehensive_results(est_states, oxts_data, timestamps, rmse_values):
    """
    Create comprehensive visualization of EKF results.
    
    Args:
        est_states (np.array): Estimated states from EKF
        oxts_data (list): List of OXTS measurements
        timestamps (list): List of measurement timestamps
        rmse_values (dict): Dictionary of RMSE values for different quantities
    """
    # Convert timestamps to seconds from start
    time = [(t - timestamps[0]).total_seconds() for t in timestamps]
    
    # Create figure with GridSpec
    fig = plt.figure(figsize=(20, 15))
    gs = GridSpec(3, 3, figure=fig)
    
    # 1. 3D Trajectory Plot
    ax1 = fig.add_subplot(gs[0, 0], projection='3d')
    ax1.plot(est_states[:,0], est_states[:,1], est_states[:,2], 'b-', 
            label='EKF Estimate')
    # Add GPS measurements
    gps_x = [data.x for data in oxts_data]
    gps_y = [data.y for data in oxts_data]
    gps_z = [data.z for data in oxts_data]
    ax1.plot(gps_x, gps_y, gps_z, 'r.', label='GPS Measurements')
    ax1.set_title('3D Trajectory')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.legend()
    
    # 2. Orientation Plot
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(time, est_states[:,3], 'r-', label='Roll')
    ax2.plot(time, est_states[:,4], 'g-', label='Pitch')
    ax2.plot(time, est_states[:,5], 'b-', label='Yaw')
    ax2.set_title('Orientation vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (rad)')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Velocity Plot
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(time, est_states[:,6], 'r-', label='Vx')
    ax3.plot(time, est_states[:,7], 'g-', label='Vy')
    ax3.plot(time, est_states[:,8], 'b-', label='Vz')
    ax3.set_title('Velocity Components vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.legend()
    ax3.grid(True)
    
    # 4. Position Error Plot
    ax4 = fig.add_subplot(gs[1, 0])
    position_error = np.sqrt(np.sum((est_states[:,0:3] - 
                                   np.array([[d.x, d.y, d.z] for d in oxts_data]))**2, 
                                   axis=1))
    ax4.plot(time, position_error, 'r-', label='Position Error')
    ax4.set_title(f'Position Error vs Time (RMSE: {rmse_values["position"]:.3f} m)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Error (m)')
    ax4.grid(True)
    
    # 5. Orientation Error Plot
    ax5 = fig.add_subplot(gs[1, 1])
    orientation_error = np.sqrt(np.sum((est_states[:,3:6] - 
                                      np.array([[d.roll, d.pitch, d.yaw] 
                                               for d in oxts_data]))**2, axis=1))
    ax5.plot(time, orientation_error, 'g-', label='Orientation Error')
    ax5.set_title(f'Orientation Error vs Time (RMSE: {rmse_values["orientation"]:.3f} rad)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Error (rad)')
    ax5.grid(True)
    
    # 6. Velocity Error Plot
    ax6 = fig.add_subplot(gs[1, 2])
    velocity_error = np.sqrt(np.sum((est_states[:,6:9] - 
                                   np.array([[d.vf, d.vl, d.vu] 
                                            for d in oxts_data]))**2, axis=1))
    ax6.plot(time, velocity_error, 'b-', label='Velocity Error')
    ax6.set_title(f'Velocity Error vs Time (RMSE: {rmse_values["velocity"]:.3f} m/s)')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Error (m/s)')
    ax6.grid(True)
    
    # 7. GPS Accuracy Plot
    ax7 = fig.add_subplot(gs[2, 0])
    gps_accuracy = [d.pos_accuracy for d in oxts_data]
    ax7.plot(time, gps_accuracy, 'k-', label='GPS Accuracy')
    ax7.set_title('GPS Position Accuracy vs Time')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Accuracy (m)')
    ax7.grid(True)
    
    # 8. IMU Measurements Plot
    ax8 = fig.add_subplot(gs[2, 1])
    acc_magnitude = [np.sqrt(d.ax**2 + d.ay**2 + d.az**2) for d in oxts_data]
    gyro_magnitude = [np.sqrt(d.wx**2 + d.wy**2 + d.wz**2) for d in oxts_data]
    ax8.plot(time, acc_magnitude, 'r-', label='Acc Magnitude')
    ax8.plot(time, gyro_magnitude, 'b-', label='Gyro Magnitude')
    ax8.set_title('IMU Measurements vs Time')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Magnitude')
    ax8.legend()
    ax8.grid(True)
    
    # 9. Filter Innovation Plot
    ax9 = fig.add_subplot(gs[2, 2])
    innovation = np.sqrt(np.sum((est_states[1:,0:3] - est_states[:-1,0:3])**2, axis=1))
    ax9.plot(time[1:], innovation, 'g-', label='Position Innovation')
    ax9.set_title('Filter Innovation vs Time')
    ax9.set_xlabel('Time (s)')
    ax9.set_ylabel('Innovation Magnitude (m)')
    ax9.grid(True)
    
    plt.tight_layout()
    return fig

def compute_rmse_metrics(est_states, oxts_data, ekf):
    """
    Compute RMSE with proper frame transformations.
    """
    position_errors = []
    orientation_errors = []
    velocity_errors = []
    
    for est, oxts in zip(est_states, oxts_data):
        # Position error (in world frame)
        pos_error = np.sqrt(np.sum((est[0:3] - np.array([oxts.x, oxts.y, oxts.z]))**2))
        position_errors.append(pos_error)
        
        # Orientation error
        ori_error = np.sqrt(np.sum((est[3:6] - np.array([oxts.roll, oxts.pitch, oxts.yaw]))**2))
        orientation_errors.append(ori_error)
        
        # Transform EKF velocity from world to body frame for comparison
        v_est_world = est[6:9]
        v_est_body = ekf.transform_velocity_world_to_body(v_est_world, oxts.roll, oxts.pitch, oxts.yaw)
        
        # OXTS velocities in body frame
        v_oxts_body = np.array([oxts.vf, -oxts.vl, oxts.vu])  # Note the sign change for leftward velocity
        
        # Velocity error (in body frame)
        vel_error = np.sqrt(np.sum((v_est_body - v_oxts_body)**2))
        velocity_errors.append(vel_error)
    
    return {
        "position": np.sqrt(np.mean(np.array(position_errors)**2)),
        "orientation": np.sqrt(np.mean(np.array(orientation_errors)**2)),
        "velocity": np.sqrt(np.mean(np.array(velocity_errors)**2))
    }

def get_rotation_matrix(roll, pitch, yaw):
    """Helper function for rotation matrix computation."""
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    
    R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    
    return R_z @ R_y @ R_x





def main():
    """
    Main function to run the 3D EKF algorithm.
    """
    print("Starting 3D EKF processing...")
    
    # 1. Configuration
    DATA_PATH = r"C:\Users\anujp\Desktop\sensor_fusion\data\2011_09_26_drive_0001_sync\2011_09_26\2011_09_26_drive_0001_sync\oxts\data"

    
    # 2. Load timestamps
    print("Loading timestamps...")
    timestamps = []
    with open(os.path.join(os.path.dirname(DATA_PATH), "timestamps.txt"), 'r') as f:
        for line in f:
            try:
                # First try with nanoseconds
                timestamp = datetime.strptime(line.strip(), '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                try:
                    # If that fails, try without nanoseconds
                    timestamp = datetime.strptime(line.strip().split('.')[0], '%Y-%m-%d %H:%M:%S')
                    # Add back the fractional seconds
                    microseconds = int(line.strip().split('.')[1][:6])  # Take only first 6 digits
                    timestamp = timestamp.replace(microsecond=microseconds)
                except Exception as e:
                    print(f"Warning: Could not parse timestamp: {line.strip()}")
                    continue
            timestamps.append(timestamp)
    
    if not timestamps:
        raise ValueError("No timestamps could be parsed from the file!")
    
    print(f"Successfully loaded {len(timestamps)} timestamps")
  
    # 3. Calculate time steps
    dt = [(timestamps[i+1] - timestamps[i]).total_seconds() 
          for i in range(len(timestamps)-1)]
    dt.append(dt[-1])  # Repeat last dt for final frame
    
    # 4. Load OXTS data
    print("Loading OXTS data...")
    oxts_data = []
    files = sorted([f for f in os.listdir(DATA_PATH) 
                   if f.endswith(".txt") and "Zone.Identifier" not in f])
    
    for filename in files:
        with open(os.path.join(DATA_PATH, filename), 'r') as f:
            line = f.readline()
            oxts_data.append(OXTSData(line))
    
    # 5. Convert GPS coordinates to local frame
    print("Converting GPS coordinates to local frame...")
    ref_lat = oxts_data[0].lat
    ref_lon = oxts_data[0].lon
    ref_alt = oxts_data[0].alt
    
    for data in oxts_data:
        data.x, data.y, data.z = latlon_to_xyz(
            data.lat, data.lon, data.alt,
            ref_lat, ref_lon, ref_alt
        )
    
    # 6. Initialize EKF
    print("Initializing EKF...")
    init_state = np.zeros(9)
    init_state[3:6] = [oxts_data[0].roll, oxts_data[0].pitch, oxts_data[0].yaw]
    init_cov = np.diag([1.0]*3 + [0.1]*3 + [0.1]*3)
    
    ekf = ExtendedKalmanFilter3D(init_state, init_cov)
    
    # 7. Run EKF
    print("Running EKF...")
    est_states = [init_state]
    
    for i in range(1, len(oxts_data)):
        if i % 100 == 0:
            print(f"Processing frame {i}/{len(oxts_data)}")
            
        # Predict using IMU
        ekf.predict(oxts_data[i], dt[i])
        
        # Update using GPS
        ekf.update_gps(oxts_data[i])
        
        est_states.append(ekf.x.copy())
    
    est_states = np.array(est_states)
    
    # 8. Compute performance metrics
    print("Computing performance metrics...")
    rmse_values = compute_rmse_metrics(est_states, oxts_data, ekf)  # Pass ekf instance

    
    print("\nRMSE Values:")
    print(f"Position RMSE: {rmse_values['position']:.3f} m")
    print(f"Orientation RMSE: {rmse_values['orientation']:.3f} rad")
    print(f"Velocity RMSE: {rmse_values['velocity']:.3f} m/s")
    print("\nValidation Data:")
    print(f"First frame velocities:")
    print(f"OXTS Body: [{oxts_data[0].vf:.2f}, {-oxts_data[0].vl:.2f}, {oxts_data[0].vu:.2f}]")
    v_world = ekf.transform_velocity_body_to_world(
        np.array([oxts_data[0].vf, -oxts_data[0].vl, oxts_data[0].vu]),
        oxts_data[0].roll, oxts_data[0].pitch, oxts_data[0].yaw
    )
    print(f"Transformed to World: [{v_world[0]:.2f}, {v_world[1]:.2f}, {v_world[2]:.2f}]")
        
    # 9. Plot results
    print("Generating plots...")
    fig = plot_comprehensive_results(est_states, oxts_data, timestamps, rmse_values)
    
    # Save the results
    plt.savefig('3d_ekf_results.png', dpi=300, bbox_inches='tight')
    print("Results saved as '3d_ekf_results.png'")
    
    # Show the plots
    plt.show()

if __name__ == "__main__":
    main()