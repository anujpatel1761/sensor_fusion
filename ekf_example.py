import os
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------
# 1) UTILITY FUNCTIONS
# -----------------------------------------------------------------------
def latlon_to_xy(lat, lon, lat0, lon0):
    """
    Very simplified conversion of lat/lon to local x/y (meters),
    treating lat0/lon0 as the origin (0,0).

    For real usage:
      - Use 'pyproj' or a proper geodesy library
      - Or an ENU conversion from lat0/lon0

    Args:
        lat (float): Current latitude in degrees.
        lon (float): Current longitude in degrees.
        lat0 (float): Reference (origin) latitude in degrees.
        lon0 (float): Reference (origin) longitude in degrees.

    Returns:
        x, y (floats): Approximate local coordinates in meters.
    """
    R = 6378137.0  # Earth radius (approx)
    d_lat = np.radians(lat - lat0)
    d_lon = np.radians(lon - lon0)
    x = d_lon * R * np.cos(np.radians(lat0))
    y = d_lat * R
    return x, y


def compute_rmse(x_est, x_true):
    """
    Compute the Root Mean Square Error between two sets of 2D points.

    Args:
        x_est (np.array): shape (N, 2) of estimated positions.
        x_true (np.array): shape (N, 2) of ground-truth or reference positions.

    Returns:
        float: RMSE value
    """
    diffs = x_est - x_true
    sq_err = np.sum(diffs**2, axis=1)
    rmse = np.sqrt(np.mean(sq_err))
    return rmse


# -----------------------------------------------------------------------
# 2) EXTENDED KALMAN FILTER CLASS
# -----------------------------------------------------------------------
class ExtendedKalmanFilter:
    """
    A simple 2D EKF that tracks:
       state = [x, y, yaw, v]

    and fuses:
       - IMU data (yaw_rate, forward_accel) as control
       - GPS measurements (x_gps, y_gps) as measurement

    For a more advanced project, you might:
      - parse actual gyro rates from OXTS
      - add z/altitude, roll/pitch, velocities, biases, etc.
    """

    def __init__(self, init_state, init_cov, process_cov, measurement_cov):
        """
        Initialize the EKF with given parameters.

        Args:
            init_state (np.array): shape (4,) e.g. [x0, y0, yaw0, v0]
            init_cov (np.array): shape (4,4) initial state covariance
            process_cov (np.array): shape (4,4) Q matrix
            measurement_cov (np.array): shape (2,2) R matrix for GPS
        """
        self.x = init_state      # State vector: [x, y, yaw, v]
        self.P = init_cov        # Covariance of the state
        self.Q = process_cov     # Process noise covariance
        self.R = measurement_cov # Measurement noise covariance (GPS)

    def predict(self, u, dt):
        """
        EKF prediction step using a simple 2D bicycle-like kinematic model.

        Args:
            u (np.array): shape (2,) = [yaw_rate, accel]
            dt (float): time step in seconds
        """
        yaw = self.x[2]
        v   = self.x[3]
        yaw_rate = u[0]
        accel    = u[1]

        # Nonlinear prediction
        # x_{k+1} = x_k + v*cos(yaw)*dt
        # y_{k+1} = y_k + v*sin(yaw)*dt
        # yaw_{k+1} = yaw_k + yaw_rate * dt
        # v_{k+1}   = v_k + accel * dt
        fx = np.array([
            self.x[0] + v * np.cos(yaw) * dt,
            self.x[1] + v * np.sin(yaw) * dt,
            yaw + yaw_rate * dt,
            v + accel * dt
        ])

        # Jacobian F of f wrt x
        F = np.eye(4)
        F[0, 2] = -v * np.sin(yaw) * dt  # d x_{k+1} / d yaw
        F[0, 3] =  np.cos(yaw) * dt      # d x_{k+1} / d v
        F[1, 2] =  v * np.cos(yaw) * dt  # d y_{k+1} / d yaw
        F[1, 3] =  np.sin(yaw) * dt      # d y_{k+1} / d v

        # Covariance prediction
        P_pred = F @ self.P @ F.T + self.Q

        # Update filter state
        self.x = fx
        self.P = P_pred

    def update_gps(self, z):
        """
        EKF update step for GPS measurement: z = [x_gps, y_gps].

        Args:
            z (np.array): shape (2,) giving measured position in 2D
        """
        # Measurement model: z = H * x
        # H: 2x4, mapping [x, y, yaw, v] -> [x, y]
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Predict measurement
        z_pred = H @ self.x  # shape (2,)

        # Innovation
        y_k = z - z_pred

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)  # shape (4,2)

        # Update state & covariance
        self.x = self.x + K @ y_k
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P


# -----------------------------------------------------------------------
# 3) MAIN SCRIPT (EXTENDED) TO READ OXTS DATA & RUN EKF
# -----------------------------------------------------------------------
def main():
    """
    Demonstration of a 2D EKF for KITTI-like GPS+IMU data.

    This script:
      1. Reads OXTS data from .txt files (lat, lon, etc.)
      2. Converts lat/lon to a local (x, y) frame
      3. Approximates yaw_rate from heading differences (naive)
      4. Builds a 2D EKF to fuse "IMU" (yaw_rate + forward_accel) + "GPS" (x, y)
      5. Plots:
         - XY trajectory (EKF vs. GPS)
         - Heading vs. time
         - Velocity vs. time
      6. Computes a position RMSE between EKF and raw GPS
    """

    # ---------------------
    # 3.1 - User Parameters
    # ---------------------
    # Update this path to match your KITTI raw data location
    DATA_PATH = r"\\wsl.localhost\Ubuntu\home\anujpatel1761\sensor_fusion\data\2011_09_26_drive_0001_sync\2011_09_26\2011_09_26_drive_0001_sync\oxts\data"
    # Approx OXTS frequency is 10 Hz; adjust if using timestamps
    dt = 0.1

    # Initialize the filter covariances (tweak as needed)
    init_cov   = np.eye(4) * 1.0
    process_cov = np.diag([0.1, 0.1, 0.1, 1.0])  # Q
    measurement_cov = np.diag([1.0, 1.0])        # R

    # ---------------------
    # 3.2 - Load OXTS Files
    # ---------------------
    files = sorted([f for f in os.listdir(DATA_PATH) 
                    if f.endswith(".txt") and "Zone.Identifier" not in f])

    lats = []
    lons = []
    forward_accels = []
    # We'll derive yaw_rate from heading differences below
    # Or read from columns if available
    # Timestamps omitted for brevity, but recommended for real usage

    for filename in files:
        with open(os.path.join(DATA_PATH, filename), 'r') as f:
            line = f.readline().strip()
            vals = [float(c) for c in line.split()]

            # Example columns: 
            # lat=vals[0], lon=vals[1], alt=vals[2], roll=vals[3], pitch=vals[4], yaw=vals[5], ...
            # forward_accel ~ vals[11] if it is 'ax'
            lat = vals[0]
            lon = vals[1]
            fwd_acc = vals[11]

            lats.append(lat)
            lons.append(lon)
            forward_accels.append(fwd_acc)

    # -------------------------------
    # 3.3 - Convert lat/lon to local XY
    # -------------------------------
    lat0, lon0 = lats[0], lons[0]
    xy_coords = []
    for lat, lon in zip(lats, lons):
        x, y = latlon_to_xy(lat, lon, lat0, lon0)
        xy_coords.append([x, y])
    xy_coords = np.array(xy_coords)

    # --------------------------------------
    # 3.4 - Approximate Yaw & Yaw_Rate (Naive)
    # --------------------------------------
    # heading = atan2( deltaY, deltaX ) for consecutive frames
    headings = []
    for i in range(len(xy_coords) - 1):
        dx = xy_coords[i+1, 0] - xy_coords[i, 0]
        dy = xy_coords[i+1, 1] - xy_coords[i, 1]
        head = np.arctan2(dy, dx)
        headings.append(head)
    # Last frame's heading: replicate
    headings.append(headings[-1])
    
    # yaw_rate is derivative of heading
    yaw_rates = []
    for i in range(len(headings) - 1):
        d_yaw = headings[i+1] - headings[i]
        # Wrap to [-pi, pi]
        d_yaw = (d_yaw + np.pi) % (2*np.pi) - np.pi
        yaw_rates.append(d_yaw / dt)
    yaw_rates.append(yaw_rates[-1])  # replicate final

    # --------------------------------------
    # 3.5 - Initialize the EKF
    # --------------------------------------
    # State = [x, y, yaw, v]
    # We'll start with x=0, y=0, yaw=headings[0], v=0
    init_state = np.array([0.0, 0.0, headings[0], 0.0])
    ekf = ExtendedKalmanFilter(
        init_state, init_cov, process_cov, measurement_cov
    )

    # --------------------------------------
    # 3.6 - MAIN EKF LOOP
    # --------------------------------------
    est_states = []
    for i in range(len(xy_coords)):
        if i == 0:
            est_states.append(ekf.x.copy()) 
            continue

        # Control input: [yaw_rate, forward_accel]
        u = np.array([yaw_rates[i], forward_accels[i]])

        # PREDICT
        ekf.predict(u, dt)

        # GPS measurement: shift current GPS by first point => "relative to origin"
        z = xy_coords[i] - xy_coords[0]

        # UPDATE
        ekf.update_gps(z)

        est_states.append(ekf.x.copy())

    est_states = np.array(est_states)  # shape (N, 4)

    # --------------------------------------
    # 3.7 - Evaluate & Plot
    # --------------------------------------
    # (1) Final State
    print("\nFinal estimated state [x, y, yaw, v]:", est_states[-1])

    # (2) Compute position RMSE
    # EKF position is est_states[:,0:2], raw "GPS(shifted)" is xy_coords - xy_coords[0]
    gps_shifted = xy_coords - xy_coords[0]
    pos_rmse = compute_rmse(est_states[:,0:2], gps_shifted)
    print(f"Position RMSE (m): {pos_rmse:.3f}")

    # PLOTTING
    time_vec = np.arange(len(est_states)) * dt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("2D EKF for KITTI Raw OXTS (Extended Example)")

    # -- (A) XY Trajectory
    ax = axes[0,0]
    ax.plot(gps_shifted[:,0], gps_shifted[:,1], 'r-', label="GPS (shifted)")
    ax.plot(est_states[:,0], est_states[:,1], 'b--', label="EKF")
    ax.set_title("XY Trajectory")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.legend()

    # -- (B) Yaw / Heading over Time
    ax = axes[0,1]
    ax.plot(time_vec, headings, 'g-', label="Naive heading from GPS")
    ax.plot(time_vec, est_states[:,2], 'b--', label="EKF yaw")
    ax.set_title("Heading vs. Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heading (rad)")
    ax.legend()

    # -- (C) Velocity over Time
    ax = axes[1,0]
    ax.plot(time_vec, est_states[:,3], 'b-', label="EKF velocity")
    ax.set_title("Estimated Velocity vs. Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.legend()

    # -- (D) Position Error over Time
    # Distance between EKF [x, y] and GPS(shifted) at each frame
    ekf_xy = est_states[:,0:2]
    gps_xy = gps_shifted
    dist_errs = np.sqrt(np.sum((ekf_xy - gps_xy)**2, axis=1))
    ax = axes[1,1]
    ax.plot(time_vec, dist_errs, 'r-', label="Position Error")
    ax.set_title("EKF vs. GPS Position Error")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.legend()

    plt.tight_layout()
    plt.show()

# End of main()


if __name__ == "__main__":
    main()
