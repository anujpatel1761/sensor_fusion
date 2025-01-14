# 3D GPS/IMU Sensor Fusion using Extended Kalman Filter

## Quick Results 🎯
- **Position Accuracy**: 0.118 meters RMSE
- **Orientation Accuracy**: 0.004 radians RMSE
- **Velocity Accuracy**: 3.654 m/s RMSE
- **Dataset**: KITTI Vision Benchmark Suite
- **Processing Time**: Real-time capable (108 frames processed)

![Comprehensive Results](3d_ekf_results.png)

## Project Overview 🚀
This project implements a high-precision 3D Extended Kalman Filter (EKF) that fuses GPS and IMU data for accurate vehicle state estimation. The system provides real-time tracking of:
- 3D Position (x, y, z)
- 3D Orientation (roll, pitch, yaw)
- 3D Velocity (vx, vy, vz)

## Key Features ⭐
1. **Sensor Fusion**
   - GPS position measurements with accuracy-based noise modeling
   - IMU data (accelerometer + gyroscope) integration
   - Adaptive measurement noise handling

2. **Advanced Processing**
   - Real-time capable implementation
   - Automatic gravity compensation
   - Body-to-world frame transformations
   - Timestamp synchronization

3. **Robust Error Handling**
   - GPS accuracy thresholding
   - Numerical stability safeguards
   - Outlier detection and rejection

## Results Visualization 📊
The results plot (3d_ekf_results.png) shows 9 subplots:

**Top Row**:
- 3D trajectory (estimated vs ground truth)
- Orientation components over time
- Velocity components over time

**Middle Row**:
- Position error analysis
- Orientation error analysis
- Velocity error analysis

**Bottom Row**:
- GPS accuracy metrics
- Raw IMU measurements
- Filter innovation analysis

## Sample Output 📝
```python
Starting 3D EKF processing...
Loading timestamps...
Successfully loaded 108 timestamps
Loading OXTS data...
Converting GPS coordinates to local frame...
Initializing EKF...
Running EKF...
Processing frame 100/108
Computing performance metrics...

RMSE Values:
Position RMSE: 0.118 m
Orientation RMSE: 0.004 rad
Velocity RMSE: 3.654 m/s
```

## Implementation Details 🔧
### State Vector (9-dimensional)
```python
X = [x, y, z,           # Position
     roll, pitch, yaw,  # Orientation
     vx, vy, vz]        # Velocity
```

### Core Components
1. **GPS Integration**
   - Position updates with accuracy weighting
   - Adaptive measurement noise covariance
   - Local coordinate frame conversion

2. **IMU Processing**
   - Accelerometer gravity compensation
   - Gyroscope angular rate integration
   - Body-to-world frame transformations

### Project Structure
```
sensor_fusion/
├── ekf.py              # Main EKF implementation
├── ekf_example.py      # Usage example
├── 3d_ekf_results.png  # Results visualization
└── data/               # KITTI dataset (not included)
```

## Getting Started 🚀
1. **Setup**
   ```bash
   git clone https://github.com/anujpatel1761/sensor_fusion.git
   cd sensor_fusion
   ```

2. **Dependencies**
   ```bash
   conda create -n sensor_fusion python=3.8
   conda activate sensor_fusion
   pip install numpy matplotlib datetime
   ```

3. **Run**
   ```bash
   python ekf.py
   ```

## Data Requirements 📊
- KITTI Vision Benchmark Suite
- Required sensors:
  - GPS (position data)
  - IMU (accelerometer + gyroscope)
  - Timestamps for synchronization

## Future Improvements 🔄
- [ ] ROS integration
- [ ] Real-time visualization
- [ ] Multi-threading support
- [ ] Additional sensor support

## References 📚
1. KITTI Vision Benchmark Suite
2. Probabilistic Robotics (Thrun et al.)
3. Extended Kalman Filter theory

## License 📜
MIT License

## Contact 📫
- GitHub: [@anujpatel1761](https://github.com/anujpatel1761)
```
