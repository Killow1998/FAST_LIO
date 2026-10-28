# FAST-LIO: shared C++ core, Noetic and Humble

The `main` branch builds one C++17 estimator with two native ROS adapters.
The stable-window IMU initialization and configurable gravity repair are shared
by both adapters. The core uses Eigen, PCL and OpenMP; it has no ROS dependency.

- `include/fast_lio/`: public configuration, sensor/output types and `Mapper` API.
- `src/core/`: per-instance input synchronization, registration, local map and outputs.
- `src/imu_process_*.cpp`, `src/preprocess*.cpp`: shared initialization and deskew/preprocessing.
- `adapters/`: one parameter catalog and message conversion helpers.
- `noetic/`, `humble/`: native package discovery, launch/config files and thin nodes.

Each `Mapper` has one serial owner. Feed sensor samples, call `process()` until
it returns `Waiting`, and call `finish()` after stopping inputs to save pending
PCD data. An optional synchronous `OdometryObserver` receives IMU predictions
during propagation and scan odometry immediately after correction, before map
insertion and cloud conversion. Observers must not re-enter the mapper.
Frame results contain immutable clouds. IMU and output pose stamps use integer
nanoseconds; LiDAR input stamps use seconds. Topic and frame names live in the
adapters. Filter state and the ikd-Tree remain private to each instance.

See [the refactor validation record](docs/core_wrappers_20261027.md) for baseline
identity, output comparisons, resource measurements and remaining hardware tests.

## Related Works and Extended Application

**SLAM:**

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
3. [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): A robust, real-time LiDAR-IMU extrinsic initialization and synchronization package..
4. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): The integration of FAST-LIO with **Re-localization** function module.
5. [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) | [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2): A state-of-art LiDAR-inertial-visual odometry (LIVO) system with high computational efficiency, robustness, and pixel-level accuracy.

**Control and Plan:**

1. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
2. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
3. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
4. [Bubble Planner](https://arxiv.org/abs/2202.12177): Planning High-speed Smooth Quadrotor Trajectories using Receding Corridors.

<!-- 10. [**FAST-LIVO**](https://github.com/hku-mars/FAST-LIVO): Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry. -->

## FAST-LIO
**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package address many key issues:
1. Fast iterated Kalman filter for odometry optimization;
2. Automaticaly initialized at most steady environments;
3. Parallel KD-Tree Search to decrease the computation;

This fork vendors `ikd-Tree` directly under `include/ikd-Tree`, so cloning the repository does not require any git submodule initialization.
This fork intentionally omits the bundled `doc/` media assets to stay lightweight; the original figures, GIFs, and PDFs are referenced from the upstream repository instead.

## FAST-LIO 2.0 (2021-07-05 Update)
**Related video:**  [FAST-LIO2](https://youtu.be/2OvjGnxszf8),  [FAST-LIO1](https://youtu.be/iYCY6T79oNU)

**Upstream media references:** [FAST-LIO2 demo GIF](https://github.com/hku-mars/FAST_LIO/blob/main/doc/real_experiment2.gif), [FAST-LIO1 demo GIF](https://github.com/hku-mars/FAST_LIO/blob/main/doc/ulhkwh_fastlio.gif), [pipeline overview](https://github.com/hku-mars/FAST_LIO/blob/main/doc/overview_fastlio2.svg)

**New Features:**
1. Incremental mapping using [ikd-Tree](https://github.com/hku-mars/ikd-Tree), achieve faster speed and over 100Hz LiDAR rate.
2. Direct odometry (scan to map) on Raw LiDAR points (feature extraction can be disabled), achieving better accuracy.
3. Since no requirements for feature extraction, FAST-LIO2 can support many types of LiDAR including spinning (Velodyne, Ouster) and solid-state (Livox Avia, Horizon, MID-70) LiDARs, and can be easily extended to support more LiDARs.
4. Support external IMU.
5. Support ARM-based platforms including Khadas VIM3, Nivida TX2, Raspberry Pi 4B(8G RAM).

**Related papers**:

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://github.com/hku-mars/FAST_LIO/blob/main/doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**Contributors**

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

## 1. Prerequisites

Use Ubuntu 22.04 / ROS 2 Humble or Ubuntu 20.04 / ROS 1 Noetic, with Eigen3,
PCL, OpenMP, CMake >= 3.16, ccache and a C++17 compiler. GTest is needed for core tests.
Noetic can also be built in a Focal container on a Jammy host.

The Livox message contract is unchanged: Humble uses
`livox_ros_driver2/msg/CustomMsg`; Noetic uses `livox_ros_driver/CustomMsg`.
Install and source the corresponding native driver before building the wrapper.
The hardware driver itself is separate from this repository.

## 2. Build

Clone the `main` branch. Both native packages retain the name `fast_lio`, so
select one before workspace discovery. The selector only changes ignored build
markers; both adapters remain in the same source tree. Build and install paths
below are separate from an existing robot installation.

### Humble

From a colcon workspace containing this repository at `src/FAST_LIO`:

```bash
source /opt/ros/humble/setup.bash
# Source the workspace containing livox_ros_driver2.
bash src/FAST_LIO/scripts/select_distro_wrapper.sh humble
colcon --log-base log/fast_lio/humble-release build --packages-select fast_lio \
  --build-base build/fast_lio/humble-release --install-base install/fast_lio/humble-release \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
source install/fast_lio/humble-release/setup.bash
ros2 launch fast_lio mapping_mid360.launch.py
```

### Noetic

From a catkin workspace containing this repository and `livox_ros_driver`:

```bash
source /opt/ros/noetic/setup.bash
bash src/FAST_LIO/scripts/select_distro_wrapper.sh noetic
catkin config --profile release --build-space build/fast_lio/noetic-release \
  --devel-space devel/fast_lio/noetic-release --install-space install/fast_lio/noetic-release \
  --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
catkin build --profile release fast_lio -j4 -p1
source install/fast_lio/noetic-release/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

ROS 1 parameters accept the original global namespace and private overrides.
Configurations are under `noetic/config` and `humble/config`; installed launch
names and topic names are retained. Root `config/` remains a symlink to `humble/config` for existing frozen startup
commands. Native launch sources live in their wrapper directories. ROS 1 source
configuration is always under `noetic/config`.

### Core without ROS

```bash
cmake -S src/FAST_LIO/core -B build/fast_lio/core-release \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build/fast_lio/core-release -j4
(cd build/fast_lio/core-release && ctest --output-on-failure)
cmake --install build/fast_lio/core-release --prefix "$PWD/install/fast_lio/core-release"
```

A C++ consumer can use `find_package(fast_lio_core REQUIRED)` and link
`fast_lio::fast_lio_core`. The SDK is also installed by both native wrappers.
Only the public configuration/output headers, Eigen and PCL common are exposed;
IKFoM, ikd-Tree, preprocessing and PCL IO/filter implementation headers remain
internal.

For development use a separate build/install directory and
`-DCMAKE_BUILD_TYPE=RelWithDebInfo`. Optimization flags follow the selected CMake
configuration; no unconditional `-O3` or `-march=native` is added. Set
`CMAKE_BUILD_PARALLEL_LEVEL=4` when limiting build concurrency.
`FAST_LIO_MATCH_THREADS` selects the point-matching workers. Its default preserves
Humble/core's single worker and Noetic's existing CPU-dependent default (two
workers for four visible x86 cores, three for more than four). Other OpenMP users
in the process are not reconfigured by this matching loop.

## 3. Directly run
Noted:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.

C. We recommend to set the **extrinsic_est_en** to false if the extrinsic is give. As for the extrinsic initiallization, please refer to our recent work: [**Robust Real-time LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

### IMU startup in this fork

Keep the sensor stationary until `IMU Initial Done` and the first `/Odometry`
message. Initialization uses a recent, continuous one-second IMU window, with
at least 20 samples. It checks both noise and changes between the two half-window
means before committing gravity and gyro bias. Older startup motion leaves the
window automatically; elapsed time alone never forces acceptance.

The checks use estimated uncertainty of the acceleration and gyro means instead
of a fixed raw acceleration-noise cutoff. Each mean variance is the larger of
the sample variance divided by sample count and the variance of four batch
means divided by four. This retains correlated variation instead of treating
every sample as independent evidence. Scalar variance traces preserve decisions
under a fixed change of sensor orientation.

The normalized acceleration-mean standard error must not exceed 0.0025 g, and
the gyro-mean standard error must not exceed 0.005 rad/s. These use half the
existing half-window stability budgets: mean changes must stay within 0.005 g
and 0.01 rad/s. Angular-rate RMS remains limited to 0.05 rad/s. No per-sensor
noise threshold is learned by progressively relaxing a failed check.
Acceleration statistics are normalized by the measured gravity magnitude, so
recorded g and m/s² inputs use the same decision. These are bounded
stationary-start checks, not a statistical proof of rest, a dynamic initializer,
or an IMU calibration. Four batches cannot model every noise correlation.
Constant acceleration and sufficiently slow constant rotation cannot be
distinguished from gravity and bias using this window alone.

While waiting, the node reports at most ten window summaries per initialization,
at most once per sensor second, and emits no initialized point cloud or odometry.
A recording that starts
in continuous motion may therefore wait indefinitely; provide a stationary
lead-in instead of treating its missing initial poses as a successful start.
IMU gaps above 0.1 s, backward time and invalid measurements restart the window.
Storage is bounded at 2,048 samples. After initialization, the window is released
and the existing tracking pipeline continues unchanged.

`common.gravity_m_s2` sets both the acceleration scale and the filter's gravity
magnitude. The initialization-to-deskew boundary uses the accepted scan end
time and explicit last-IMU acceleration/rate values.

With `BUILD_TESTING=ON`, `test_imu_initialization` covers motion rejection/recovery,
noisy stationary input with gyro bias, acceleration units, timestamps and bounded
storage. `test_imu_process_initialization` checks the committed filter state and
stationary acceleration cancellation for three configured gravity magnitudes.

### 3.1 For Mid-360
Connect to your PC to Livox Mid-360 LiDAR by following  [Livox-ros-driver2 installation](https://github.com/Livox-SDK/livox_ros_driver2), then
```bash
    cd ~/$ROS2_WS
    source install/setup.bash
    ros2 launch fast_lio mapping_mid360.launch.py
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
- For livox serials, FAST-LIO only support the data collected by the custom message format since only its `livox_ros_driver2/CustomMsg` data structure produces the timestamp of each LiDAR point which is very important for the motion undistortion.

### 3.2 For other Livox LiDARs (Avia, Horizon, etc)

`mapping_avia.launch.py` or `mapping_horizon.launch.py` theoretically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters before run:

Edit `humble/config/avia.yaml` or `humble/config/horizon.yaml` to set the below parameters:

1. LiDAR point cloud topic name: `lid_topic`
2. IMU topic name: `imu_topic`
3. Translational extrinsic: `extrinsic_T`
4. Rotational extrinsic: `extrinsic_R` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.

### 3.3 For Velodyne or Ouster (Velodyne as an example)

Step A: Setup before run

Edit `humble/config/velodyne.yaml` to set the below parameters:

1. LiDAR point cloud topic name: `lid_topic`
2. IMU topic name: `imu_topic` (both internal and external, 6-aixes or 9-axies are fine)
3. Set the parameter `timestamp_unit` based on the unit of **time** (Velodyne) or **t** (Ouster) field in PoindCloud2 rostopic
4. Line number (we tested 16, 32 and 64 line, but not tested 128 or above): `scan_line`
5. Translational extrinsic: `extrinsic_T`
6. Rotational extrinsic: `extrinsic_R` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).

Step B: Run below
```bash
    ros2 launch fast_lio mapping_velodyne.launch.py
```

Step C: Run LiDAR's ros driver or play rosbag.

### 3.4 For MARSIM Simulator

Install MARSIM: https://github.com/hku-mars/MARSIM and run MARSIM as below

```
cd ~/$MARSIM_ROS_DIR$
roslaunch test_interface single_drone_avia.launch
```

Then Run FAST-LIO:

```
ros2 launch fast_lio mapping_marsim.launch.py
```

### 3.5 PCD file save

Enable `pcd_save.pcd_save_en`. In this fork, the native `config/mid360.yaml` enables it by default, so Mid-360 runs will accumulate global-frame scans and save them to `FAST_LIO/PCD/scans.pcd` when the node exits normally. Generated `.pcd` files are ignored by git, while the empty `PCD/` directory is kept in the repository.

The C++ SDK can set `Config::pcd_directory` to select an output directory.
Native wrappers retain the source-tree `PCD/` default. The previously inert ROS
parameters `map_file_path`, `filter_size_corner` and `mapping.fov_degree` remain
inert; this refactor does not assign them new behavior. A positive save interval
writes numbered chunks; normal shutdown writes the remaining `scans.pcd`.
PCD saving is owned by the core and does not require a ROS publisher.

### 3.6 Optional runtime log

`runtime_pos_log_enable` enables one asynchronous `runtime.log`. The writer batches
every 250 ms, caps queued text at 1 MiB and stops appending at 100,000,000 bytes
across restarts. Queue/storage drops and write failures are reported at shutdown.
Set `FAST_LIO_LOG_DIR` to an instance-owned directory; otherwise the directory is
`$HOME/.ros/fast_lio/logs`. Do not share it between simultaneous log writers.
The default is disabled. The old unconditional source-tree `Log/` files and
per-IMU disk writes are removed. The new row format is scan-end time, space-separated position xyz and quaternion
xyzw, total/match/solve seconds, downsampled point count and tree size.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

## 4. Rosbag Example
### 4.1 Livox Avia Rosbag
Files: Can be downloaded from [google drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)

Reference result images remain available in the upstream repository: [HKU_LG_Indoor](https://github.com/hku-mars/FAST_LIO/blob/main/doc/results/HKU_LG_Indoor.png), [HKU_MB_002](https://github.com/hku-mars/FAST_LIO/blob/main/doc/results/HKU_MB_002.png)

Run:
```
roslaunch fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag

```

### 4.2 Velodyne HDL-32E Rosbag

**NCLT Dataset**: Original bin file can be found [here](http://robots.engin.umich.edu/nclt/).

We produce [Rosbag Files](https://drive.google.com/drive/folders/1blQJuAB4S80NwZmpM6oALyHWvBljPSOE?usp=sharing) and [a python script](https://drive.google.com/file/d/1QC9IRBv2_-cgo_AEvL62E1ml1IL9ht6J/view?usp=sharing) to generate Rosbag files: ```python3 sensordata_to_rosbag_fastlio.py bin_file_dir bag_name.bag```
    
Run:
```
ros2 launch fast_lio mapping_velodyne.launch.py
ros2 bag play YOUR_DOWNLOADED.bag
```

## 5.Implementation on UAV
In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a small-scale quadrotor which can carry a Livox Avia LiDAR with 70 degree FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz Intel i7-8550U CPU and 8 G RAM, as shown in below.

The main structure of this UAV is 3d printed (Aluminum or PLA), the .stl file will be open-sourced in the future.

Upstream references: [UAV photo](https://github.com/hku-mars/FAST_LIO/blob/main/doc/uav01.jpg), [UAV system diagram](https://github.com/hku-mars/FAST_LIO/blob/main/doc/uav_system.png), [ground configuration PDF](https://github.com/hku-mars/FAST_LIO/blob/main/doc/uav_ground.pdf)

## 6.Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
