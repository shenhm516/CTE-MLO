# CTE-MLO: A Continuous-time and Efficient Multi-LiDAR Odometry with Localizability-aware Point Cloud Sampling

<p align="center">
<a href="https://ieeexplore.ieee.org/document/10900545"><img src="https://img.shields.io/badge/Paper-IEEE%20TFR-004088.svg" alt="Paper" /></a>
<a href="https://arxiv.org/abs/2408.04901"><img src="https://img.shields.io/badge/ArXiv-2408.04901-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
<a href="https://www.youtube.com/watch?v=Q29PGPitHUI"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
<a href="https://www.bilibili.com/video/BV1xfZcYvEGC"><img src="https://img.shields.io/badge/哔哩哔哩-Bilibili-fb7299" alt="Bilibili" /></a>
</p>

<div align="center">
<a href="https://youtu.be/Q29PGPitHUI" target="_blank"><img src="doc/CTE-MLO-Titel.png" alt="video" width="90%" /></a>
</div>

## Features
1. Continuous-time: A Gaussian process representation is adopted to describe the continuous-time trajectory, which enables a natural combination of the continuous-time LiDAR odometry with the Kalman filter.
2. Decentralized Multi-LiDAR Synchronization: combine multiple LiDAR measurements using the LiDAR splitting technique, which is robust to arbitrary LiDAR failure.
3. Efficient: A localizability-aware point cloud sampling method is proposed to improve the real-time performance of MLO, which quantifies the localizability contribution for each point, and ensures localizability using a minimal number of points with high localizability contribution.
4. Extensible: CTE-MLO supports different LiDAR configurations, including spinning LiDAR, non-repetitive scan LiDAR, and solid-state LiDAR.

## Setup
1. Install Ubuntu with ROS. This project has been tested on Ubuntu 20.04 (ROS Noetic) with the default PCL 1.8 and Eigen 3.3.7.
2. Install the [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2/tree/master)
    ```
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```
3. Install [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) and [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2).
    ```
    git clone https://github.com/Livox-SDK/livox_ros_driver.git ~/ws_livox/src
    cd ws_livox
    catkin_make
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/ws_livox/src/livox_ros_driver2
    cd ~/ws_livox/src/livox_ros_driver2
    ./build.sh ROS1
    ```
4. Clone CTE-MLO and catkin_make
    ```
    mkdir -p ~/ctemlo_ws/src
    cd ~/ctemlo_ws/src
    git clone https://github.com/shenhm516/CTE-MLO.git
    cd ..
    source ~/ws_livox/devel/setup.bash
    catkin_make
    ```

## Extrinsic
The first row of the extrinsic matrix represents the extrinsic from ```0#LiDAR``` to the body frame (<img src="https://latex.codecogs.com/svg.image?{}^b{\mathbf{T}}_{l_0}"/>), while the remaining rows represent the extrinsic of other LiDARs to ```0#LiDAR``` (<img src="https://latex.codecogs.com/svg.image?{}^{l_0}{\mathbf{T}}_{l_i}" />).

We provide a Python script to calculate extrinsics using the NTU-VIRAL dataset as an example.
```
python3 ctemlo_ws/src/CTE-MLO/script/extrinsic.py
```
## Run on [NTU-VIRAL Dataset](https://ntu-aris.github.io/ntu_viral_dataset)
```
source ~/ctemlo_ws/devel/setup.bash
roslaunch cte_mlo mapping_NTUviral.launch   
```

*Remark:* Don't forget to set the path of bag in ```mapping_NTUviral.launch```.

## Run on [MCD](https://mcdviral.github.io)
```
source ~/ctemlo_ws/devel/setup.bash
roslaunch cte_mlo mapping_mcd.launch   
```
<div align="center">
<img src="doc/MCDMappingResults.png"  width="100%" />
</div>

The [evo](https://github.com/MichaelGrupp/evo) package can be used to evaluate the ATE of CTE-MLO.
<div align="center">
<img src="doc/MCDATEResults.png"  width="90%" />
</div>

<!-- *Remark:* This results is running with single Livox Mid-70. -->
## LiDAR-Only Aggressive Motion Estimation
The CTE-MLO can be easily configured to operate in single LiDAR mode by set the ```lidar_num: 1``` in the ```.yaml```.
CTE-MLO is tested over three aggressive data sequences provided by [Point-LIO](https://github.com/hku-mars/Point-LIO), including ```PULSAR```, ```Spinning```, and ```pendulum```.
<div align="center">
<img src="doc/PULSAR.gif"  width="32.5%" />
<img src="doc/Spinning.gif"  width="32.5%" />
<img src="doc/pendulum.gif"  width="32.5%" />
</div>

Simple Test:
```
source ~/ctemlo_ws/devel/setup.bash
roslaunch cte_mlo mapping_pointlio.launch
rosbag play PULSAR.bag --clock
```

*Remark:* We recommend setting the ```PCS``` to ```False``` when working with a single LiDAR mode.

## LiDAR-Only Localization on Pre-build Map
The CTE-MLO can be easily configured to perform localization on a pre-built map by setting ```use_prebuild_map: True``` in the ```.yaml```.
Don't forget to specify the path of the pre-built map by setting the ```offline_map_path``` in ```.yaml```.
The ```init_T``` in the ```.yaml``` file represents the initial pose of the first scan in the pre-build map, which can be obtained either from a ground truth trajectory or through point cloud registration between the first scan and the pre-built map. To facilitate registration, we provide a [point cloud registration GUI](https://github.com/TobyLyu/icp_gui).


Testing on MCD Prior Map:
```
source ~/ctemlo_ws/devel/setup.bash
roslaunch cte_mlo mapping_mcd_prior.launch
rosbag play ntu_day_01/ntu_day_01_mid70.bag
```
*Remark:* Please wait until the terminal displays ```The Pre-build Map has been Voxelized``` before playing the rosbag.
## Use Livox Lidar with sensor_msgs/PointCloud2
We provide an example to test CTE-MLO using the Livox Lidar with the sensor_msgs/PointCloud2 message type.
```
source ~/ctemlo_ws/devel/setup.bash
roslaunch cte_mlo mapping_livoxpc2.launch   
```
## Acknowledgments
This project is developed based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), [BALM](https://github.com/hku-mars/BALM), and [X-ICP](https://sites.google.com/leggedrobotics.com/x-icp). Thanks for their excellent work!

## Additional Information
1. New Functions: A sliding window was added to calculate localizability for point cloud sampling; Replace IEKF with IESKF.
2. Contact us: For any technical issues, please feel free to contact me (hongming.shen@ntu.edu.sg).
3. Citation: If you find this work useful or interesting, please kindly give us a star ⭐; If our repository supports your academic projects, please cite our paper. Thank you!
    ```bibtex
    @ARTICLE{10900545,
    author={Shen, Hongming and Wu, Zhenyu and Hui, Yulin and Wang, Wei and Lyu, Qiyang and Deng, Tianchen and Zhu, Yeqing and Tian, Bailing and Wang, Danwei},
    journal={IEEE Transactions on Field Robotics}, 
    title={CTE-MLO: Continuous-Time and Efficient Multi-LiDAR Odometry With Localizability-Aware Point Cloud Sampling}, 
    year={2025},
    volume={2},
    number={},
    pages={165-187},
    doi={10.1109/TFR.2025.3543142}}
    ```