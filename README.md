# 西北工业大学队伍的自主巡检方案

### 方案使用雷达-IMU数据融合的激光惯性里程计，加入 SC-A-LOAM 实现回环闭合和位姿图优化，改进自适应轨迹规划技术作为规划器，最终设计**基于INDI的内环姿态控制+基于TinyMPC的外环位置/速度控制的控制器**

## 实验配置

- 环境：ROS1 Noetic GAZEBO
- 激光雷达：Mid360
- IMU：设计的高频率IMU模块
- 驱动包：Livox SDK2

## 算法环境配置

```bash
# Ceres 2.1.0
sudo apt-get -y install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
wget -O ceres-solver.zip https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip
unzip -q ceres-solver.zip -d "${TRDPARTY_DIR}"
pushd "${TRDPARTY_DIR}/ceres-solver-2.1.0"
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=TRUE ..
make -j8
sudo make install

# Eigen 3.3.7
wget -O eigen3.zip <https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip>
unzip -q eigen3.zip -d "${TRDPARTY_DIR}"
pushd "${TRDPARTY_DIR}/eigen-3.3.7"
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=TRUE ..
sudo make install
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# GTSAM
cd ~
git clone https://github.com/borglab/gtsam.git
mkdir build && cd build
cmake -D GTSAM_USE_SYSTEM_EIGEN=ON ..
make
sudo make install

# Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build && cmake ..
make
sudo make install

# 下载 & 编译项目
git clone https://github.com/emNavi/Fast-LIO2.git

cd Fast-LIO2
catkin_make
```

## 使用

Mid360激光雷达上电，插入Mid360激光雷达网口至电脑，[配置好 livox_ros_driver2 驱动包中的 IP 地址](https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file#4-lidar-config)，确保雷达可使用

```bash
# 开启 Fast_LIO2 算法
bash ./run_fast_lio.sh 

# 开启 Ego-Planner 算法（需要去预设规划航点）
bash ./run_ego_planner.sh
```

注：[预设规划航点可修改该文件](https://github.com/emNavi/Fast-LIO2/blob/main/src/ego-planner-swarm-v1/src/planner/plan_manage/launch/param.xml)
