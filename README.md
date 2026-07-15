# PX4 + EGO-Planner + MID360 + FAST-LIO 仿真工作空间

本仓库使用 Git submodule 管理各个 ROS 功能包。每个功能包都可以独立克隆，也可以一次性递归克隆完整 `catkin_ws`。

简述：egoplanner在gazebo环境下，使用px4无人机和fastlio定位进行仿真，且便于一比一还原至实机部署
1.基础环境配置
- 安装 ROS Noetic
sudo mkdir -p /usr/share/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1-latest.list
sudo apt update

sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-tf \
  python3-rosdep \
  python3-catkin-tools \
  python3-vcstool
sudo rosdep init
rosdep update
- 安装 MAVROS GeographicLib 数据
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
- 明确安装 Gazebo Classic 11
sudo apt install -y \
  gazebo11 \
  libgazebo11-dev \
  libeigen3-dev \
  libopencv-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libxml2-utils \
  libimage-exiftool-perl \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libgstreamer-plugins-base1.0-dev \
  bc
gazebo --version
gzserver --version
pkg-config --modversion gazebo
- 安装并编译 PX4 v1.13.3
git clone --recursive --branch v1.13.3 \
  https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot/
git submodule update --init --recursive
git describe --tags --exact-match
由于这个老版本的 Python requirements 与新 pip/Empy 可能不兼容，先固定版本：
python3 -m pip install --user "pip<24.1"
cd PX4-Autopilot/
bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
随后固定老 PX4 所需的 Empy
python3 -m pip uninstall -y em 2>/dev/null || true
python3 -m pip install --user --force-reinstall "empy==3.3.4"
先测试原始 Iris 仿真
cd PX4-Autopilot/
make px4_sitl gazebo
正常情况会打开 Gazebo Classic 11， 无人机和 PX4 控制台，如下图
[图片]
- 安装 Livox-SDK2
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2

mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install
sudo ldconfig
检查
ldconfig -p | grep livox
如果能看到 /usr/local/lib 下的 Livox SDK 库，说明安装完成。

- 创建一个工作空间用来存放我们后续的功能包
mkdir -p /home/lin/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git 
cd livox_ros_driver2
# 我是用的ROS1，如果使用ROS2参考原仓库安装
./build.sh ROS1
# 激活全局路径source ../../devel/setup.bash
先安装可由 rosdep 识别的依赖
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
- 安装livox_laser_simulation
cd ~/catkin_ws/src
# 拉取Custom这个分支比较好，不用消息转化，直接可以给Fastlio使用，如果要用其他消息，就拉取其他分支就行
git clone https://github.com/linny-tech/Mid360_imu_sim.git
cd ..
catkin_make
# 最好在~/.bashrc文件中全局激活工作空间路径source devel/setup.bash
先单独测试雷达插件（记得先在bashrc里加上source /home/.../catkin_ws/devel/setup.bash）
roslaunch livox_laser_simulation livox_simulation.launch
在另一个终端中验证，有输出就没问题
rostopic type /scan
- 复制 MID360 和组合无人机模型
cd ~
git clone https://github.com/qiurongcan/Mid360_px4.git

cp -a \
  /home/你自己的用户名/Mid360_px4/Mid360 \
  /home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/

cp -a \
  /home/你自己的用户名/Mid360_px4/iris_mid360 \
  /home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/
检查
ls /home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360
ls /home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/iris_mid360
- 修改 MID360 CSV 绝对路径，在/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360下
     见71行，一定一定要修改为你电脑的路径，这样才能成功执行，还有38行和68，70行改成我图片所示（后面三个是用来同步仿真时间的）
[图片]
[图片]
70          <downsample>1</downsample>
71          <csv_file_name>/home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360/livox_mid40/scan_mode/mid360.csv</csv_file_name>
72          <ros_topic>/scan</ros_topic>
- 配置.bashrc，可以直接在末尾加上我这套（记得把路径改成自己的用户名），自己注意一下与原来重复的，这里我直接列了总的，有一些后面才用到
# ROS Noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

# EGO Gazebo workspace
if [ -f /home/自己的用户名/ego_gazebo_ws/devel/setup.bash ]; then
    source /home/自己的用户名/ego_gazebo_ws/devel/setup.bash
fi

# Main workspace: livox_ros_driver2, MID360 plugin, FAST-LIO
if [ -f /home/自己的用户名/catkin_ws/devel/setup.bash ]; then
    source /home/自己的用户名/catkin_ws/devel/setup.bash
fi

# PX4 v1.13.3 + Gazebo Classic
if [ -f /home/自己的用户名/PX4-Autopilot/Tools/setup_gazebo.bash ]; then
    source /home/自己的用户名/PX4-Autopilot/Tools/setup_gazebo.bash \
        /home/自己的用户名/PX4-Autopilot \
        /home/自己的用户名/PX4-Autopilot/build/px4_sitl_default
fi

# Explicit ROS package roots: restore paths overwritten by catkin setup files
export ROS_PACKAGE_PATH="/home/自己的用户名/ego_gazebo_ws/src:/home/自己的用户名/catkin_ws/src:/home/lin/PX4-Autopilot:/home/lin/PX4-Autopilot/Tools/sitl_gazebo:/opt/ros/noetic/share"

# Gazebo plugins: PX4 plugins + MID360 simulation plugin
export GAZEBO_PLUGIN_PATH="/home/自己的用户名/catkin_ws/devel/lib:/home/自己的用户名/PX4-Autopilot/build/px4_sitl_default/build_gazebo:${GAZEBO_PLUGIN_PATH}"

# Shared libraries
export LD_LIBRARY_PATH="/usr/local/lib:/home/自己的用户名/catkin_ws/devel/lib:/home/自己的用户名/PX4-Autopilot/build/px4_sitl_default/build_gazebo:${LD_LIBRARY_PATH}"

# VMware Gazebo compatibility
export SVGA_VGPU10=0
- 检查，看看是不是都能找到这些包
rospack find px4
rospack find mavlink_sitl_gazebo
rospack find livox_laser_simulation

echo "$GAZEBO_MODEL_PATH"
echo "$GAZEBO_PLUGIN_PATH"
echo "$LD_LIBRARY_PATH"
- 检查没问题后，启动 PX4 + MID360
roslaunch px4 mavros_posix_sitl.launch \
  vehicle:=iris \
  sdf:=/home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/iris_mid360/iris_mid360.sdf
rostopic type /scan
rostopic hz /scan
rostopic echo /scan
应该能看到一个带着mid360的飞机且点云话题/scan有输出如下图
[图片]

这样就完成了基础环境的配置，接下来将引入ego算法和自建gazebo地图

- 先把我仓库里的camera_pose_node    ego_gazebo_worlds  这两个功能包克隆下来并编译
cd catkin_ws/src

git clone https://github.com/linny-tech/ego_gazebo_worlds.git
git clone https://github.com/linny-tech/camera_pose_node.git
验证，看看启动后是否有一架带mid360的飞机和障碍地图，如下
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
[图片]
- 克隆ego并编译
cd catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git

cd ..
catkin_make
- 克隆飞行控制功能包offboard_hover
cd catkin_ws/src
git clone https://github.com/linny-tech/offboard_hover.git
cd ..
catkin_make --pkg offboard_hover
- 添加fastlio
sudo apt update

sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    python3-dev \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions
cd catkin_ws/src
git clone --recursive https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init --recursive

改为 livox_ros_driver2
cd catkin_ws/src/FAST_LIO

grep -RIl \
    --exclude-dir=.git \
    'livox_ros_driver' \
    CMakeLists.txt package.xml include src |
xargs -r sed -i 's/livox_ros_driver/livox_ros_driver2/g'
grep -R "livox_ros_driver2" \
    CMakeLists.txt package.xml include src | head -20
有输出以下代表成功
livox_ros_driver2
livox_ros_driver2/CustomMsg.h
livox_ros_driver2::CustomMsg
检查是否有残留头文件
grep -R "livox_ros_driver/CustomMsg" \
    CMakeLists.txt package.xml include src
开一个新终端编译
cd catkin_ws

catkin_make -DCMAKE_BUILD_TYPE=Release 
编译成功后检查一下是否能找到
source ~/.bashrc
rospack profile
rospack find fast_lio
- 配置 MID360 仿真话题
FAST_LIO_DIR=$(rospack find fast_lio)

test -f "$FAST_LIO_DIR/config/mid360.yaml" || \
cp "$FAST_LIO_DIR/config/avia.yaml" \
"$FAST_LIO_DIR/config/mid360.yaml"

nano "$FAST_LIO_DIR/config/mid360.yaml"
修改我标红的地方
common:
    lid_topic: "/scan"
    imu_topic: "/mavros/imu/data"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 1
    scan_line: 4
    blind: 0.5

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360
    det_range: 30.0
    extrinsic_est_en: false
    extrinsic_T: [ -0.005, 0.005, 0.097 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1 ]

publish:
    path_en: true
    scan_publish_en: true
    dense_publish_en: true
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: false
    interval: -1
保存后检查：
grep -nE \
'lid_topic|imu_topic|lidar_type|scan_line|blind|extrinsic_T' \
"$FAST_LIO_DIR/config/mid360.yaml"
- 创建仿真启动文件
test -f "$FAST_LIO_DIR/launch/mapping_mid360_sim.launch" || \
cp "$FAST_LIO_DIR/launch/mapping_avia.launch" \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"

sed -i 's#config/avia.yaml#config/mid360.yaml#' \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"
grep rosparam \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"
- 由于px4使用mid360进入offbroad需要修改飞控参数，有两种方法，qgc地面站直接修改或者px4控制台命令修改，这里用简单一些的qgc修改
- 先启动仿真
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
