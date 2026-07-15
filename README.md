# PX4 + EGO-Planner + MID360 + FAST-LIO 仿真工作空间

## **15.2  仿真环境搭建**

<table>
  <tbody>
    <tr>
      <td>操作系统</td>
      <td>Ubuntu 20.04</td>
    </tr>
    <tr>
      <td>ROS</td>
      <td>ROS Noetic（ROS 1）</td>
    </tr>
    <tr>
      <td>PX4</td>
      <td>v1.13.3</td>
    </tr>
    <tr>
      <td>仿真器</td>
      <td>Gazebo Classic 11</td>
    </tr>
  </tbody>
</table>

**简述：egoplanner在gazebo环境下，使用px4无人机和fastlio定位进行仿真，且便于一比一还原至实机部署**

### **1.基础环境配置**

- 安装 ROS Noetic

```bash
添加 ROS 软件源
sudo mkdir -p /usr/share/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros1-latest.list
```

```text
安装 ROS、MAVROS、Gazebo ROS 插件及点云依赖
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
```

```text
初始化 rosdep
sudo rosdep init
rosdep update
```

- 安装 MAVROS GeographicLib 数据

```text
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

- 明确安装 Gazebo Classic 11

```text
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
```

```text
验证：
gazebo --version
gzserver --version
pkg-config --modversion gazebo
```

- 安装并编译 PX4 v1.13.3

```text
克隆时必须指定 tag 并递归获取子模块：
git clone --recursive --branch v1.13.3 \
  https://github.com/PX4/PX4-Autopilot.git

cd PX4-Autopilot/
git submodule update --init --recursive
git describe --tags --exact-match
```

由于这个老版本的 Python requirements 与新 pip/Empy 可能不兼容，先固定版本：

```text
python3 -m pip install --user "pip<24.1"
```

```text
运行 PX4 的基础安装脚本，但跳过 NuttX 和脚本内置的旧仿真安装流程
cd PX4-Autopilot/
bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
```

随后固定老 PX4 所需的 Empy

```text
python3 -m pip uninstall -y em 2>/dev/null || true
python3 -m pip install --user --force-reinstall "empy==3.3.4"
```

先测试原始 Iris 仿真

```text
cd PX4-Autopilot/
make px4_sitl gazebo
```

正常情况会打开 Gazebo Classic 11， 无人机和 PX4 控制台，如下图

![PX4 SITL 与 Gazebo 启动结果](docs/images/ego-planner/ego-01.png)

- 安装 Livox-SDK2

```text
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2

mkdir build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

检查

```text
ldconfig -p | grep livox
```

如果能看到 /usr/local/lib 下的 Livox SDK 库，说明安装完成。

- 创建一个工作空间用来存放我们后续的功能包

```bash
mkdir -p /home/lin/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git 
cd livox_ros_driver2
# 我是用的ROS1，如果使用ROS2参考原仓库安装
./build.sh ROS1
# 激活全局路径source ../../devel/setup.bash
```

先安装可由 rosdep 识别的依赖

```text
cd catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

- 安装livox_laser_simulation

```markdown
cd ~/catkin_ws/src
# 拉取Custom这个分支比较好，不用消息转化，直接可以给Fastlio使用，如果要用其他消息，就拉取其他分支就行
git clone https://github.com/linny-tech/Mid360_imu_sim.git
cd ..
catkin_make
# 最好在~/.bashrc文件中全局激活工作空间路径source devel/setup.bash
```

先单独测试雷达插件（记得先在bashrc里加上source /home/.../catkin_ws/devel/setup.bash）

```text
roslaunch livox_laser_simulation livox_simulation.launch
```

在另一个终端中验证，有输出就没问题

```text
rostopic type /scan
```

- 复制 MID360 和组合无人机模型

```bash
cd ~
git clone https://github.com/qiurongcan/Mid360_px4.git

cp -a \
  /home/你自己的用户名/Mid360_px4/Mid360 \
  /home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/

cp -a \
  /home/你自己的用户名/Mid360_px4/iris_mid360 \
  /home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/
```

检查

```text
ls /home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360
ls /home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/iris_mid360
```

- 修改 MID360 CSV 绝对路径，在/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360下

见71行，一定一定要修改为你电脑的路径，这样才能成功执行，还有38行和68，70行改成我图片所示（后面三个是用来同步仿真时间的）

![MID360 SDF 配置一](docs/images/ego-planner/ego-02.png)

![MID360 SDF 配置二](docs/images/ego-planner/ego-03.png)

```bash
改红标的地方
70          <downsample>1</downsample>
71          <csv_file_name>/home/lin/PX4-Autopilot/Tools/sitl_gazebo/models/Mid360/livox_mid40/scan_mode/mid360.csv</csv_file_name>
72          <ros_topic>/scan</ros_topic>
```

- 配置.bashrc，可以直接在末尾加上我这套（记得把路径改成自己的用户名），自己注意一下与原来重复的，这里我直接列了总的，有一些后面才用到

```markdown
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
```

- 检查，看看是不是都能找到这些包

```bash
rospack find px4
rospack find mavlink_sitl_gazebo
rospack find livox_laser_simulation

echo "$GAZEBO_MODEL_PATH"
echo "$GAZEBO_PLUGIN_PATH"
echo "$LD_LIBRARY_PATH"
```

- 检查没问题后，启动 PX4 + MID360

```text
终端1
roslaunch px4 mavros_posix_sitl.launch \
  vehicle:=iris \
  sdf:=/home/你自己的用户名/PX4-Autopilot/Tools/sitl_gazebo/models/iris_mid360/iris_mid360.sdf
```

```text
终端2
rostopic type /scan
rostopic hz /scan
rostopic echo /scan
```

应该能看到一个带着mid360的飞机且点云话题/scan有输出如下图

![PX4 与 MID360 点云话题测试](docs/images/ego-planner/ego-04.png)

这样就完成了基础环境的配置，接下来将引入ego算法和自建gazebo地图

- 先把我仓库里的camera_pose_node    ego_gazebo_worlds  这两个功能包克隆下来并编译

```text
cd catkin_ws/src

git clone https://github.com/linny-tech/ego_gazebo_worlds.git
git clone https://github.com/linny-tech/camera_pose_node.git
```

验证，看看启动后是否有一架带mid360的飞机和障碍地图，如下

```text
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
```

![多障碍物 Gazebo 仿真环境](docs/images/ego-planner/ego-05.png)

- 克隆ego并编译

```text
cd catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git

cd ..
catkin_make
```

- 克隆飞行控制功能包offboard_hover

```text
cd catkin_ws/src
git clone https://github.com/linny-tech/offboard_hover.git
cd ..
catkin_make --pkg offboard_hover
```

- 添加fastlio

```text
安装依赖
sudo apt update

sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libomp-dev \
    python3-dev \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions
```

```text
cd catkin_ws/src
git clone --recursive https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init --recursive
```

改为 livox_ros_driver2

```text
cd catkin_ws/src/FAST_LIO

grep -RIl \
    --exclude-dir=.git \
    'livox_ros_driver' \
    CMakeLists.txt package.xml include src |
xargs -r sed -i 's/livox_ros_driver/livox_ros_driver2/g'
```

```text
检查结果
grep -R "livox_ros_driver2" \
    CMakeLists.txt package.xml include src | head -20
```

有输出以下代表成功

```text
livox_ros_driver2
livox_ros_driver2/CustomMsg.h
livox_ros_driver2::CustomMsg
```

检查是否有残留头文件

```text
这个应该没有输出
grep -R "livox_ros_driver/CustomMsg" \
    CMakeLists.txt package.xml include src
```

开一个新终端编译

```text
cd catkin_ws

catkin_make -DCMAKE_BUILD_TYPE=Release
```

编译成功后检查一下是否能找到

```text
source ~/.bashrc
rospack profile
rospack find fast_lio
```

- 配置 MID360 仿真话题

```bash
FAST_LIO_DIR=$(rospack find fast_lio)

test -f "$FAST_LIO_DIR/config/mid360.yaml" || \
cp "$FAST_LIO_DIR/config/avia.yaml" \
"$FAST_LIO_DIR/config/mid360.yaml"

nano "$FAST_LIO_DIR/config/mid360.yaml"
```

修改我标红的地方

```yaml
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
```

保存后检查：

```text
grep -nE \
'lid_topic|imu_topic|lidar_type|scan_line|blind|extrinsic_T' \
"$FAST_LIO_DIR/config/mid360.yaml"
```

- 创建仿真启动文件

```bash
test -f "$FAST_LIO_DIR/launch/mapping_mid360_sim.launch" || \
cp "$FAST_LIO_DIR/launch/mapping_avia.launch" \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"

sed -i 's#config/avia.yaml#config/mid360.yaml#' \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"
```

```text
应该输出config/mid360.yaml
grep rosparam \
"$FAST_LIO_DIR/launch/mapping_mid360_sim.launch"
```

- 由于px4使用mid360进入offbroad需要修改飞控参数，有两种方法，qgc地面站直接修改或者px4控制台命令修改，这里用简单一些的qgc修改

- 先启动仿真

```text
roslaunch ego_gazebo_worlds px4_mid360_ego.launch
```

然后启动qgc，进入参数设置页面（这里默认装好了qgc）

```text
EKF2_AID_MASK  修改为24
EKF2_HGT_MODE  修改为vision
COM_RC_IN_MODE 设置为4
COM_RCL_EXCEPT 设置为6
```

![QGroundControl EKF2 参数设置](docs/images/ego-planner/ego-06.png)

![QGroundControl COM 参数设置](docs/images/ego-planner/ego-07.png)

设置完后点右上角的的工具重启飞行器，然后终止仿真程序再次启动就保存了，可以自己进qgc查看是否修改成功

- 到这里仿真所需的所有东西都完成了，接下来进入测试

### **2.仿真测试**

#### **（1）先简要概述一下每个功能包的作用**

livox_ros_driver2 ：mid360驱动包

Mid360_imu_sim ：mid360仿真包

FAST_LIO  ：建图

camera_pose_node  ：把fastlio传出的数据转换成px4能接受的，这个节点可以查看雷达里程计数据与飞控位置数据

offboard_hover  ：起飞程序（这里默认是1.5m），并进入offborad等待ego的规划点

ego-planner  ：避障算法

ego_gazebo_worlds  ：仿真地图的存放地，一共有三种地图都是50*50*4m的，如下

1.多障碍地图  启动命令：roslaunch ego_gazebo_worlds px4_mid360_ego.launch

![多障碍物地图](docs/images/ego-planner/ego-08.png)

2.森林地图  启动命令：roslaunch ego_gazebo_worlds px4_mid360_forest.launch

![森林地图](docs/images/ego-planner/ego-09.png)

3.房屋地图  启动命令：roslaunch ego_gazebo_worlds px4_mid360_residential.launch

![房屋地图](docs/images/ego-planner/ego-10.png)

4.狭小空间地图  启动命令：roslaunch ego_gazebo_worlds px4_mid360_tunnel.launch

![隧道地图](docs/images/ego-planner/ego-11.png)

消息关系

![EGO-Planner 仿真消息关系](docs/images/ego-planner/ego-12.png)

#### **（2）启动顺序（启动前记得把ego中的点云和里程计话题改为mid360发布话题，详情看前面的参数设置）**

```text
终端一，用哪个地图就启动哪个
roslaunch ego_gazebo_worlds px4_mid360_ego.launch //多障碍物地图
roslaunch ego_gazebo_worlds px4_mid360_forest.launch //森林地图
roslaunch ego_gazebo_worlds px4_mid360_residential.launch //房屋地图
roslaunch ego_gazebo_worlds px4_mid360_tunnel.launch //隧道地图
```

![房屋地图](docs/images/ego-planner/ego-10.png)

```text
终端二，看你要不要用rviz发送规划点，这里我用的预巡航点所以用了命令二
roslaunch fast_lio mapping_mid360_sim.launch //有rviz的
roslaunch fast_lio mapping_mid360_sim.launch rviz:=false  //无rviz的
```

![FAST-LIO 与 RViz 启动结果](docs/images/ego-planner/ego-13.png)

```text
终端三
roslaunch camera_pose_node pose_tf.launch
```

![camera_pose_node 启动结果](docs/images/ego-planner/ego-14.png)

```text
终端四，启动这个后会看到飞机起飞1.5m悬停
roslaunch offboard_hover offboard_lidar_hover.launch
```

![offboard_hover 起飞悬停结果](docs/images/ego-planner/ego-15.png)

```text
终端五，启动后发布规划点无人机就会开始走规划路径
roslaunch ego_planner run_in_sim.launch
```

![EGO-Planner 规划运行结果](docs/images/ego-planner/ego-16.png)

#### **（3）对参数修改进行测试**

速度与加速度固定：

```xml
    <arg name="max_vel" value="0.5" />
```

```xml
    <arg name="max_acc" value="0.8" />
```

对以下三个参数进行修改

a.机体膨胀半径  grid_map/obstacles_inflation

b.距障碍物边缘安全距离  optimization/dist0

c.地图栅格分辨率  grid_map/resolution

- **实验A：修改机体膨胀半径  (optimization/dist0=0.5  地图栅格分辨率  grid_map/resolution=0.1)**

<table>
  <tbody>
    <tr>
      <td>A1</td>
      <td>0.1</td>
    </tr>
    <tr>
      <td>A2</td>
      <td>0.2</td>
    </tr>
    <tr>
      <td>A3</td>
      <td>0.3</td>
    </tr>
    <tr>
      <td>A4</td>
      <td>0.4</td>
    </tr>
    <tr>
      <td>A5</td>
      <td>0.5</td>
    </tr>
  </tbody>
</table>

这里用森林地图，飞的都是同一个目标点

A1:成功

**[a2.mp4]**

A2成功

**[a3.mp4]**

A3成功

**[a4.mp4]**

A4失败，差一点

**[a4.mp4]**

A5失败，差的多一些

**[a5.mp4]**

根据上面五组测试能看出来，自身膨胀半径小的时候更容易穿越障碍且规划的路线更贴近障碍物，但机体、定位误差和控制误差可能导致实际碰撞；自身膨胀半径大的时候可能会规划失败，轨迹更加保守

总结：在穿越狭小空间时，机体膨胀半径  grid_map/obstacles_inflation应该设置较小的数值，因为还有距障碍物边缘安全距离  optimization/dist0存在，所以加起来的安全距离不能太大。

- **实验B：修改安全距离  ( grid_map/obstacles_inflation=0.2    grid_map/resolution=0.1)**

<table>
  <tbody>
    <tr>
      <td>B1</td>
      <td>0.2</td>
    </tr>
    <tr>
      <td>B2</td>
      <td>0.35</td>
    </tr>
    <tr>
      <td>B3</td>
      <td>0.5</td>
    </tr>
    <tr>
      <td>B4</td>
      <td>0.65</td>
    </tr>
    <tr>
      <td>B5</td>
      <td>0.8</td>
    </tr>
  </tbody>
</table>

还是用森林地图，目标点不变

B1 撞了

**[b1.mp4]**

B2 成功

**[b2.mp4]**

B3成功

**[b3.mp4]**

B4 未成功

**[b4.mp4]**

B5  未成功

**[b5.mp4]**

根据上面五组测试0.2时撞了，0.3-0.5规划成功，0.65和0.85没到规划点但也没有坠机，说明改optimization/dist0这个参数会使飞机规划路线随之变化，且对飞机保护有重要意义。过小的参数路径可能更短，但机体极易发生实际碰撞，参数增大后无人机距离障碍物更远碰撞风险降低，轨迹更加保守，但规划失败概率可能上升。

总结，这个参数与前面的自身膨胀半径共同组成机体中心距离障碍物外表面的距离，视具体机体大小而改变参数要同时考虑grid_map/obstacles_inflation和optimization/dist0。

- **实验C：修改栅格分辨率**

<table>
  <tbody>
    <tr>
      <td>C1</td>
      <td>0.05 </td>
    </tr>
    <tr>
      <td>C2</td>
      <td>0.10</td>
    </tr>
    <tr>
      <td>C3</td>
      <td>0.20</td>
    </tr>
    <tr>
      <td>C4</td>
      <td>0.40</td>
    </tr>
    <tr>
      <td>C5</td>
      <td>0.50</td>
    </tr>
  </tbody>
</table>

这里用房屋地图，下面是他的点云

![栅格分辨率实验 RViz 点云](docs/images/ego-planner/ego-17.png)

C1  非常卡，开久了就崩溃了，cpu占用率极高

![栅格分辨率实验资源占用一](docs/images/ego-planner/ego-18.png)

C2 正常，避障也没有什么问题，cpu占用率略比C3，4，5高一点，但几乎没区别

![栅格分辨率实验系统资源一](docs/images/ego-planner/ego-19.png)

C3 避障正常，cpu占用也不高

![栅格分辨率实验资源占用二](docs/images/ego-planner/ego-20.png)

C4，C5的cpu占用率几乎没区别，但在过狭窄通道时都有点问题

![栅格分辨率实验系统资源二](docs/images/ego-planner/ego-21.png)

![栅格分辨率实验资源占用三](docs/images/ego-planner/ego-22.png)

**[c5.mp4]**

根据上面五组测试

0.05 m用不了程序会卡死​，cpu占用率过高

0.10 m效果不错，通常是精度和计算量之间较合理的初始值；

0.20～0.25 m计算较快，cpu分配比较合理

0.4以上 cpu占用极小，但小障碍物和狭窄空间表达可能失真

总结这个参数不能小于0.1，也不能过大

#### **（4）不同环境测试**

- 房屋地图

**[ho.mp4]**

![小区房屋地图二维平面图](docs/images/ego-planner/ego-23.png)

- 随机障碍物地图

**[测试1.mp4]**

![随机障碍物地图二维平面图](docs/images/ego-planner/ego-24.png)

- 森林地图

**[for.mp4]**

![森林地图二维平面图](docs/images/ego-planner/ego-25.png)

- 隧道地图

没调出来，感觉要试一下斜着放雷达
