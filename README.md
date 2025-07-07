
### 1. 项目概述
当前项目架构：  
无人机使用Ardupilot飞控，通过串口连接到树莓派。 

控制部分：树莓派中运行控制代码，发布ros2消息，通过mavros桥接转换为mavlink协议通过串口与飞控通信实现实行控制。    

图像识别部分：树莓派读取树莓派摄像头的数据，发布ros2消息到以太网，地面站（你的计算机）接收ros2消息，通过yolo识别出对象的宽高与位置，再通过ros2发布由树莓派上的控制程序接收，转化为飞机的目标姿态。  
```
飞机{{飞控}-串口连接-{机载计算机(树莓派)}}-以太网（ros2）-地面站{图传}  
                |missionplanner    |-ros2-摄像头
```
仿真项目架构：  
飞控改为用ardupolot-sitl模拟，安装mavros桥接，控制代码接收与发布ros2消息，  
通过gazebo观察飞机状态，输出摄像头数据。  
```
地面站{mavlink{ardupolot-sitl}-mavros-ros2{控制代码}}-ros2{图传代码}  
                                                     | gazebo获得图像数据-ros2
```
总结：
- 项目概述
![项目概述](doc/项目概述.png)
- 仿真概述
![仿真概述](doc/仿真概述.png)



### 2. 安装ardupilot仿真（即ardupilot sitl 与 mavros以及gazebo）

docker
```
./build_docker.sh
./run_docker.sh gazebo
./run_docker.sh mavros
```

```sh
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
#更新子模块git submodule update --init --recursive
#重新加载路径（注销和登录以永久）：
. ~/.profile
./waf configure --board sitl
./waf copter
cd ardupilot/ArduCopter
sim_vehicle.py -w
sim_vehicle.py --console --map
# 查看具体报错
./sim_vehicle.py -v ArduCopter --console --map --moddebug 3
#export PATH="$HOME/ardupilot/Tools/autotest:$PATH"
# sim_vehicle.py -w -v ArduCopter
# sim_vehicle.py --vehicle=ArduCopter --console --map
# 外挂电脑硬件
##安装nvidia-container-toolkit
### 参考教程
https://blog.csdn.net/2404_89578344/article/details/146569579
### 安装依赖
sudo apt-get update
sudo apt-get install -y curl

### 添加密钥和仓库
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
### 再执行安装
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
### 更新Docker配置
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker（重启Docker服务）
# 挂载GPU启动Docker指令
docker run -it --gpus all \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  镜像名/ID
```

如果你使用wsl2，你可以尝试[通过 Mission Planner 的模拟功能运行SITL](https://ardupilot.org/planner/docs/mission-planner-simulation.html#mission-planner-simulation)

ardupilot官网：  
https://ardupilot.org/ardupilot/

安装missionplanner：（可选，用于真机校准，可以选择在windows安装）  
https://ardupilot.org/planner/docs/mission-planner-installation.html  


仿真部分官方教程：  

https://ardupilot.org/dev/docs/ros2.html  
https://ardupilot.org/dev/docs/ros2-sitl.html  
https://ardupilot.org/dev/docs/ros2-gazebo.html  

具体流程：  

构建APM源码：  
https://ardupilot.org/dev/docs/building-the-code.html  

SITL仿真：
https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

Gazebo仿真：  
https://ardupilot.org/dev/docs/sitl-with-gazebo.html  

安装与ros2版本对应的gazebo  
https://gazebosim.org/docs/ionic/install/
https://gazebosim.org/docs/fortress/ros2_gz_vendor_pkgs/  


### 3. 启动仿真
#### 使用 SITL
sim_vehicle.py
提供了一个启动脚本，用于自动为当前代码分支构建 SITL 固件版本、加载仿真模型、启动仿真器、设置环境和车辆参数，以及启动 MAVProxy GCS。可以指定许多脚本启动参数，键入此参数可查看完整列表：
`sim_vehicle.py --help`(先输入`export PATH=$PATH:$HOME/ardupilot/Tools/autotest`或将该命令写入~/.bashrc自动运行)
```
#用到的列表
  -v VEHICLE, --vehicle=VEHICLE
                        vehicle type (飞行器类型选择ArduCopter四旋翼)
  -f FRAME, --frame=FRAME
                        set vehicle frame type
                        ArduCopter: gazebo-
                            iris
                        （选择机架类型：gazebo-iris）
Simulation options:
    --model=MODEL       Override simulation model to use（新开终端，与gazebo通信）
Compatibility MAVProxy options (consider using --mavproxy-args instead):
  --map               load map module on startup（新开一个map窗口显示地图）
  --console           load console module on startup （新开一个console窗口显示飞机状态）
```
```sh
#标准启动命令
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
启动完后应新开三个窗口，如图所示,左下角为原来的窗口，ubuntu系统中新开的终端与原来的终端样式一致
![SITL](SITL.png)

#### gazebo与APM SITL综合使用
- 启动gazebo
    ```bash
    gz -h

    sim:           Run and manage the Gazebo Simulator.

    gz sim -h

    -r                           Run simulation on start.
    -v [ --verbose ] [arg]       Adjust the level of console output (0~4).控制台输出的输出等级
                                The default verbosity is 1, use -v without

                                arguments for level 3.

    ```
- 构建插件省略
- 配置 Gazebo 环境
    Gazebo 使用许多环境变量来查找插件和模型 在运行时。这些可以在用于运行 Gazebo 的终端中设置，也可以设置 在 .bashrc 或 .zshrc 文件中：

    - 在终端中
        ```sh
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
        export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
        ```
    - 在 .bashrc 或 .zshrc 中
        ```sh
        echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
        echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gaz
        ```
综合启动

Iris quadcopter
Run Gazebo
```sh
gz sim -v4 -r iris_runway.sdf
```
Run SITL
```sh
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
给飞控发送模式切换，解锁和起飞距离起飞点5米高度命令  
Arm and takeoff
```
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

完成起飞操作后的界面
![gz](gz.png)


### 1. 示例代码结构介绍 

`ros2_ws` 项目是一个基于 ROS 2 框架的无人机控制项目，主要用于仿真和实际控制无人机。项目结构和功能如下：

### 项目结构
px4_ros_com：无人机控制代码   
ros2_interfaces：定义图像识别所用的ros2消息类型
ros_gz_sim_ardupilot：gazebo地图与gazebo中摄像头配置  

\Chapter4_编写无人机控制代码\ros2_ws\ardupilot_ws\src\px4_ros_com\src\examples\offboard_control\OffboardControl.cpp  
- OffboardControl.cpp

：该文件包含了无人机的离线控制代码。
- PosControl.h 和 PosControl.cpp
：这些文件定义和实现了位置控制类 

    PosControl
，用于无人机的 PID 控制。

- PID.cpp 和 PID.h：这些文件定义和实现了 PID 控制器，用于无人机的姿态和位置控制。

#### 功能介绍
1. **仿真环境配置**：项目支持 APM-SITL 和 Gazebo 仿真环境，用户可以在仿真环境中测试和调整控制算法。
2. **离线控制**：通过 OffboardControl 类实现无人机的离线控制，支持位置、速度和姿态的控制。
3. **PID 控制**：项目实现了 PID 控制器，用于无人机的精确控制。用户可以通过调整 PID 参数来优化控制效果。
4. **自动调参**（未实现）：支持自动调参功能，通过 auto_tune 方法实现 PID 参数的自动调整。

#### 使用方法
1. **安装依赖**：确保已安装 ROS 2 和相关依赖库。
2. **构建项目**：在 `ros2_ws` 目录下运行 `colcon build` 命令构建项目。
3. **运行仿真**：使用 `ros2 launch` 命令启动仿真环境，例如 `ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py`。
4. **运行控制代码**：在仿真环境中运行控制代码，例如 `ros2 run px4_ros_com offboard_control`。

通过以上步骤，用户可以在仿真环境中测试和调整无人机的控制算法，并将其应用于实际无人机的控制。
将
### 安装依赖
- px4_ros_com
```sh
#!/bin/bash
#为树莓派从头开始安装依赖项 已安装ros2
#安装mavros-humble
#添加清华源
sudo apt install curl gnupg2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
#发行版Ubuntu 22.04 LTS (jammy)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt-get install ros-humble-mavros 
sudo apt-get install ros-humble-mavros-extras


#clone git仓库
cd ~/
mkdir ardupilot_ws
cd ardupilot_ws
#git clone https://github.com/dtyjno/drone.git
mv drone src # 更改为将附件中ardupilot_ws文件夹及其中内容移动到src文件夹下

#GeographicLib 用于gps定位
#解决[mavros_node-1]   what():  UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm | Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!报错
cd ~/Downloads
wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz
tar xfpz v2.3.tar.gz 
cd geographiclib-2.3 
#创建一个单独的构建目录并输入它，例如，
mkdir BUILD
cd BUILD 
#配置软件，指定源目录的路径，使用
#../configure 
#make
#make install
 cmake ..  
  make 
sudo make install

sudo apt-get install geographiclib-tools
sudo geographiclib-get-geoids egm96-5

# colcon无法找到  geographiclib-config.cmake
  sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
  
  
cd ~/ardupilot_ws
rosdep install --from-paths src --ignore-src -r -y

```
ros_gz_sim_ardupilot

Chapter3的仿真环境
https://ardupilot.org/dev/docs/sitl-with-gazebo.html

[安装ArduPilot Gazebo 插件](https://github.com/ArduPilot/ardupilot_gazebo)

ros2_interfaces

无

要编译软件包先执行以下命令
```bash
colcon build --packages-select ros2_interfaces
source install/setup.bash
```

### 程序脚本

以gazebo.sh为例，安装项目后：
```sh
cd ~/ros2_ws/ardupilot_ws;
source /opt/ros/jazzy/setup.bash;
source install/local_setup.bash; #找到ros_gz_sim_ardupilot
source ~/ros2_ws/ws/install/local_setup.bash; #找到ardupilot的gazebo插件
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}; #为gazebo找到插件添加全局变量
export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/ardupilot_ws/src/ros_gz_sim_ardupilot/models:$HOME/ros2_ws/ardupilot_ws/src/ros_gz_sim_ardupilot/worlds:${GZ_SIM_SYSTEM_PLUGIN_PATH}; #指定gazebo的模型路径和地图路径
ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py; #脚本启动gz_sim_server， gz_sim_gui,和gazebo消息到ros2消息的桥接
```
脚本示例
```bash
#!/bin/bash
gnome-terminal -t "gazebo" -x bash -c "
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH;
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH; 
#gz sim -v4 -r iris_runway.sdf;
#gz sim -v4 -r gimbal.sdf;

export GZ_SIM_RESOURCE_PATH=$HOME/code/ros2/ardupilot_ws/src/ros_gz_sim_ardupilot/models:$HOME/code/ros2/ardupilot_ws/src/ros_gz_sim_ardupilot/worlds:${GZ_SIM_SYSTEM_PLUGIN_PATH};
ros2 launch ros_gz_sim_ardupilot iris_runway.launch.py;
"

sleep 1s
gnome-terminal -t "SITL" -x bash -c "
#sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console;
"
sleep 10s
gnome-terminal -t "mavros" -x bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;
"
#gnome-terminal -t "mavproxy" -x bash -c "mavproxy.py --console --map --aircraft test --master=:14550"
```
运行script/中脚本以启动   
当前环境为ubuntu24.04+ros2jazzy，如有需要自行更改脚本以适配依赖
```sh
ardupilot.sh              #sitl
gazebo.sh                 #启动gazebo，发布图像
ros2_mavros.sh            #mavros
ros2_colcon.sh            #编译控制代码
ros2_offboard.sh          #启动控制代码
# 如安装图象识别代码（见Chapter2实践部分）
image_sub_gazebo_small.sh #接收图像，发布yolo识别后位置
```

### SITL
您可以通过使用 -L 参数和命名的 sim_vehicle.py 来使用特定位置的车辆启动模拟器 ardupilot/Tools/autotest/locations.txt 
```
#NAME=latitude,longitude,absolute-altitude,heading
HDU=30.317423,120.340137,0,178
```
您可以将自己的位置添加到文件中。顺序为 Lat、Lng、Alt、Heading，其中 alt 是 MSL 和米，heading 是度。

您可以将自己的私有位置添加到本地 locations.txt 文件中，其格式与主文件相同。在 linux 上，文件位于 $HOME/.config/ardupilot/locations.txt - 您需要使用您最喜欢的文本编辑器创建此文件。
```bash
sim_vehicle.py -D -v ArduCopter -L HDU -f gazebo-iris --model JSON --map --console --out 127.0.0.1:14550 --add-param-file=src/ros_gz_sim_ardupilot/config/gazebo-iris-gimbal.parm;
```

### 启动程序

先 cd 到项目目录

仿真环境
```
./ardupilot_mavros.sh

source install/setup.bash
ros2 run topic images_sub_gazebo
```
控制代码
```
source install/setup.bash
ros2 run px4_ros_com _offboard
#-----------或使用脚本
./run.sh
```
