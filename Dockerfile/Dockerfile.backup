# TODO: Add SITL support

ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 设置Ubuntu镜像源 (noble for jazzy, jammy for humble)
RUN arch=$(dpkg --print-architecture) && \
    if [ "${ROS_DISTRO}" = "humble" ]; then \
      UBUNTU_VERSION="jammy"; \
    elif [ "${ROS_DISTRO}" = "jazzy" ]; then \
      UBUNTU_VERSION="noble"; \
    else \
      UBUNTU_VERSION="jammy"; \
    fi && \
    if [ "$arch" = "amd64" ] || [ "$arch" = "i386" ]; then \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_VERSION} main restricted universe multiverse" > /etc/apt/sources.list && \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_VERSION}-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_VERSION}-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ ${UBUNTU_VERSION}-security main restricted universe multiverse" >> /etc/apt/sources.list ; \
    else \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ ${UBUNTU_VERSION} main restricted universe multiverse" > /etc/apt/sources.list && \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ ${UBUNTU_VERSION}-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
      echo "deb [arch=$arch] https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ ${UBUNTU_VERSION}-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
      echo "deb [arch=$arch] http://ports.ubuntu.com/ubuntu-ports/ ${UBUNTU_VERSION}-security main restricted universe multiverse" >> /etc/apt/sources.list ; \
    fi && \
    rm -f /etc/apt/sources.list.d/* && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg2 && \
    rm -f /etc/apt/sources.list.d/ros2.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    chmod a+r /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu ${UBUNTU_VERSION} main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get upgrade -y

# 安装必要的依赖包（包括图形界面和仿真环境）
RUN apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libncurses5-dev \
    rapidjson-dev \ 
    # lsb-release \
    # ROS2 packages
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    # 图形界面和仿真支持
    x11-apps \
    mesa-utils \
    # libgl1-mesa-glx \
    libgl1-mesa-dri \
    xvfb \
    gnome-terminal \
    dbus-x11 \
    # ArduPilot SITL 依赖
    # python3-matplotlib \
    # python3-serial \
    # python3-wxgtk4.0 \
    # python3-lxml \
    # python3-pip \
    # python3-setuptools \
    # python3-dev \
    # 地理库依赖
    geographiclib-tools \
    # libgeographic-dev \
    # 其他有用工具
    vim \
    nano \
    htop \
    && \
    # 安装GeographicLib数据集
    geographiclib-get-geoids egm96-5 && \
    : "remove cache" && \
    # Gazebo支持
    
    if [ "${ROS_DISTRO}" = "humble" ]; then \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz; \
      sudo apt-get install -y  curl lsb-release gnupg

        # https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu

        sudo apt-get update
        sudo apt-get install curl lsb-release gnupg
        sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
        sudo apt-get update
        sudo apt-get install gz-harmonic
        apt-get install ros-humble-ros-gzharmonic



    else if [ "${ROS_DISTRO}" = "jazzy" ]; then \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz; \
    fi && \
    apt-get autoremove -y -qq && \
    # 清理APT缓存
    rm -rf /var/lib/apt/lists/*

# 创建用户
ARG USERNAME=dxy

RUN groupadd $USERNAME \
    && useradd --gid $USERNAME -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# 复制项目源码
COPY . /home/dxy_apm_ws/
RUN chown -R $USERNAME:$USERNAME /home/dxy_apm_ws/

# 切换到普通用户
USER $USERNAME

# # 安装ArduPilot SITL
# RUN cd /home && \
#     git clone https://github.com/ArduPilot/ardupilot.git && \
#     cd ardupilot && \
#     git submodule update --init --recursive && \
#     Tools/environment_install/install-prereqs-ubuntu.sh -y && \
#     echo 'export PATH=$PATH:/home/ardupilot/Tools/autotest' >> ~/.bashrc && \
#     echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc

# 设置rosdep
RUN cd /home/dxy_apm_ws/ \
    && sudo mkdir -p /etc/ros/rosdep/sources.list.d/  \
    && sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list \
    && export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && if [ "${ROS_DISTRO}" = "humble" ]; then \
        cd /home/dxy_apm_ws/src/ardupilot_gazebo && \
        git checkout fortress; \
      else \
        cd /home/dxy_apm_ws/src/ardupilot_gazebo && \
        git checkout ros2; \
      fi

# 清理之前的构建缓存和安装项目依赖
RUN cd /home/dxy_apm_ws/ \
    && rm -rf build/ install/ log/ \
    && sudo apt-get update \
    && rosdep install --from-paths src -i -y --rosdistro ${ROS_DISTRO} --skip-keys="gz-sim8 ignition-gazebo6 gz-cmake3" \
    && sudo apt-get autoremove -y -qq \
    && sudo rm -rf /var/lib/apt/lists/*

# 构建项目
RUN cd /home/dxy_apm_ws/ \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --packages-select ros2_interfaces ros2_yolo_msgs" \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source install/setup.bash \
    && colcon build --cmake-args -DBUILD_TESTING=OFF --symlink-install --packages-skip topic topic2"

# # 设置环境变量到bashrc
# RUN echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc \
#   && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
#   && echo "source /home/dxy_apm_ws/install/setup.bash" >> ~/.bashrc \
#   && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
#   && echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/dxy_apm_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc \
#   && echo "export GZ_SIM_RESOURCE_PATH=/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/models:/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/worlds:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc

# 设置工作目录
WORKDIR /home/dxy_apm_ws

# 设置环境变量
ENV ROS_WORKSPACE=/home/dxy_apm_ws
ENV DISPLAY=:0
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/dxy_apm_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo
ENV GZ_SIM_RESOURCE_PATH=/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/models:/home/dxy_apm_ws/src/ros_gz_sim_ardupilot/worlds

COPY Dockerfile/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]

