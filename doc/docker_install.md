```bash
docker build -t dxy_apm_ws:latest -f Dockerfile/Dockerfile .
```

https://zhuanlan.zhihu.com/p/187505981

容器技术只隔离程序运行依赖的各种库以及配置但容器之间可以共享同一个操作系统。 容器更加的轻量级且占用的资源更少

docker中有这样几个概念：

dockerfile
image
container

实际上你可以简单的把image理解为可执行程序，container就是运行起来的进程。

那么写程序需要源代码，那么“写”image就需要dockerfile，dockerfile就是image的源代码，docker就是"编译器"。

实际上docker使用了常见的CS架构，也就是client-server模式，docker client负责处理用户输入的各种命令，比如docker build、docker run，真正工作的其实是server，也就是docker demon

- docker build

  当我们写完dockerfile交给docker“编译”时使用这个命令，那么client在接收到请求后转发给docker daemon，接着docker daemon根据dockerfile创建出“可执行程序”image。

- docker run
  有了“可执行程序”image后就可以运行程序了，接下来使用命令docker run，docker daemon接收到该命令后找到具体的image，然后加载到内存开始执行，image执行起来就是所谓的container。

3，docker pull
docker registry 可以用来存放各种image，公共的可以供任何人下载image的仓库就是docker Hub。docker pull命令从Docker Hub中下载image。

## 安装

Docker Engine-Community:

https://docs.docker.com/engine/install/debian/


设置 Docker 的 apt 存储库。

### Debian
#### Add Docker's official GPG key:
```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
#### Add the repository to Apt sources:
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
如果您使用衍生发行版，例如 Kali Linux，则可能需要替换此命令中预期打印版本代号的部分：


(. /etc/os-release && echo "$VERSION_CODENAME")
将这部分替换为对应 Debian 版本的代号，例如 bookworm。

#### 安装 Docker 软件包。

要安装最新版本，请运行：
```bash
 sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### ubuntu
执行以下命令卸载所有冲突的软件包。

```bash
 for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

使用 apt 存储库安装

在新主机上首次安装 Docker Engine 之前，您需要设置 Docker apt 存储库。之后，您可以从存储库安装和更新 Docker。

设置 Docker 的 apt 存储库。

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
```bash
# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
安装 Docker 软件包。

Latest   最近的 Specific version  特定版本
要安装最新版本，请运行：

```bash
 sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

## 换源

docker info


首先打开配置文件：

sudo nano /etc/docker/daemon.json
然后直接粘贴下列内容：

```json
{
    "registry-mirrors": [
   "https://docker.m.daocloud.io",
   "https://docker.imgdb.de",
   "https://docker-0.unsee.tech",
   "https://docker.hlmirror.com",
   "https://cjie.eu.org"
    ]
}
```

sudo systemctl daemon-reload 

 sudo systemctl restart docker

  sudo docker pull hello-world


要安装 docker 并设置正确的用户权限，请使用以下命令。

sudo apt install docker.io git python3-pip
pip3 install vcstool
echo export PATH=$HOME/.local/bin:$PATH >> ~/.bashrc
source ~/.bashrc
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
## 测试

通过运行 hello-world 映像来验证安装是否成功：
```bash
 sudo docker run hello-world
 ```
此命令将下载测试映像并在容器中运行它。当容器运行时，它会打印确认消息并退出。

## hw


Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (arm64v8)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/

#### run运行镜像
Docker 允许你在容器内运行应用程序， 使用 docker run 命令来在容器内运行一个应用程序。

 docker run ubuntu:15.10 /bin/echo "Hello world"

runoob@runoob:~$ docker run -t -i ubuntu:15.10 /bin/bash 
root@d77ccb2e5cca:/#
参数说明：

-i: 交互式操作。
-t: 终端。
ubuntu:15.10: 这是指用 ubuntu 15.10 版本镜像为基础来启动容器。
/bin/bash：放在镜像名后的是命令，这里我们希望有个交互式 Shell，因此用的是 /bin/bash。

 #### ps查看运行的容器 ps -a	列出所有容器

确认容器有在运行，可以通过 docker ps 来查看：

docker ps -a	列出所有容器（包括已停止的容器）

#### docker images列出本地镜像

使用 docker images 来列出本地主机上的镜像。

#### pull 获取一个新的镜像
当我们在本地主机上使用一个不存在的镜像时 Docker 就会自动下载这个镜像。如果我们想预先下载这个镜像，我们可以使用 docker pull 命令来下载它。

Crunoob@runoob:~$ docker pull ubuntu:13.10
 
#### docker search httpd 查找镜像
NAME: 镜像仓库源的名称

DESCRIPTION: 镜像的描述

OFFICIAL: 是否 docker 官方发布

stars: 类似 Github 里面的 star，表示点赞、喜欢的意思。

AUTOMATED: 自动构建。
#### commot创建镜像
当我们从 docker 镜像仓库中下载的镜像不能满足我们的需求时，我们可以通过以下两种方式对镜像进行更改。

1、从已经创建的容器中更新镜像，并且提交这个镜像
2、使用 Dockerfile 指令来创建一个新的镜像

通过命令 docker commit 来提交容器副本。

 ID 为 e218edb10161 的容器：

runoob@runoob:~$ docker commit -m="has update" -a="runoob" e218edb10161 runoob/ubuntu:v2

- -m: 提交的描述信息
- -a: 指定镜像作者
- e218edb10161：容器 ID
- runoob/ubuntu:v2: 指定要创建的目标镜像名

使用 docker images 命令来查看我们的新镜像 runoob/ubuntu:v2：
#### docker build构建镜像
我们使用命令 docker build， 从零开始来创建一个新的镜像。为此，我们需要创建一个 Dockerfile 文件，其中包含一组指令来告诉 Docker 如何构建我们的镜像。
```
FROM    centos:6.7
MAINTAINER      Fisher "fisher@sudops.com"

RUN     /bin/echo 'root:123456' |chpasswd
RUN     useradd runoob
RUN     /bin/echo 'runoob:123456' |chpasswd
RUN     /bin/echo -e "LANG=\"en_US.UTF-8\"" >/etc/default/local
EXPOSE  22
EXPOSE  80
CMD     /usr/sbin/sshd -D
```
每一个指令都会在镜像上创建一个新的层，每一个指令的前缀都必须是大写的。

第一条FROM，指定使用哪个镜像源

RUN 指令告诉docker 在镜像内执行命令，安装了什么。。。

然后，我们使用 Dockerfile 文件，通过 docker build 命令来构建一个镜像。

runoob@runoob:~$ docker build -t runoob/centos:6.7 .

参数说明：

-t ：指定要创建的目标镜像名

. ：Dockerfile 文件所在目录，可以指定Dockerfile 的绝对路径

docker run -t -i runoob/centos:6.7  /bin/bash
[root@41c28d18b5fb /]# id runoob
#### 删除docker rmi 
#### tag设置镜像标签
我们可以使用 docker tag 命令，为镜像添加一个新的标签。

runoob@runoob:~$ docker tag 860c279d2fec runoob/centos:dev
### 网络端口映射
docker run -d -P training/webapp python app.py

docker run -d -p 5000:5000 training/webapp python app.py

-P：是容器内部端口随机映射到主机的端口。
-p：是容器内部端口绑定到指定的主机端口。
## 容器使用
- 镜像（Image）：容器的静态模板，包含了应用程序运行所需的所有依赖和文件。镜像是不可变的。
- 容器（Container）：镜像的一个运行实例，具有自己的文件系统、进程、网络等，且是动态的。容器从镜像启动，并在运行时保持可变。
  Docker 客户端 : docker
可以通过命令 docker command --help 更深入的了解指定的 Docker 命令使用方法\

首先查看有哪些image以及有哪些container
```
sudo docker image ls
 
REPOSITORY                TAG                 IMAGE ID            CREATED             SIZE
currycode/tf_serving_vc   v1.0                26fdfe329859        2 months ago        3.85GB
currycode/tf_serving_vc   1.0                 d6957b0caf48        2 months ago        3.79GB
ubuntu                    18.04               93fd78260bd1        3 months ago        86.2MB
tensorflow/serving        latest              d42952c6f8a6        4 months ago        230MB
hello-world               latest              4ab4c602aa5e        6 months ago        1.84kB
```
```bash
sudo docker ps -a
 
# 结果
CONTAINER ID        IMAGE                         COMMAND                  CREATED             STATUS                      PORTS               NAMES
2effa7569ce3        currycode/tf_serving_vc:1.0   "/bin/bash"              2 months ago        Exited (0) 7 weeks ago                          tf_container_vc
a01e03520497        tensorflow/serving            "/usr/bin/tf_serving…"   2 months ago        Exited (137) 2 months ago                       determined_morse
ea05fb751b1a        hello-world                   "/hello"                 2 months ago        Exited (0) 2 months ago                         dazzling_kirch
```
创建container
```
docker run --name=tf_container_ASR -p 9001:9001 -p 9002:9002 -it currycode/tf_serving:v0330 
```
启动containner

如果你之前已经创建过container（比如上面docker ps 命令中显示有三个container），则可以直接启动container，不需要从image创建一个新的container了
```
docker start -i tf_container_vc
```

如果container此时运行在后台（没有使用-i参数），那么需要使用docker attach命令或者docker exec命令进入 他的终端

docker attach

[currycode@mjrc-server11 ~]$ sudo docker attach tf_container_vc
root@2effa7569ce3:/tensorflow-serving#
但是attach方法有个缺点，exit后container也跟着退出了

要想退出container时，让container仍然在后台运行着，可以使用“docker exec -it”命令。每次使用这个命令进入container，当退出container后，container仍然在后台运行，命令使用方法如下：

2.2.2 docker exec
[currycode@mjrc-server11 ~]$ sudo docker exec -it tf_container_vc /bin/bash

这样输入“exit”或者按键“Ctrl + C”退出container时，这个container仍然在后台运行：

## 在容器中安装新的程序
启动ubuntu image

安装命令apt-get install -y ping 

docker run learn/tutorial apt-get install -y ping
## 保存对容器的修改
用 docker ps -l命令获得安装完ping命令之后容器的id。然后把这个镜像保存为learn/ping

 docker commit 698 learn/ping
 docker commit -m "" -a "" container_id name:v1.0

 查看被修改的容器 ：docker ps -l
提交指定容器保存为新的镜像： docker commit \<container id\> \<new image name\>
查看本地所有镜像：docker images
## 复制文件
docker ps -a

docker cp 本地文件路径 容器ID/容器NAME:容器内路径

docker 容器名或容器id:容器内路径 本地
## 保存docker镜像到本地 从本地加载镜像
### 保存
docker images
REPOSITORY   TAG       IMAGE ID       CREATED      SIZE
redis        latest    ccee4cdf984f   8 days ago   105MB

docker save -o /mydocker/images/redis.tar redis镜像名称
### 加载
docker images

docker load -i /mydocker/images/redis.tar

docker images

## ros2

docker pull ros:jazzy-ros-core

docker run -it --rm ros:jazzy-ros-core

docker run --network=host -it your_ros2_image


 Debian/Ubuntu 系统
sudo apt-get install qemu qemu-user-static binfmt-support

 注册 ARM64 模拟器
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

  安装X11 让容器可以在宿主机显示图像及rviz               

#允许所有用户，包括docker,访问X11 的显示接口
sudo apt-get install x11-xserver-utils
xhost +
#输出为：access control disabled, clients can connect from any host
 之后就可以开始生成容器(container)咯！！

#添加选项
 --network host       \                       #可使开启的容器与宿主机共享IP和端口
-v /tmp/.X11-unix:/tmp/.X11-unix \           #共享本地unix端口
 -e DISPLAY=unix$DISPLAY \                    #修改环境变量DISPLAY
 -e GDK_SCALE \                               
 -e GDK_DPI_SCALE \