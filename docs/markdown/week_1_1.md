---
layout: splash
lang: en
ref: phyexp3_1-1
permalink: /docs/week1-1/
sidebar:
  title: １コマ目
  nav: "phyexp3_1-1"
---


# [Class 1] Learning ROS Environment

{% capture staff01 %}
**赤ボックス領域は助教のための追加内容です｡**

## ROS Installation and environment configuration

参考資料 : [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

ROS KineticバージョンはUbuntu 15.10、Ubuntu 16.04、Debian 8を基盤として動作するものの、Ubuntu 16.04の使用をお勧めします。

### sources.list の設定
次のコマンドを入力すると、インストールするROSの最新バージョンパッケージを入手できます。
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### キー(key) の設定
Ubuntuサーバーに接続するためのキーを設定します。
```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### ROS のインストール
Ubuntuのパッケージインデックスを最新のものに更新します。
```bash
$ sudo apt-get update
```

必要なROSパッケージをそれぞれ個別にインストールできますが、一般的には、以下のようにdesktop-fullパッケージのインストールをお勧めします。

```bash
sudo apt-get install ros-kinetic-desktop-full
```

### rosdepの初期化
ROSを使用する前に、rosdepを初期化する必要があります。rosdepは、ROSで作成されたコードをコンパイルしたり、実行時に必要な関連パッケージ(公式的にはdependencyと呼びます)のインストールをサポートします。
```bash
$ sudo rosdep init
$ rosdep update
```

### ROSの環境設定
以下のように、ROSに関する設定が保存された環境ファイルが、shellを実行するたびに自動的に実行されるように.bashrcファイルに追加します。
```bash
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

以下は、現在開かれているshellで`.bashrc`ファイルを再読み込みするコマンドです。
```bash
$ source ~/.bashrc
```

### パッケージビルドに必要なDependency のインストール
追加で以下のパッケージをインストールすると、ROSの作業空間であるworkspaceを作成・管理したり、ROSパッケージと関連する他のパッケージのダウンロードに役立ちます。
```bash
$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

## 主なROS コマンド

参考資料 : [https://w3.cs.jmu.edu/spragunr/CS354_S19/handouts/ROSCheatsheet.pdf](https://w3.cs.jmu.edu/spragunr/CS354_S19/handouts/ROSCheatsheet.pdf)

|コマンド|説明|
|:---|:---|
|roscore|単一コマンドとして使用され、ROS MasterとROSの実行に必要な各種サーバーを実行します。|
|rosrun|明示されたnodeを実行します。|
|roslaunch|ファイルに明示された複数のnodeをオプションと共に実行します。|
|rosclean|ROSのlogファイルを確認または削除します。|
|roscd|明示されたROSパッケージが保存されているディレクトリに移動します。|
|rostopic|ROSのtopic情報を確認します。|
|rosservice|ROSのservice情報を確認します。|
|rosnode|ROSのnode情報を確認します。|
|rosparam|ROSのパラメータ (parameter) 情報を確認します。|
|rosbag|ROS内部のメッセージを記録または再生します。|
|rosmsg|ROSのmessageデータ構造を示します。|
|rossrv|ROSのserviceデータ構造を示します|
|catkin_create_pkg|自動的にROSパッケージと関連ファイルを生成します。|
|catkin_make|Catkinビルドシステムでパッケージをビルドします。|
|rospack|明示されたROSパッケージの情報を確認します。|


# TurtleBot3 Operation

## ネットワーク設定

### PCネットワーク設定

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration.png)

TurtleBot3の場合、ROS MasterがRemote PCで駆動します。  
ROS基盤のTurtleBot3とRemote PCが互いに通信を行うためには、IPアドレスが設定されている必要があります。  
このとき、Remote PCとTurtleBot3のPC(またはSBC)は同じルータに接続され、同じワイヤレスネットワークに接続される必要があります。

ネットワークに接続後、以下のコマンドをRemote PCのターミナルウィンドウに入力して、IPアドレスを検索します。

```bash
$ ifconfig
```

赤枠部分のIPアドレスが、Remote PCのIPアドレスです。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration2.png)

以下のコマンドをターミナルウィンドウに入力して、環境設定ファイルを修正します。
```bash
$ nano ~/.bashrc
```

`ALT` + `/`ショートカットを押すと、ファイルの末尾に移動します。  
以下のように、Remote PCのIPアドレスをROS_MASTER_URIとROS_HOSTNAME項目に入力します。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration3.png)

`CTRL` + `X`ショートカットで修正を終了すると、保存するかどうかを確認する画面が出ます。  
`Y`と`Enter`を順番に押し、同一のファイルに上書きして終了します。  
次に、以下のコマンドを使用してbashrcに反映します。

```bash
$ source ~/.bashrc
```

## TurtleBot3 SBC Network Setup

TurtleBotのIPアドレスを見つけるためにTurtleBot SBCのターミナルウィンドウでコマンドの下に入力します。

```bash
$ ifconfig
```

赤枠部分のIPアドレスが、TurtleBot3 SBCのIPアドレスです。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration4.png)

以下のコマンドをターミナルウィンドウに入力して、環境設定ファイルを修正します。
```bash
$ nano ~/.bashrc
```

`ALT` + `/`ショートカットを押すと、ファイルの末尾に移動します。
以下のように、Remote PCのIPアドレスをROS_MASTER_URIとROS_HOSTNAME項目に入力します。

ROS_MASTER_URIのアドレスをlocalhostからRemote PCのアドレスに修正してください｡
その後、ROS_HOSTNAMEのlocalhostをTurtleBot3 SBCのIPアドレスに修正してください｡

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration5.png)

`CTRL` + `X`ショートカットで修正を終了すると、保存するかどうかを確認する画面が出ます。  
`Y`と`Enter`を順番に押し、同一のファイルに上書きして終了します。  
次に、以下のコマンドを使用してbashrcにを調達。

```bash
$ source ~/.bashrc
```

## Bringup

当内容はRemote PCで実行してください。roscoreはTurtlebot3 PCで実行しないでください。  
各装置(Turtlebot3 PC, Remote PC)のIPアドレスが正しく設定されているかを確認してください。
バッテリーの電圧が１１Vより低いと、アラームが鳴り続け、作動装置が非活性化されます。アラームが鳴った場合、バッテリーを充電しなければなりません。 

### roscoreの実行
[Remote PC] roscoreを実行してください。 
```bash
$ roscore
```

```bash
$ rostopic list
/rosout
/rosout_agg
```

### Turtlebot3のBringup 

次の設置方法はROS 1 Kineticでのみ使用できます。

{% capture kinetic-setting %}
**注意**
- Remote PCで ROS 1 Kinetic Kameバージョンを使ってTurtleBot3を駆動する場合、以下のコマンドをTurtleBot3のSBCで実行してください。コマンドを実行するとTurtleBot3パッケージの内容が`kinetic-devel`ブランチにアップデートされます。この作業を行うためにはインターネットに繋がっている必要があります。  
**$ cd ~/catkin_ws/src && rm -rf turtlebot3**  
**$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git**  
**$ cd ~/catkin_ws/src/turtlebot3**  
**$ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/**  
**$ cd ~/catkin_ws && catkin_make -j1**
**$ source ~/.bashrc**

- gitが設置されてなくてエラーが発生する場合、以下のコマンドでgitを設置してください。  
**$ sudo apt install git** 
{% endcapture %}
<div class="notice--warning">{{ kinetic-setting | markdownify }}</div>

[Turtlebot3 SBC] 以下のコマンドを入力し、Turtlebot3を起動してください。 
以下のコマンドを実行する際発生するソフトウェアバージョンワーニングが実行に問題を起こすわけではありません。

```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Turtlebot3のモデルがBurgerの場合、ターミナルに以下のメッセージが表示されます。 

```
SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.13
 * /turtlebot3_core/baud: 115200
 * /turtlebot3_core/port: /dev/ttyACM0
 * /turtlebot3_core/tf_prefix: 
 * /turtlebot3_lds/frame_id: base_scan
 * /turtlebot3_lds/port: /dev/ttyUSB0

NODES
  /
    turtlebot3_core (rosserial_python/serial_node.py)
    turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
    turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS_MASTER_URI=http://192.168.1.2:11311

process[turtlebot3_core-1]: started with pid [14198]
process[turtlebot3_lds-2]: started with pid [14199]
process[turtlebot3_diagnostics-3]: started with pid [14200]
[INFO] [1531306690.947198]: ROS Serial Python Node
[INFO] [1531306691.000143]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1531306693.522019]: Note: publish buffer size is 1024 bytes
[INFO] [1531306693.525615]: Setup publisher on sensor_state [turtlebot3_msgs/SensorState]
[INFO] [1531306693.544159]: Setup publisher on version_info [turtlebot3_msgs/VersionInfo]
[INFO] [1531306693.620722]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1531306693.642319]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1531306693.687786]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1531306693.706260]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1531306693.722754]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1531306693.759059]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1531306695.979057]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1531306696.007135]: Note: subscribe buffer size is 1024 bytes
[INFO] [1531306696.009083]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1531306696.040047]: Setup subscriber on sound [turtlebot3_msgs/Sound]
[INFO] [1531306696.069571]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1531306696.096364]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1531306696.390979]: Setup TF on Odometry [odom]
[INFO] [1531306696.394314]: Setup TF on IMU [imu_link]
[INFO] [1531306696.397498]: Setup TF on MagneticField [mag_link]
[INFO] [1531306696.400537]: Setup TF on JointState [base_link]
[INFO] [1531306696.407813]: --------------------------
[INFO] [1531306696.411412]: Connected to OpenCR board!
[INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
[INFO] [1531306696.418398]: --------------------------
[INFO] [1531306696.421749]: Start Calibration of Gyro
[INFO] [1531306698.953226]: Calibration End
```

同期化失敗メッセージがターミナルウィンドウに表示された場合、Turtlebot3のセンサー装置がきちんと繋がれているかを確認してください。 


```bash
$ rostopic list -v

Published topics:
 * /rpms [std_msgs/UInt16] 1 publisher
 * /version_info [turtlebot3_msgs/VersionInfo] 1 publisher
 * /battery_state [sensor_msgs/BatteryState] 1 publisher
 * /joint_states [sensor_msgs/JointState] 1 publisher
 * /rosout [rosgraph_msgs/Log] 3 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /cmd_vel_rc100 [geometry_msgs/Twist] 1 publisher
 * /firmware_version [turtlebot3_msgs/VersionInfo] 1 publisher
 * /imu [sensor_msgs/Imu] 1 publisher
 * /odom [nav_msgs/Odometry] 1 publisher
 * /scan [sensor_msgs/LaserScan] 1 publisher
 * /diagnostics [diagnostic_msgs/DiagnosticArray] 2 publishers
 * /tf [tf/tfMessage] 1 publisher
 * /sensor_state [turtlebot3_msgs/SensorState] 1 publisher
 * /magnetic_field [sensor_msgs/MagneticField] 1 publisher

Subscribed topics:
 * /firmware_version [turtlebot3_msgs/VersionInfo] 1 subscriber
 * /motor_power [std_msgs/Bool] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /sound [turtlebot3_msgs/Sound] 1 subscriber
 * /reset [std_msgs/Empty] 1 subscriber
 * /imu [sensor_msgs/Imu] 1 subscriber
 * /scan [sensor_msgs/LaserScan] 1 subscriber
 * /cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /sensor_state [turtlebot3_msgs/SensorState] 1 subscriber

```

{% capture capture03 %}
**roslaunch turtlebot3_bringup turtlebot3_robot.launch**
1. turtlebot3_core.launch
    - subscribe : cmd_vel
    - publish : joint_states, odom

2. turtlebot3_lidar.launch
    - publish : scan

turtlebot3_robot.launchファイルを実行すると、turtlebot3_core.launchとturtlebot3_lidar.launchファイルが実行され、TurtleBot3の状態をチェックするノード(node)であるturtlebot3_diagnosticsが生成され、TurtleBot3の各種センサやハードウェアの状態についての情報をpublishします。turtlebot3_core.launchファイルでは、OpenCRと通信してjoint_states、odomをpublishし、cmd_velをsubscribeするノードが生成されます。turtlebot3_lidar.launchファイルでは、LIDARを作動させ、センサーから得られたscanデータをpublishするノード(node)が生成されます。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

```bash
$ rqt_graph
```

![](/assets/images/ritsumeikan/009.png)

### RvizにTurtlebot3をLoad 
[Remote PC] robot_state_publisherとRVizを実行してください。  
当コマンドを行う前にTurtlebot3のモデル名を指定しなければなりません。$ {TB3_MODEL}は、burger、waffle、waffle_piの中で使用するモデル名を指定してください。exportの設定を永続化するためには、Export Turtlebot3_MODELのページを参照してください。 
```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

{% capture capture04 %}
**roslaunch turtlebot3_bringup turtlebot3_remote.launch**
1. turtlebot3_remote.launch
    - urdf：Unified Robot Description Formatの略で、ロボットの構成や接続形態を表すXML形式のファイルです。
    - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
    - subscribe : joint_states 
    - publish : tf

turtlebot3_remote.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
turtlebot3_slam.launchファイル内部にturtlebot3_remote.launchが含まれているのでturtlebot3_slam.launchが実行されると自動的にturtlebot3_remote.launchが最初に実行されます｡
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](/assets/images/ritsumeikan/010.png)

新しいターミナルウィンドウを一つ開いて、下記のコマンドを入力してください。 
```bash
$ rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
```

{% capture capture05 %}
**rosrun rviz rviz -d \`rospack find turtlebot3_description\`/rviz/model.rviz**
- subscribe : tf, scan

rvizを実行すると、tfとscanデータをそれぞれロボットの姿勢と周辺の障害物の情報として視覚化します。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)


## Keyboard Teleoperation
TurtleBot3は、様々なデバイスで遠隔操作することができます。(Leap Motionは対象外)ここで示した実施例は、DYNAMIXEL,Raspberry pi3,OpenCR1.0,Ubuntu Mate16.04(ROS Kinetic)の構成で起動可能なPS3、XBOX 360、ROBOTIS RC100等の無線デバイスでテストされています。

### シンプルな遠隔操作のノード
[Remote PC] Remote PCでturtlebot3_teleop_keyを起動します。
```bash
$ export TURTLEBOT3_MODEL=%{TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

{% capture capture06 %}
**roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch**
- publish : cmd_vel

turtlebot3_teleop_key.launchファイルを実行して生成されたturtlebot3_teleop_keyboardノードでは、キーボードの入力を読み取ってlinearとangular値を更新し、linearとangularが含まれたtwist形式のtopicであるcmd_velをpublishします。  
その後、Turtlebot3のSBCで実行されたturtlebot3_robot.launchに含まれたturtlebot3_core.launchでcmd_velを受信します。  
cmd_velトピックは、rosserialを介してOpenCRに伝達され、OpenCRにアップロードされたファームウェアでDYNAMIXELを制御するためのコマンドとして出力されます。  
受信されたコマンドに従って車輪と接続されたDYNAMIXELが駆動し、ロボットを動かします。
{% endcapture %}
<div class="notice--success">{{ capture06 | markdownify }}</div>


ノードが正常に起動されている場合は、次の命令は、ターミナルウィンドウに現れます。

```
Control Your Turtlebot3!
---------------------------
Moving around:
        w
    a   s   d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```
