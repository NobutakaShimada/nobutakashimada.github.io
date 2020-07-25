---
layout: splash
lang: en
ref: phyexp3\_basic
permalink: /docs/basic/
sidebar:
  title: 基本編(Turtlebot3の操作とROSの基礎)
  nav: "phyexp3\_basic"
---

# [Class 1] 基本編(Turtlebot3の操作とROSの基礎)

この実験はTurtlebot3+OpenManipulator-Xというロボットプラットフォームを用いて、
広くロボットの制御に用いられるROS(Robot Operating System)の分散ノード
アーキテクチャを学ぶとともに、シミュレータ(Gazebo)と実機による距離センサや
ロボットの内界センサの計測値の取得と可視化の仕方や、それらを応用した環境地図の
生成と誘導(Simultaneous Localization And Mapping)を体験することが目的です。


## 使用ロボット
実験で用いるロボットはROBOTIS社製のTurtlebot3です。

![Turtlebot3+OpenManipulator-X](https://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/hardware_setup.png)

### 起動手順

1. アームが図のようなホームポジションになっていることを確認する。
2. バッテリーが接続され、本体前面のスロットに正しく装着されているか確認する。
3. 本体上面の距離センサに干渉する物がないことを確認する
4. 平らで開けた場所に置いて、本体前面のスイッチを入れる
5. アームがホームポジションに移動し、距離センサが回転し始める。

### 停止手順

1. 走行している場合はコントローラや"teleop" ROSノードからメッセージを送り
停止させる。方法がない時は本体を両手でピックアップする（アームを持たないこと）。
2. 電源が落ちるとアームが脱力するので支えられるように準備する。
3. 本体前面のスイッチを切る。

### バッテリーについて

バッテリーの有効時間は待機状態で90分程度、走行させたりアームを稼働させると
稼働時間が短くなります。長く使わない時はスイッチを切るようにします。



## 主なROS コマンド

ROSコマンドはターミナル（端末）上のシェルからコマンド入力によって操作することが基本です。Gazebo、RVizのようにアプリとしてウィンドウが起動するものもあります。

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
|catkin\_create\_pkg|自動的にROSパッケージと関連ファイルを生成します。|
|catkin\_make|Catkinビルドシステムでパッケージをビルドします。|
|rospack|明示されたROSパッケージの情報を確認します。|


## ROSの起動とキー入力を用いたロボットの操作

[Remote PC] とある部分は自分のLinux端末で実行すること。[]
oscoreはTurtlebot3 PCで実行しないでください。  
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

Turtlebot3上でROSノードを起動するには、１）SSHでTurtlebot3にログインしてノードを実行する方法と、２）Host PC上でlaunchファイル内からmachineタグを使って起動する方法、の２つがあります。
ここでは、ROSの仕組みを確認するために１）のSSHログインして実行する方法を試します。

[Remote PC] SSHでログインします。例えば「01」番のロボットにログインするには以下のように２桁の番号を引数に与えます。パスワードは必要ありません（公開鍵でログインします）。
```bash
$ ssh 01
```

[Turtlebot3 SBC] ログインができたら以下のコマンドを入力し、Turtlebot3を起動してください。 

```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Turtlebot3のモデルがBurgerの場合、ターミナルに以下のメッセージが表示されます（実験で使用するのはwaffle\_pi）。 

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

ノードが起動したかどうかは、rostopicコマンドを使って確かめられます。rostopic listコマンドは起動しているノード同士が通信するトピック（宛先のようなもの）の一覧を表示します。

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
turtlebot3_robot.launchは、turtlebot3の状態、オドメトリーの情報を出力するノードや、速度指令を受け取って並進や旋回を行うノード、さらに距離センサーの測定値を出力するノードを起動します。

{% capture capture03 %}
 [発展] turtlebot3\_robot.launchの詳細
**roslaunch turtlebot3\_bringup turtlebot3\_robot.launch**
1. turtlebot3_core.launch
- subscribe : cmd_vel
    - publish : joint_states, odom

2. turtlebot3_lidar.launch
    - publish : scan

turtlebot3_robot.launchファイルを実行すると、turtlebot3_core.launchとturtlebot3_lidar.launchファイルが実行され、TurtleBot3の状態をチェックするノード(node)であるturtlebot3_diagnosticsが生成され、TurtleBot3の各種センサやハードウェアの状態についての情報をpublishします。turtlebot3_core.launchファイルでは、OpenCRと通信してjoint_states、odomをpublishし、cmd_velをsubscribeするノードが生成されます。turtlebot3_lidar.launchファイルでは、LIDARを作動させ、センサーから得られたscanデータをpublishするノード(node)が生成されます。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

### rqt_graphコマンドによるROSノード・トピックの可視化

[remote PC] Host PC側の端末でrqt_graphコマンドを実行すると起動しているROSノードとトピックの様子が図示されたウインドウが開きます。
```bash
$ rqt_graph
```

![](/assets/images/ritsumeikan/009.png)

### RvizによるTurtlebot3の状態の可視化

[Remote PC] robot_state_publisherとRVizを実行してください。  
```bash
$ roslaunch exp3 remote.launch
```

{% capture capture04 %}
**roslaunch exp3 remote.launch**
1. remote.launch
    - urdf：Unified Robot Description Formatの略で、ロボットの構成や接続形態を表すXML形式のファイルです。
    - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
    - subscribe : joint_states 
    - publish : tf

remote.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
あとでSLAMを実行する時に使うslam.launchファイル内部にはremote.launchが含まれているので、slam.launchが実行されると自動的にremote.launchが最初に実行されます｡
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](/assets/images/ritsumeikan/010.png)

新しいターミナルウィンドウを一つ開いて、下記のコマンドを入力してください。 
```bash
$ roslaunch exp3 rviz.launch
```

{% capture capture05 %}
 [発展] rviz.launchの詳細
**roslaunch exp3 rviz.launch**
- subscribe : tf, scan

rvizを実行すると、tf(transform:座標変換)とscanデータをそれぞれロボットの姿勢と周辺の障害物の情報として視覚化します。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)


## キー入力によるTurtlebot3の遠隔操作

### シンプルな遠隔操作のノード

[Remote PC] Remote PCでteleop_keyを起動します。
```bash
$ roslaunch exp3 teleop.launch
```

{% capture capture06 %}
[発展] teleop.launchによる遠隔操作
**roslaunch exp3 teleop.launch**
- publish : cmd_vel

teleop.launchファイルを実行して生成されたturtlebot3_teleop_keyboardノードでは、キーボードの入力を読み取って平行移動速度linearと旋回角速度angular値を更新し、linearとangularが含まれたtwist形式のtopicであるcmd_velをpublishします。  
その後、Turtlebot3のSBCで実行されたturtlebot3_robot.launchに含まれたturtlebot3_core.launchでcmd_velを受信します。  
受信されたコマンドに従って車輪と接続されたモータが駆動し、ロボットを動かします。
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




