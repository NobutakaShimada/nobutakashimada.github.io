---
layout: splash
lang: en
ref: phyexp3_3-2
permalink: /docs/week3-2/
sidebar:
  title: ６コマ目
  nav: "phyexp3_3-2"
---

# [Class 6] TurtleBot3 Manipulatorを使用してSLAMを実行する
OpenMANIPULATOR-Xを組み付けたTurtleBot3のSLAMは、以前学習したSLAMと少々差異があります。
ロボットアームがLDSセンサーの一定部分を塞いでいるため、SLAMに使用されるLDSセンサーの範囲を制限することによってスムーズな地図作成が可能となります。
以下の通り`scan_data_filter.yaml`ファイルにおいてLDSセンサーの設定された角度範囲データをフィルタリングすることによって、有効でない角度値を使用せずに地図を描くことができます。

```
scan_filter_chain:

- name: Remove 120 to 240 degree
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: 2.0944
    upper_angle: 4.18879
```

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_slam.png)

## roscoreを実行する
[Remote PC] roscoreをユーザのPCで動作させます。
```bash
$ roscore
```

## Bringupを実行する
[TurtleBot3 SBC] 以下のコマンドによって、rosserialとLDSセンサを動作させるノードを実行します。
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
{% capture capture01 %}
**roslaunch turtlebot3_bringup turtlebot3_robot.launch**
1. **turtlebot3_core.launch**
    - subscribe : cmd_vel
    - publish : joint_states, odom

2. **turtlebot3_lidar.launch**
    - publish : scan

OpenCRのファームウェアが変更されたため、turtlebot3_robot.launchファイルを実行すると、Week1で説明したturtlebot3_robot.launchを実行させたときにpublishされるトピックに加えて、joint_trajectory_point、gripper_positionのトピック2種をsubscribeします。joint_trajectory_pointはOpenMANIPULATORの各関節の位置値を伝達し、OpenCRを通じてOpenMANIPULATORを構成するDYNAMIXELアクチュエータに伝達されます。gripper_positionはOpenMANIPULATORグリッパの位置値であり、joint_trajectory_pointと同様に、OpenCRを通じてグリッパーを構成するDYNAMIXELに伝達され、グリッパを制御します。rqtのMessage Publisherを使用して位置値を転送する方法によっても簡単に制御することができます。
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>

SLAMを使用して地図を作成する場合は、Manipulationを使用していないため、以下のOpenMANIPULATORを制御するコントローラーとmove_groupインタフェースは実行する必要はありません。

```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

## SLAMノードを実行する
[Remote PC]ここでは、Gmappingを活用したSLAMを実行します。

```bash
$ roslaunch turtlebot3_manipulation_slam slam.launch
```
{% capture capture06 %}
**roslaunch turtlebot3_manipulation_slam slam.launch**
1. **urdf**
  - Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
2. **robot_state_publisher**
  - robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
  - subscribe : joint_states 
  - publish : tf
3. **laser_filterノード**
  - LDSセンサの有効ではない値の範囲をフィルタリングするノードを実行します。ここでは、OpenMANIPULATORが設置されている後方部の角度を無視します。
4. **turtlebot3_gmapping.launch**
  - Gmappingを利用したSLAMを実行するために必要なパラメータ情報がパラメータサーバにロードされます。この設定を利用してgmappingを設定します。
5. **turtlebot3_gmapping.rviz**
  - Gmappingを適用したSLAMをRviz画面に表示するために必要なRvizのデフォルト設定を適用し、Rvizを実行します。

turtlebot3_manipulation_slam.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
{% endcapture %}
<div class="notice--success">{{ capture06 | markdownify }}</div>

## turtlebot3_teleop_keyノードを実行する
[Remote PC] 作成されていない地図上の位置にロボットを移動させ、地図を完成させます。
```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

[Remote PC] 地図が完成した後、map_saverノードを実行して地図ファイルを保存します。
```bash
$ rosrun map_server map_saver -f ~/${map_name}
```
<-f>オプションは、地図ファイルを保存する場所とファイル名を指定します。上のコマンドでは、～/${map_name}オプションが使用されているため、ユーザーのhomeフォルダ(～/または/home/<username>)に ${map_name}.pgmと${map_name}.yamlファイルとして保存されます。${map_name}に保存したいファイル名を入力してください。

# Navigation
OpenMANIPULATORを組み付けたTurtleBot3のNavigationは、基本TurtleBot3のプラットフォームで実行するNavigationと大きな差異はありません。ただし、SLAMと同様にLDSセンサの範囲を指定しておくことが望ましく、Navigationの途中で必要な場合、OpenMANIPULATORを駆動するためにロボットアームとグリッパーを制御する関連ノードを実行することができます。

## roscoreを実行する
[Remote PC] roscoreをユーザーのPCで動作させます。
```bash
$ roscore
```

## Bringupを実行する
[TurtleBot3 SBC] 以下のコマンドによって、rosserialとLDSセンサを動作させるノードを実行します。
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
<div class="notice--success">{{ capture01 | markdownify }}</div>

## Navigationを実行する
[Remote PC] 以下のコマンドを実行すると、Navigationの実行に必要な様々なパラメータと地図、GUI環境を作るためのURDFやRviz環境設定などを読み込みます。多くのノードが同時に実行される実行ファイルであるため、実行されるファイルとノードを最初に確認してから実行してください。

```bash
$ roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml
```
{% capture capture07 %}
**roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml**
1. **urdf**
  - TurtleBot3とOpenMANIPULATORが結合した形のturtlebot3_manipulation_robot.urdf.xacroファイルを読み込みます。このファイルでは、TurtleBot3の形態を記述したファイルと、OpenMANIPULATORの形態を記述したファイルを結合し、全体的なロボットの形を作り上げます。
2. **robot_state_publisher**
  - robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
3. **laser_filter**
  - LDSセンサの設定された角度範囲データをフィルタリングします。
4. **map_server**
  - SLAMによって完成した地図と設定ファイルを読み込みます。
5. **amcl.launch**
  - AMCLパーティクルフィルタを使用するための各種パラメータを読み込みます。地図とセンサのscan値、ロボットのinitialposeとtfを読み取り、particle filterを使用して地図上でロボットの位置を予測します。
6. **move_base.launch**
  - move_baseパッケージのmove_baseノードは、ロボットのNavigation stackにアクセスするROSインターフェイスを提供します。move_baseノードは、global plannerとlocal plannerを接続してロボットを目的地まで移動させ、この時それぞれのplannerに合ったcostmapも保管します。ロボットの目的地(goal)をAction形態のトピックで受信すると、現在地(feedback)と状態(status)、移動の結果(result)をアップデートするため、同様にAction形態のトピックを使用します。また、現在の状態に合わせてロボットを動かすためのcmd_velトピックが持続的にpublishされます。
7. **rviz**
  - 各種データとパラメータを視覚化したGUIウィンドウを生成します。
{% endcapture %}
<div class="notice--success">{{ capture07 | markdownify }}</div>

![](/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_nav.png)

## OpenMANIPULATORを制御する
Navigationを実行する際にOpenMANIPULATORを制御するノードを生成すると、Navigationと共にロボットアームの制御が可能になります。
ロボットが動いている間にOpenMANIPULATORを動かす場合、振動や重心の移動によってロボットおよびロボットアームの動作が不安定になることがあります。ロボットアームは、ロボットが動いていない状態で動作させることをお勧めします。

### turtlebot3_manipulation_bringupノードを実行
[Remote PC] OpenMANIPULATORのみを制御するときと同様に、arm_controllerとgripper_controllerを実行します。
```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch**
1. **turtlebot3_manipulation_bringupノード**
  - turtlebot3_manipulation_bringup.launchを実行すると、arm_controllerとgripper_controllerこれら2つのコントローラーが実行されます。move_groupと通信するaction serverコントローラーの役割として、それぞれmove_groupを介してアームとグリッパー関節の目標軌跡を読み込み、順にpublishします。publishされたトピックは、OpenCRを介してロボットの関節に組み込まれたDYNAMIXELに伝達され、OpenMANIPULATORを動かします。

2. **turtlebot3_core.launch**
  - subscribe : cmd_vel
  - publish : joint_states, odom

3. **turtlebot3_lidar.launch**
  - publish : scan

turtlebot3_core.launchとturtlebot3_lidar.launchファイルが実行され、TurtleBot3の状態をチェックするノード(node)であるturtlebot3_diagnosticsが生成され、TurtleBot3の各種センサやハードウェアの状態についての情報をpublishします。turtlebot3_core.launchファイルでは、OpenCRと通信してjoint_states、odomをpublishし、cmd_velをsubscribeするノードが生成されます。turtlebot3_lidar.launchファイルでは、LIDARを作動させ、センサーから得られたscanデータをpublishするノード(node)が生成されます。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

### move_groupノードの実行
move_groupノードを実行した後は、MoveItを使用してOpenMANIPULATORを制御することも、ROBOTIS GUIを使用して制御することもできます。ここでは、ROBOTIS GUIを実行する方法を紹介します。2つのうち、適切なインターフェースを使用してください。

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
{% capture capture03 %}
**roslaunch turtlebot3_manipulation_moveit_config move_group.launch**

move_group.launchを実行すると、move_groupノードが実行されます。 move_groupノードは、ユーザーインタフェースを介してコマンドを受けとり、ロボットコントローラにaction形式で伝達します。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

### ROBOTIS GUIコントローラーを実行
[Remote PC] ROBOTIS GUIは、OpenMANIPULATORの1番目のDYNAMIXELを基準にグリッパーの有効な把持位置(グリッパー間の赤い六面体)をリファレンスとするTask Space Controlや各ジョイント関節の角度をレファレンスとするJoint Space Controlをサポートします。
必要に応じて使いやすい制御方法を使用することができます。

```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
{% capture capture05 %}
**roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch**

ユーザーインターフェースでC++ move_group_interfaceを使用したqt guiが実行されます。インタフェースを介して受けとった現在のジョイント位置およびend-effector位置がgui上に表示されます。Sendボタンをクリックすると、設定された位置値についてインターフェースを介してmove_groupに伝え、コントローラーに伝達し、ロボットを動かします。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>
