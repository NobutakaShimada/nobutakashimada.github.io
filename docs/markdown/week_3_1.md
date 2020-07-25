---
layout: splash
lang: en
ref: phyexp3_3-1
permalink: /docs/week3-1/
sidebar:
  title: ５コマ目
  nav: "phyexp3_3-1"
---

# [Class 5] Manipulation

OpenMANIPULATOR-Xは、ROSをサポートするROBOTISのオープンソースロボットアームです。DYNAMIXELと3Dプリンタを利用して製作された部品で、組み立てが可能なため作りやすく、価格が手頃だという利点があります。  
特にOpenMANIPULATOR-Xは、TurtleBot3 WaffleやWaffle Piとの互換性を持つよう設計されており、ここではTurtleBot3 Waffle Piに組み付けられたマニピュレータを使う方法について説明します。

## Softwareを設定
[Remote PC] TurtleBot3に組み付けられたOpenMANIPULATOR-Xを使用するためのパッケージをダウンロードし、ビルドします。

```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ sudo apt install ros-kinetic-ros-control && ros-kinetic-ros-controllers && ros-kinetic-control* && ros-kinetic-moveit*
$ cd ~/catkin_ws && catkin_make
```

## Hardwareを設定
TurtleBot3 Waffle PiのLDSセンサは、ロボットの中央部分に位置しています。  
OpenMANIPULATOR-Xを取り付けるためには、LDSセンサの位置を下の図の赤いボックスに向かって移動させ、黄色いボックスにOpenMANIPULATOR-Xの第1関節を取り付ける必要があります。  
正確な位置に取り付けなかった場合、ロボットの構成を説明するURDFに定義されたセンサーとロボットアームの位置が実際のロボットの位置と異なり、意図しない動作が実行されたり、ロボットアームが意図しない場所に移動して衝突が起こる場合があります。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble_points.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble.png)

## OpenCRを設定
OpenMANIPULATOR-XがTurtleBot3 Waffle PiのOpenCRに接続されると、OpenCRが接続されたすべてのダイナミックセルを制御できるように、ファームウェアを変更する必要があります。皆さんのTurtleBot3 Waffle Piには、既にこのようなファームウェアがアップロードされています。

[TurtleBot3 SBC] OpenCRファームウェアをTurtleBot3のRaspberry Piでアップロードするには、以下の通りコマンドを入力します。
```bash
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=om_with_tb3
$ rm -rf ./opencr_update.tar.bz2
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 && tar -xvf opencr_update.tar.bz2 && cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
```
コマンドを入力してしばらくすると、新しいファームウェアがOpenCRにアップロードされ、アップロードが成功すると、ターミナルウィンドウの末尾に`jump_to_fw`というフレーズが表示されます。

{% capture danger01 %}
**WARNING** : OpenCRのファームウェアが正常に更新されるとOpenCRが再起動され、OpenMANIPULATOR-Xがデフォルトの位置に動くようになります。したがって、OpenMANIPULATOR-Xの電線が捻れたり、デフォルトの位置に動いている間に本体や他の物体と衝突することがないよう、以下のような姿勢を整えてからファームウェアをアップデートしてください。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_gazebo_1.png)
{% endcapture %}
<div class="notice--danger">{{ danger01 | markdownify }}</div>

## TurtleBot3 Bringup

### roscoreを実行する
[Remote PC] ROSを駆動するためのroscoreをユーザーのPCで動作させます。
```bash
$ roscore
```

### TurtleBot3モデルを定義する
[TurtleBot3 SBC] `.bashrc`ファイルにTURTLEBOT3_MODELについて定義を行っていない場合、以下のコマンドによって使用中のTurtleBot3モデルを定義しなければなりません。`waffle_pi`以外にも、製品のハードウェア構成に応じて`burger`や`waffle`を使用することができます。
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
```

### Bringupを実行する
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

OpenCRのファームウェアが変更されたため、turtlebot3_robot.launchファイルを実行すると、Week1で説明したturtlebot3_robot.launchを実行させたときにpublishされるトピックに加えて、joint_trajectory_point、gripper_positionのトピック2種をsubscribeします。joint_trajectory_pointはOpenMANIPULATORの各関節の位置値を伝達し、OpenCRを通じてOpenMANIPULATORを構成するDYNAMIXELアクチュエータに伝達されます。gripper_positionはOpenMANIPULATORグリッパーの位置値であり、joint_trajectory_pointと同様に、OpenCRを通じてグリッパーを構成するDYNAMIXELに伝達され、グリッパーを制御します。rqtのMessage Publisherを使用して位置値を転送する方法によっても簡単に制御することができます。
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>

# GazeboシミュレータでOpenMANIPULATORを制御する

## Gazeboシミュレータの実行
[Remote PC] 以下のコマンドを新たなターミナルウィンドウに入力し、OpenMANIPULATORが適用されたTurtleBot3のモデルをGazebo環境でロードします。
```bash
$ roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation.launch**

Gazebo上にOpenMANIPULATORが結合されたTurtleBot3 Waffle Piモデルがロードされ、ロボットと通信する2つのロボットコントローラであるarm_controller、gripper_controllerが実行されます。これらはそれぞれ、ロボットアームの関節とグリッパーを制御するコントローラーです。
方式は、実際のロボットを使用する場合と同じです。以下のコードを実行し、move_groupと通信してロボットを制御します。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

![](/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_gazebo.png)

## move_groupノードを実行

[Remote PC] MoveItと連動するためにはmove_groupノードを実行する必要があります。Gazeboシミュレータで[▶] 実行ボタンを押してシミュレーションを開始した場合、以下のコマンドを入力した後、下の図のように"**You can start planning now!**"というメッセージが表示されます。

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
{% capture capture03 %}
**roslaunch turtlebot3_manipulation_moveit_config move_group.launch**

move_group.launchを実行すると、move_groupノードが実行されます。 move_groupノードは、ユーザーインタフェースを介してコマンドを受けとり、ロボットコントローラーにaction形式で伝達します。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

![](/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_move_controller.png)

## Rvizを実行
[Remote PC] MoveIt環境が設定された`moveit.rviz`ファイルを読み込み、RvizでMoveItを使用可能にします。
GUIでInteractive Markerを活用したロボットアームを制御でき、目標位置への動作をシミュレートすることができるため、衝突などに備えることが可能です。

```bash
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
{% capture capture04 %}
**roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch**

MoveItが有効になったRvizが実行されます。Motion Planning pluginが実行され、それまでにmoveit_setup_assistantを介して既に保存されているモーションやinteractive markerを介して設定したモーションを、move_groupに伝達することができます。目標位置を設定した後、Plan and Executeボタンを押すと、ロボットが動きはじめます。
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_rviz.png)

**注意**  
MoveIt!のInteractive Markerを活用してOpenMANIPULATORーXをコントロールする場合、MoveIt! ソフトウェアの気候学解析に限界があり、円滑なコントロールが出来ない可能性があります。
{: .notice--warning}

## ROBOTIS GUIコントローラーを実行
[Remote PC] Rvizを使用せずにGazeboと接続し、ロボットアームを制御する場合は、ロボティズGUIはOpenMANIPULATORの1番目のDYNAMIXELを基準にグリッパーの有効な把持位置(グリッパー間の赤い六面体)をリファレンスとするTask Space Controlや各ジョイント関節の角度を基準とするJoint Space Controlをサポートします。
必要に応じて便利な制御方法を使用できます。

```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
{% capture capture05 %}
**roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch**

ユーザーインターフェースでC++ move_group_interfaceを使用したqt guiが実行されます。インタフェースを介して受けとった現在のジョイント位置およびend-effector位置がgui上に表示されます。Sendボタンをクリックすると、設定された位置値についてインターフェースを介してmove_groupに伝え、コントローラーに伝達しロボットを動かします。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_gui_controller.png)

# 実際のOpenMANIPULATORを制御する

MoveItのmove_groupというノードは、以下のように様々な情報をもとに計算された軌跡を、ROSがサポートするaction形式でロボットコントローラーに提供する統合装置(intergrator)としての役割を果たします。ユーザーは、move_groupノードにmoveitが提供する3種のインターフェース(C++、Python、RViz GUI)を通じてアクセスすることができます。ユーザーインターフェースを介してコマンドを受け取ると、move_groupノードはmoveit config情報(ジョイント角度の制限、機構学解析、衝突感知)およびロボットの状態情報に基づいて軌跡を生成し、ロボットコントローラーに提供します。

![](/ritsumeikan_github/assets/images/ritsumeikan/move_group.png)

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

[Remote PC] 基本的なTurtleBot3のプラットフォームとは異なり、OpenMANIPULATORを制御できるサービスサーバが必要であるため、このBringupでは、以前の講義でPC実行していたturtlebot3_remote.launchを終了し、以下のようにManipulationに特化したlaunchファイルを実行します。

```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch**
**turtlebot3_manipulation_bringupノード**

turtlebot3_manipulation_bringup.launchを実行すると、arm_controllerとgripper_controllerこれら2つのコントローラーが実行されます。move_groupと通信するaction serverコントローラーの役割として、それぞれmove_groupを介してアームとグリッパー関節の目標軌跡を読み込み、順にpublishします。publishされたトピックは、OpenCRを介してロボットの関節に組み込まれたDYNAMIXELに伝達され、OpenMANIPULATORを動かします。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

## move_groupを実行する
[Remote PC] MoveItと連動しているユーザーインターフェースであるmove_groupノードを実行します。
```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
<div class="notice--success">{{ capture03 | markdownify }}</div>

## RVizを実行する
[Remote PC] 各種データの視覚化とInteractive Markerを活用したOpenMANIPULATORの制御のため、RVizを実行します。
```bash
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
<div class="notice--success">{{ capture04 | markdownify }}</div>

## ROBOTIS GUI を実行する
[Remote PC] RVizとは別に、必要に応じてROBOTIS GUIを介し、OpenMANIPULATORを制御することもできます。
```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
<div class="notice--success">{{ capture05 | markdownify }}</div>
