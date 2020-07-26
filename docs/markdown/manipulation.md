---
layout: splash
lang: en
ref: phyexp3_manipulation
permalink: /docs/manipulation/
sidebar:
  title: OpenMANIPULATOR-Xを使ったマニピュレーション
  nav: "phyexp3_manipulation"
---

# [Class 6] OpenMANIPULATOR-Xを使ったマニピュレーション

実験で用いるTurtlebot3は、上部に５自由度のOpenMANIPULATOR-Xマニピュレータを搭載しています。このアームもTurtlebot3本体同様にROSによって動作させることができます。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_gazebo_1.png)


## GazeboシミュレータでOpenMANIPULATORを制御する

### Gazeboシミュレータの実行

roscoreを起動した後で、Gazeboを起動します。
```bash
$ roslaunch exp3 gazebo_manipulator_handle.launch
```
{% capture capture02 %}
**roslaunch exp3 gazebo_manipulator_handle.launch**

Gazebo上にOpenMANIPULATORが結合されたTurtleBot3 Waffle Piモデルがロードされ、ロボットと通信する2つのロボットコントローラであるarm_controller、gripper_controllerが実行されます。これらはそれぞれ、ロボットアームの関節とグリッパーを制御するコントローラーです。
方式は、実際のロボットを使用する場合と同じです。以下のコードを実行し、move_groupと通信してロボットを制御します。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble.png)

### move_groupノードの実行

アーム・グリッパの行動計画にはMoveItという衝突回避計画生成の機能を持った行動計画モジュールを用いると便利です。
MoveItと連動するためにはmove_groupノードを実行する必要があります。Gazeboシミュレータで[▶] 実行ボタンを押してシミュレーションを開始した後に、以下のコマンドを入力すると、下の図のように"**You can start planning now!**"というメッセージが表示されます。このメッセージが出力されない場合はGazeboの実行ボタンを押し忘れている可能性があります。

```bash
$ roslaunch exp3 move_group.launch
```
{% capture capture03 %}
**roslaunch exp3 move_group.launch**

move_group.launchを実行すると、move_groupノードが実行されます。 move_groupノードは、ユーザーインタフェースを介してコマンドを受けとり、ロボットコントローラーにaction形式で伝達します。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_move_controller.png)

### Rvizを実行
[Remote PC] MoveIt環境が設定された`moveit.rviz`ファイルを読み込み、RvizでMoveItを使用可能にします。
GUIでInteractive Markerを活用したロボットアームを制御でき、目標位置への動作をシミュレートすることができるため、衝突などに備えることが可能です。

```bash
$ roslaunch exp3 moveit_rviz.launch
```
{% capture capture04 %}
**roslaunch exp3 moveit_rviz.launch**

MoveItが有効になったRvizが実行されます。Motion Planning pluginが実行され、それまでにmoveit_setup_assistantを介して既に保存されているモーションやinteractive markerを介して設定したモーションを、move_groupに伝達することができます。目標位置を設定した後、Plan and Executeボタンを押すと、ロボットが動きはじめます。
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_rviz.png)

**注意**  
MoveIt!のInteractive Markerを活用してOpenMANIPULATORーXをコントロールする場合、MoveIt! ソフトウェアの機構学解析アルゴリズムに限界があり、円滑なコントロールができない可能性があります。
{: .notice--warning}

### ROBOTIS GUIコントローラーの実行
Rvizを使用せずにGazeboと接続し、ロボットアームを制御する場合は、ROBOTIS GUIはOpenMANIPULATORの1番目のDYNAMIXELを基準にグリッパーの有効な把持位置(グリッパー間の赤い六面体)をリファレンスとするTask Space Controlや各ジョイント関節の角度を基準とするJoint Space Controlをサポートします。
必要に応じて便利な制御方法を使用できます。

```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
{% capture capture05 %}
**roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch**

ユーザーインターフェースでC++ move_group_interfaceを使用したqt guiが実行されます。インタフェースを介して受けとった現在のジョイント位置およびend-effector位置がgui上に表示されます。Sendボタンをクリックすると、設定された位置値についてインターフェースを介してmove_groupに伝え、コントローラーに伝達しロボットを動かします。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_gui_controller.png)

## 実機でのOpenMANIPULATORの制御

MoveItのmove_groupノードは、以下のように様々な情報をもとに計算されたアーム・グリッパの軌跡を、ROSがサポートするaction形式でロボットコントローラーに提供する統合装置(intergrator)としての役割を持ちます。ユーザーは、move_groupノードにmoveitが提供する3種のインターフェース(C++、Python、RViz GUI)を通じてアクセスすることができます。ユーザーインターフェースを介してコマンドを受け取ると、move_groupノードはmoveit config情報(ジョイント角度の制限、機構学解析、衝突感知)およびロボットの状態情報に基づいて軌跡を生成し、ロボットコントローラーに渡します。

![](/assets/images/ritsumeikan/move_group.png)

### roscoreを実行する
roscoreをユーザーのPCで動作させます。
```bash
$ roscore
```

### Turtlebot3実機に接続する。
以下のコマンドによってrosserialとLDSセンサを動作させるノードを実行します(ロボット番号"09"の場合）。
```bash
$ roslaunch exp3 machine.launch id:=09
```
{% capture capture02 %}
**roslaunch exp3 machine.launch id:=09**

machine.launchを実行すると、以前と同じくSSH経由でTurtlebot3内で必要なノードを自動起動します。その中でアーム・グリッパに関連するノードとして、arm_controllerとgripper_controllerのコントローラーノードが実行されます。move_groupと通信するaction serverコントローラーの役割として、それぞれmove_groupを介してアームとグリッパー関節の目標軌跡を読み込み、順にpublishします。publishされたトピックは、ロボットの関節に組み込まれたモータに伝達され、OpenMANIPULATORを動かします。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

### move_groupを実行する
MoveItと連動しているユーザーインターフェースであるmove_groupノードを実行します。
```bash
$ roslaunch exp3 move_group.launch
```
<div class="notice--success">{{ capture03 | markdownify }}</div>

### RVizを実行する
各種データの視覚化とInteractive Markerを活用したOpenMANIPULATORの制御のため、RVizを実行します。
```bash
$ roslaunch exp3 moveit_rviz.launch
```
<div class="notice--success">{{ capture04 | markdownify }}</div>

### ROBOTIS GUI を実行する
RVizとは別に、必要に応じてROBOTIS GUIを介し、OpenMANIPULATORを制御することもできます。
```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
<div class="notice--success">{{ capture05 | markdownify }}</div>
