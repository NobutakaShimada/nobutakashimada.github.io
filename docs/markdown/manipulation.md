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

roscoreを起動した後で、以下のlaunchファイルをつかってGazeboを起動します。
```bash
$ roslaunch exp3 gazebo_manipulator_handle.launch
```
{% capture capture02 %}
**roslaunch exp3 gazebo_manipulator_handle.launch**

Gazebo上にOpenMANIPULATORを搭載したTurtleBot3 Waffle Piのモデルがロードされ、ロボットと通信するコントローラarm_controller、gripper_controllerがそれぞれ実行されます。これらはそれぞれ、ロボットアームの関節とグリッパーを制御するコントローラーです。
次節に説明するmove_groupノードがこれらのコントローラと通信してロボットを制御します。
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

move_group.launchを実行すると、move_groupノードが起動されます。 move_groupノードは、RViZなどのグラフィカルユーザーインタフェースなどを介してコマンドを受けとり、ロボットコントローラーにaction形式で伝達します。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_move_controller.png)

### Rvizを実行
[Remote PC] RViZの起動時にMoveIt環境が設定された`moveit.rviz`ファイルを読み込ませ、Rviz上でMoveItを使用可能にします。
GUIでInteractive Markerを活用したロボットアームを制御でき、目標位置への動作をシミュレートしてあらかじめ動きを確認してからロボット本体（Gazeboの場合はシミュレータ上のロボット）を実際に動かすことができます。

```bash
$ roslaunch exp3 moveit_rviz.launch
```
{% capture capture04 %}
**roslaunch exp3 moveit_rviz.launch**

このコマンドでMoveItが有効になったRvizが実行されます。Motion Planning pluginが起動し、moveit_setup_assistantを介して既に保存されているモーションやinteractive markerを介して設定したモーションを、move_groupに伝達することができます。目標姿勢を設定した後、Plan and Executeボタンを押すと、ロボットが動きはじめます。
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_rviz.png)

**注意**  
MoveIt!のInteractive Marker（上図のグリッパ部分に赤青緑などで表示されているマーカー）をマウスでつまんでアームを動かし、目標姿勢をつくることができます。この場合、MoveIt! ソフトウェアの機構学解析アルゴリズムに限界があり、円滑なコントロールができない可能性があります。
{: .notice--warning}

### ROBOTIS GUIコントローラーの実行
Rvizを使用せずにGazebo上のロボットと接続し、ロボットアームを制御する方法もあります。ROBOTIS GUIはOpenMANIPULATORの1番目のDYNAMIXEL（関節モーター）を基準にグリッパーの有効な把持位置(グリッパーの指間の赤い六面体)をリファレンスとするTask Space Controlや各ジョイント関節の角度を指定するJoint Space Controlをサポートします。
必要に応じて便利な制御方法を使用できます。

```bash
$ roslaunch exp3 gui_manipulation.launch
```
{% capture capture05 %}
**roslaunch exp3 gui_manipulation.launch**

ユーザーインターフェースでC++ move_group_interfaceを使用したqt guiが実行されます。インタフェースを介して受けとった現在のジョイント角度およびグリッパ位置がGUI上に表示されます。Sendボタンをクリックすると、設定された位置についてインターフェースを介してmove_groupに伝え、コントローラーに伝達しロボットを動かします。
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](/assets/images/ritsumeikan/tb3_omx_gui_controller.png)

## 実機でのOpenMANIPULATORの制御

MoveItのmove_groupノードは、以下のように様々な情報をもとに計算されたアーム・グリッパの軌跡を、ROSがサポートするaction形式でロボットコントローラーに提供する統合装置(intergrator)としての役割を持ちます。ユーザーは、move_groupノードにmoveitが提供する3種のインターフェース(C++、Python、RViz GUI)を通じてアクセスすることができます。ユーザーインターフェースを介してコマンドを受け取ると、move_groupノードはmoveit config情報(ジョイント角度の制限、機構学解析、衝突感知)およびロボットの状態情報に基づいて軌跡を生成し、ロボットコントローラーに渡します。
この実験ではそこまで詳細な内部構造には踏み込みませんので安心してください:-) 。

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
$ roslaunch exp3 gui_manipulation.launch
```
<div class="notice--success">{{ capture05 | markdownify }}</div>

## 課題（実機編）

{% capture staff01 %}
1. GazeboシミュレータとRViZを起動して、RViZ上のMotion Planningタブ上でアームとグリッパそれぞれに目標姿勢を指定し、軌道生成を行ってアームとグリッパを動作させてみよ。２通りくらいの例を実行し、初期姿勢の状態と動作後のゴール状態についてRViZ上のロボット表示とGazebo上のロボットの様子を画面キャプチャしてノートに貼り付けよ。
2. 1.と同様の内容を実機ロボットと接続して実行せよ。Gazebo上のロボットの代わりに実機の姿勢をスマフォ等で撮影して貼り付けよ。
3. 実機を操作して適当な物体を把持してみよ。レンジセンサの出力をRViZ上で確認すると目標位置の参考になるかもしれない。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

## 発展テーマ（解説未完成）

この節はROSパワーユーザ用の発展課題用メモです。以下の説明では実行することが困難なので、あくまで参考のための資料です。

### MoveIt!
MoveIt!はロボットの動作軌道を自動生成するためのツールで、RViZに組み込まれたGUIからも操作できますが、pythonスクリプトから実行することもできます。
詳細は次のページに記載されています。ロボットとシーンを定義し、ロボットに対してエンドエフェクタ（手先）の三次元位置と向きのゴール姿勢を指定すると、現在姿勢からゴール姿勢までの動作軌道を計画して動作命令を発行することができます。
[MoveIt!](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)

### Obstacle Detector
Obstacle Detectorは、LRF(レーザーレンジセンサー)の計測データ（/scanトピックにpublishされている）をsubscribeして、線上のオブジェクトと円形状のオブジェクトを検出し、obstacle_detector/Obstacles型のメッセージとして/obstaclesトピックに検出された座標を出力するROSノードモジュールです。
[日本語解説記事](https://qiita.com/srs/items/d3b9425cedf57ab269ff)
[Github](https://github.com/tysik/obstacle_detector)

これらを組み合わせると、ペットボトルのような円筒形の物体を検出してグリッパをその位置に移動しつかむ動作を生成することができます。

ただし、Obstacle Detectorは配布したLinuxシステムにインストールされていないので、Githubからダウンロードしてインストールする必要があります。
```bash
$ cd ~/exp3_ws/src
$ git clone https://github.com/tysik/obstacle_detector.git
$ cd ~/exp3_ws
$ catkin_make
```
<div class="notice--success">{{ capture06 | markdownify }}</div>

上記の操作はかならず教員に相談してから実行してください。ROSの環境を壊してしまう可能性があります。
