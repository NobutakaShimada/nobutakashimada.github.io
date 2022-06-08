---
layout: splash
lang: en
ref: phyexp3\_gazebo
permalink: /docs/gazebo/
sidebar:
  title: GazeboシミュレーションによるTurtlebot3の操作
  nav: "phyexp3\_basic"
---

{: .startsec2}
# [Class 2] GazeboシミュレーションによるTurtlebot3の操作

ROSサポートの市販ロボットではよくあることですが、TurtleBot3には仮想ロボットでプログラミングや開発可能な環境が用意されています。
１つの方法はフェイクノードと３D視覚化ツールであるRVizを使う方法、もう一つの方法は、3-DロボットシミュレーターのGazeboを使用することです。 
フェイクノードの方法では、ロボットの動きをテストすることはできますが、センサーの情報をシミュレートして取得することはできません。
SLAMなどセンサー情報の処理をテストするときは、センサーおよびカメラがのシミュレーションが使用できる物理シミュレータGazeboを使う必要があります。
この実験ではGazeboを使って端末上でロボットの操作を仮想的に実行してみましょう。


## Gazeboを使ったTurtlebot３シミュレーション 

実験用のUbuntu LinuxにはすでにGazeboの起動環境をインストールしてあります。
自分でセットアップしたPCでGazeboを初めて実行する場合は普段より起動に時間がかかることがあります（形状モデルや力学モデルをインターネットからダウンロードするため）。
### Empty World
次のコマンドを使用して、Gazeboのデフォルト設定のWorldで、仮想TurtleBot３をテストする際に使用できます。 

```bash
$ roslaunch exp3 empty_world.launch
```

{% capture capture03 %}
**roslaunch exp3 empty_world.launch**
- publish : joint_states, odom, scan, tf
- subscribe : cmd_vel

1. empty_world.launchを実行すると、設定ファイルに従ってGazeboシミュレータが実行され、設定されたTurtleBot3モデルがGazeboシミュレータに生成されます。この時読み込まれる設定ファイル（xacroとURDFファイル）に従って距離センサ（scan）、オドメトリー（odom）、関節モーターの状態（joint_states）、ロボット本体と関節部やセンサー系への座標変換（tf:transform）のトピックをpublishし、移動速度指示（平行移動速度、旋回角速度）(cmd_vel)をsubscribeします。

2. Gazeboが起動しウィンドウが表示されたら、**物理シミュレーションをスタートさせる必要**があります。**画面の左下のプレイボタン[▶] を押して時間をスタートさせる**ことを忘れないようにしましょう。
3. 次にteleop.launchを実行して画面の指示に従ってキーボードのキーを押すと、Gazeboシミュレータがteleopノードが生成した速度指令値を読み取り、シミュレータに生成されたロボットが動くことを確認できます（次節で詳述する）。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

![](/assets/images/ritsumeikan/empty_world_turtlebot.png)

### TurtleBot３ World 
他にもいくつかのオブジェクトを配置したworldを読み込むことができるので、試してみましょう。
TurtleBot３ Worldは、TurtleBot３のシンボルの形状を構成するシンプルなオブジェクトで構成されているマップです。
 
```bash
$ roslaunch exp3 gazebo_manipulator_world.launch
```
![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_waffle.png)

### TurtleBot３ House 
TurtleBot３ House は住居の図面で制作されたマップです。

```bash
$ roslaunch exp3 gazebo_manipulator_house.launch
```

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house1.png)

### TurtleBot3 Stage4
TurtleBot３ Stage4 は壁で囲まれた迷路状のコースです。

```bash
$ roslaunch exp3 gazebo_manipulator_stage_4.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/machine_learning/stage_4.jpg)


## Gazebo上のTurtleBot3の動作指令

### キー入力による遠隔操作 
TurtleBot3 をキーボードでコントロールするために、新しいターミナルで下記のコマンドを使って遠隔操作を実行します。 
```bash
$ roslaunch exp3 teleop.launch
  Control Your TurtleBot3!
  ---------------------------
  Moving around:
           w
      a    s    d
           x
  w/x : increase/decrease linear velocity
  a/d : increase/decrease angular velocity
  space key, s : force stop
CTRL-C to quit
```  
キーボードのキーで自由にGazebo内のロボットを動かしてみる。
{% capture capture02 %}
**roslaunch exp3 teleop.launch**

teleop.launchファイルを実行して生成されたturtlebot3_teleop_keyboardノードでは、キーボードの入力を読み取って「あるトピック」に並進速度と旋回速度の各成分をpublishする。publishされたメッセージは最終的にロボット内のファームウェアに伝達されDYNAMIXELを制御するためのコマンドとして出力される。受信されたコマンドに従って車輪と接続されたDYNAMIXELが駆動し、ロボットを動かす。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>


### 障害物回避による自律走行 
センサーの情報を読み取って障害物にぶつからないようにTurtleBot3を自律走行させるノードを実行できます。
```bash
$ roslaunch exp3 automove.launch
```
{% capture capture02 %}
**roslaunch exp3 automove.launch**
automove.launchファイルを実行すると、turtlebot3_gazeboパッケージのturtlebot3_driveノードを起動する。turtlebot3_driveノードはC++で書かれた自動走行指示をpublishするノードで、ソースコードはturtlebot3_gazebo/src/turtlebot3_drive.cppである。このソースファイルの中の閾値（どのくらいの距離で障害物と判定するか）や回避ロジックを変更すれば、独自の自動走行をさせることができるようになる。

ソースファイルを変更した時は、`~/catkin_ws`フォルダに移動して、`catkin_make`コマンドを実行する必要がある。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

## Rvizによる情報可視化
Rvizはシミュレーションが実行されている間に発行されたトピックを視覚化します。下記のコマンドを入力し、新しいターミナルウィンドウでRVizをスタートすることができます。 
```bash
$ roslaunch exp3 rviz.launch
```
![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

## 課題2
{% capture staff01 %}
1. Gazeboシミュレータ(gazebo.launch)とRViz(rviz.launch)、teleop(teleop.launch)を起動してTurtlebotを適当に移動させ、その時のGazeboの画面（視点を自由に移動せよ）、RVizの画面をキャプチャしてノートブックに添付せよ。画面キャプチャには`gnome-screenshot`コマンドを使うことができる。
```bash
$ gnome-screenshot --area -f graph.jpg
```
2. rosnode listコマンド、を実行してどのようなノードが起動しているか確認し、結果をノートブックに添付せよ。
3. rostopic listコマンドを実行して、どのようなトピックが作られているか確認し、結果をノートブックに添付せよ。
4. rostopic echoコマンドを実行して、いくつかのトピックメッセージが流れている様子を観察せよ。観察したトピック名とその内容（大量にあるので１０行程度で良い）をノートブックに添付せよ。
5. teleop.launchや各種コマンドを駆使して、Turtlebot3への行動命令が流れているトピックを特定せよ。そのトピック名と、メッセージの型名、メッセージの定義(rosmsg showコマンドを用いよ）をノートブックに添付せよ。
6. rqt_graphコマンドを実行して、ノード・トピックの関係図を表示し、画像としてノートブックに添付しなさい。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>
