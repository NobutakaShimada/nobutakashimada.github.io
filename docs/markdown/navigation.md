---
layout: phyexp3-lesson
lang: en
ref: phyexp3_navigation
permalink: /docs/navigation/
section_number: 5
section_title: 生成した環境地図に基づくナビゲーション
sidebar:
  title: 生成した環境地図に基づくナビゲーション
  nav: "phyexp3_navigation"
---
{% assign wayback_prefix = "https://web.archive.org/web/20200929183646/" %}

ナビゲーション（誘導）は、特定の環境で指定された位置にロボットを移動させることです。そのために与えられた環境にある障害物や壁などの幾何的な情報が含まれた地図が必要です。SLAMを実行することによって、
センサが獲得した距離情報やロボットの移動量情報から未知環境の地図を自動的に生成することができました。
ナビゲーションを利用すると、地図情報、ロボットの内界センサ（車輪に装着された回転計：エンコーダ）、IMUセンサ及び距離センサ）の情報を使って、ロボットが現在位置から地図上の目標位置に移動することができます。

ナビゲーションのためには、地図上の各位置に対するcostmapの計算が必要です。costmapは地図の上の移動可能な空き領域と進入不可能な障害物領域を数値を使って表現したものです。
これはロボットの位置・方向の自己位置推定情報とセンサの値から得られる障害物情報、そしてSLAMによって得られた地図に基づいて計算されます。costmapはロボットが衝突する領域、衝突可能性がある領域、自由に移動可能な領域などで表され、これをもとにロボットが安全に目標位置まで移動する経路の生成を行います。costmapは2種類に分類することができ、ロボットのスタート位置から目的地までの経路計画を策定するために使用されるglobal costmapと、ロボットの周辺において障害物などを回避するための安全な経路を計算するlocal costmapがあります。ROSでこのようなcostmapは画像上の画素値で表現されますが、0はロボットが自由に移動できる領域で、255に近づくほどロボットの衝突が発生する可能性がある危険の大きい領域として扱われます。

  - 0 : 自由領域
  - 1 ~ 127 : 低い衝突可能性のある領域
  - 128 ~ 252 : 高い衝突可能性のある領域
  - 253 ~ 254 : 衝突領域
  - 255 : ロボットが移動できない領域

{::comment}
The below image 'costmapspec.png' is retrieved from
https://wiki.ros.org/costmap_2d#Inflation by the following command;
curl --remote-name --remote-time --remote-header-name 'https://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png'
{:/comment}
![](/assets/images/ritsumeikan/costmapspec.png)

> [https://wiki.ros.org/costmap_2d#Inflation](https://wiki.ros.org/costmap_2d#Inflation)([URL on Wayback Machine]({{wayback_prefix}}https://wiki.ros.org/costmap_2d#Inflation))

DWA(Dynamic Window Approach)は、障害物を回避する経路の生成に使用される代表的な方法です。DWAは速度空間内で並進速度(v)と回転速度(ω)を使用し、ロボットが移動できる経路を予測・計算し、ハードウェアが出しうるロボットの最大速度が限界値として定められます。dwa_local_plannerでは、ロボットが目的地に到着できるよう進む経路であるglobal planと、障害物の回避に使われるglobal costmapによってロボットに移動可能な速度コマンドを計算し、local planを作成します。DWAは以下の順序で動作します。

1. ロボットの制御位置(dx、dy、d&theta;)を個別に生成します。
2. 生成されたそれぞれのサンプルを、ロボットの現在地からシミュレーションし、短時間サンプルが適用される場合、ロボットの位置を予測します。
3. シミュレーションの結果を点数に換算します。このとき、障害物との距離、目的地までの距離、global planとの距離、速度などの要素が考慮され、物体と衝突するサンプルは結果の計算から排除されます。
4. 最も高いスコアを獲得したサンプルが選択され、ロボットに伝達されます。
5. 1 ~ 4のプロセスを繰り返します。

これら自己位置推定やそれに基づくcostmapの生成、さらに移動経路計画の立案はROSの標準モジュールによってある程度実現されています。この実験ではこれらを起動して、生成した地図を使ったナビゲーションを体験します。


## ナビゲーションの実行（Gazebo/Turtlebot3実機）

### roscoreの実行（Gazebo/実機の場合共通）
  ```bash
  $ roscore
  ```

### Gazeboの実行 (Gazeboの場合のみ)
  1. launchファイルの実行
```bash
  $ roslaunch exp3 gazebo_manipulator_world.launch
  ```
  別の環境シーンを使って地図を作った場合は、その地図に対応したlaunchファイルを指定して起動すること（違う地図を使うとナビゲーションに失敗します）。

  2. 続いてGazeboのシミュレーションスタートボタン[▶] （ウィンドウ左下の三角）を押す。**忘れないこと！忘れるとシミュレーションやSLAMがスタートしません**

### TurtleBot3への接続 (ロボット番号09の場合) (実機の場合のみ)
  ```bash
  $ roslaunch exp3 machine.launch id:=09
  ```
  {% capture capture01 %}
  **roslaunch exp3 machine launch id:=09**
  1. turtlebot3_core.launch
      - subscribe : cmd_vel
      - publish : joint_states, odom

  2. turtlebot3_lidar.launch
      - publish : scan

machine.launchファイルを実行すると、SSHによってTurtlebot3に自動的に接続し、Turtlebot3内でローカルノードを自動的に起動します。
Turtlebot3の中では、turtlebot3_core.launchとturtlebot3_lidar.launchファイルが実行され、TurtleBot3の状態をチェックするノード(node)であるturtlebot3_diagnosticsが生成され、TurtleBot3の各種センサやハードウェアの状態についての情報をpublishします。turtlebot3_core.launchファイルでは、OpenCRと通信してjoint_states、odomをpublishし、cmd_velをsubscribeするノードが生成されます。turtlebot3_lidar.launchファイルでは、LIDARを作動させ、センサーから得られたscanデータをpublishするノード(node)が生成されます。
  {% endcapture %}
  <div class="notice--success">{{ capture01 | markdownify }}</div>

### navigation.launchの実行（Gazebo/実機の場合共通）
  ```bash
  $ roslaunch exp3 navigation.launch map_name:=map1
  ```
  上の例はSLAMで生成し保存した地図の名前（拡張子を除く）が"map1"の場合。
{% capture capture02 %}
**roslaunch exp3 navigation.launch map_name:=map1**
1. **roslaunch turtlebot3_bringup turtlebot3_remote.launch**
  - urdf：Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
  - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
    - subscribe : joint_states 
    - publish : tf

  turtlebot3_remote.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
  turtlebot3_slam.launchファイル内部にturtlebot3_remote.launchが含まれているのでturtlebot3_slam.launchが実行されると自動的にturtlebot3_remote.launchが最初に実行されます｡

2. map_serverノード
  - publish : map_metadata, map
  - map_serverノードは、ディスクに保存されている地図を呼び出す役割を行います。ターミナルに入力されたコマンドでmap_fileパラメータは、地図の情報が保存されたファイルの位置を伝えます。

3. amcl.launch
  - publish : tf, amcl_pose, particlecloud
  - subscribe : scan, tf, initialpose, map
  - 地図とセンサーのscan値、ロボットのinitialposeとtfを読み取り、particle filterを使用して地図上でロボットの位置を予測します。

  amclはParticle Filterというモンテカルロ法に基づく確率的状態推定アルゴリズムによって、センサ情報と地図情報からロボットの自己位置（２次元平面上の座標(x,y)とロボットの向きθ）を推定するノードです。RVizを起動すると、細かい矢印が多数表示されますが、これがamclの推定している推定パーティクルで、１つ１つが確率値を持つ推定候補です。最初は広い範囲に散らばっていて自己位置姿勢の推定が不確かなことが分かりますが、移動しながら情報を集めることによって次第にパーティクルが集まってきて真の自己位置が確からしく推定できるようになることを目視でよく確認してください。

4. move_base
  - subscribe : goal, cancel
  - publish : feedback, status, result, cmd_vel

  move_baseパッケージのmove_baseノードは、ロボットのNavigation stackにアクセスするROSインターフェイスを提供します。move_baseノードは、global plannerとlocal plannerを接続してロボットを目的地まで移動させ、この時それぞれのplannerに合ったcostmapも保管します。ロボットの目的地(goal)をAction形態のトピックで受信すると、現在地(feedback)と状態(status)、移動の結果(result)をアップデートするため、同様にAction形態のトピックを使用します。また、現在の状態に合わせてロボットを動かすためのcmd_velトピックが持続的にpublishされます。

5. rviz
  - subscribe : tf, odom, map, scan

最後にrvizが自動的に実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成された地図を表示し、現在の自己位置推定の様子が可視化され、また移動の目標地点を指定してロボットを誘導することができます。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>


### ロボットの初期姿勢を指定（Gazebo/実機の場合共通）

ナビゲーションを行う際に最も重要なことは、ロボットの正確な初期位置を地図上で示すことです。元々どこにいそうなのかがわからなければ、ロボットが自己位置を推定するにはたくさんの可能性を試さねばならず時間と計算コストが非常にかかります。  
ロボットのエンコーダとIMU、LDSなど各種センサから得られた情報をもとにTurtleBot3の位置を推定するには、確率をベースにしたAMCL(Adaptive Monte Carlo Localization)というparticle filterが使用されます。AMCLは、センサ情報に基づいてロボットの位置を推定します。また、アルゴリズムのパラメータに設定された移動量をロボットが移動するたびに、推定位置の値を更新し、更新が繰り返されるたびに推定位置の誤差が減少していきます。

RVizが起動し、ロボットがナビゲーションを実行するための地図が表示されたら、最初のロボットの位置は実際のロボットの現在位置とはズレています。  
そこで、Rviz画面の上にあるボタンの中から`2D Pose Estimate`ボタンでロボットの初期位置を実際のロボット位置姿勢に合わせます。  

  1. Rvizで`2D Pose Estimate`ボタンをクリックする。
  2. 地図上でロボットが実際に位置する点をクリックし、ロボットの前面が向いている方向に矢印をドラッグして、大きな緑色の矢印の方向をロボットが向いている方向に設定する。

これが完了すると、ロボットは緑の矢印で指定された位置と方向を初期ポーズとして使用し、実際の位置と方向を推定します。緑の矢印はタートルボット３の予想位置を表示します。レーザースキャナーは地図上に今ロボットが観測している壁の位置を表示します。地図とのズレが大きい場合はやり直します。多少ずれていてもロボットを移動させて観測情報を増やしていくと、次第に推定が確からしくなります。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_pose_estimate.png)

> 上の図で、ロボットの周辺に広く表示された多数の小さな緑の矢印は、ロボットの初期位置を指定した際にロボットのセンサーと各種情報を介してAMCLが計算したロボットの現在位置です。ロボットの移動に従って分布していた現在地の予想値がロボットに向かって近づき、収束する様子が分かります。

> **注意**： ナビゲーションを実行する前にteleop.launchが実行されている場合は、必ず終了させる必要があります。teleopは常に指定された速度(cmd_velトピック）を指令し続けているので、move_baseの指示する速度と衝突してうまく動作できなくなります）。

### Navigation Goalを設定する（Gazebo/実機の場合共通）

次にロボットを目標位置に誘導するために、到着目的地の位置と方向を指定します。初期位置の指定と同様に、Rvizの`2D Nav Goal`ボタンをクリックし、ロボットが移動できる目的地をクリックし、ロボットが向いている方向をドラッグして、矢印の方向を指定します。

1. Rviz上部の `2D Nav Goal`ボタンをクリック。
2. 地図上でロボットが移動する目的地をクリックし、ロボットが向いている方向をドラッグして、目的地設定を完了。

目的地設定が完了すると、Navigationアルゴリズムはロボットの現在位置から目的地までの経路を生成し、経路を辿れるようにロボットに命令を出します。ロボットが動いている間に経路上に**地図にはない障害物**が現れると、その障害物を回避できる経路を再度生成し直して移動を試みます。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

目的地までの経路が生成できない場合、Navigation Goal設定が失敗する場合があります。ロボットが目的地まで移動する途中でロボットを停止させたい場合は、ロボットの現在位置を目的地として再設定します。


## 課題（Gazebo編）

{% capture staff01 %}
1. Gazeboシミュレータ環境を起動し、SLAMの時に作った地図を指定してナビゲーションを実行してみなさい。
2. 最初に自己位置推定が不確かである状況を画面キャプチャして保存し、レポートファイルに添付せよ。画面キャプチャはUbuntuではgnome-screenshotを使うことができる。例えば保存するファイルをinitial.jpgにするときは以下のコマンドを実行して、マウスでキャプチャする範囲を指定する。
```bash
$ gnome-screenshot --area -f initial.jpg
```
3. ゴールを指定してナビゲーションを開始すると、移動経路がRViz上に示されること、移動している間に次第に自己位置推定が確からしくなる様子を確かめよ。このときの途中経過を画面キャプチャあるいは動画にして保存し、レポートファイルに添付せよ(レポートファイルには動画は直接張り込めないので、数枚の画像列にして貼ると良い)。動画を保存するのはsimplescreenrecoderを使うとできる（デスクトップ左端のアイコンにもある）。
4. Gazeboではシミュレーション空間中に新規のオブジェクトを置くことができる。図にあるボタンを操作して空間に配置し障害物としてみよ。地図には存在しない障害物をどのようにして避けるか確認しなさい。障害物をおいたGazeboの環境シーンを撮影しレポートファイルに添付せよ。またそれを避ける移動経路を生成して移動していく様子のRViz画面を保存しレポートファイルに添付せよ。

![](/assets/images/ritsumeikan/gazebo_object_insert.png)

{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

## 課題（実機編）

{% capture staff01 %}
1. 実機を教室内のコースに置き、SLAMの時に作った地図を指定してナビゲーションを実行してみなさい。ただしコースがSLAMの時と大きく変わっているとうまく動かない可能性がある。
2. 最初に自己位置推定が不確かである状況を画面キャプチャして保存し、レポートファイルに添付せよ。画面キャプチャはUbuntuではgnome-screenshotを使うことができる。
3. ゴールを指定してナビゲーションを開始すると、移動経路がRViz上に示されること、移動している間に次第に自己位置推定が確からしくなる様子を確かめよ。このときの途中経過を画面キャプチャあるいは動画にして保存し、レポートファイルに添付せよ(レポートファイルには動画は直接張り込めないので、数枚の画像列にして貼ると良い)。動画を保存するのはsimplescreenrecoderを使うとできる（デスクトップ左端のアイコンにもある）。
4. 実際にコース上に手や足、ブロックなどの地図には存在しない障害物を出現させ、ロボットがどのように避けるか確認しなさい。障害物のあるコースの様子を写真で撮影しレポートファイルに添付せよ。またそれを避ける移動経路を生成して移動していく様子のRViz画面を保存しレポートファイルに添付せよ。

{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


## （発展）チューニングガイド 
Navigation stackには複数のパラメータがあり、これらのパラメータの設定によって、それぞれ異なる形態のロボットに最適化されたナビゲーションを適用が可能となります。
ここでは重要な、または頻繁に使用されるパラメータの説明を行います。様々なロボットや環境に応じた、より詳細なNavigationチューニングに関しては[Basic Navigation Tuning Guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)([URL on Wayback Machine]({{ wayback_prefix }}https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide))を参照してください。
以下のパラメータは、costmapの計算に使用されるパラメータで、`turtlebot3_navigation/param/costmap_common_param_$(model).yaml`ファイルに保存され、プログラムを実行する際にロードされます。

### 目標位置付近で停止せず回転し続ける問題
ロボットに移動指示を出すと、目標位置付近まで
移動した後に停止せず小さい半径で回転し続ける場合があります。
目標位置付近で行われる細かい位置合わせにおいて、
経路計画上の想定動作と実際のロボットの挙動が一致していないために
起こる現象のようです。

目標位置付近で停止せず回転し続ける現象が起きた場合は下記の
コマンドを実行してください。
```bash
$ rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
```
現在の目標を取り消してロボットを停止させることができます。

### 経路計画のパラメータ変更
目標位置付近で回転し続ける現象が起こらないよう、
経路計画のパラメータを変更することができます。
但し、ロボット実機の挙動とGazeboによるシミュレーション環境での
ロボットの挙動は一部異なるため、それぞれの環境で適切な
パラメータが異なる点に注意してください。

デフォルトではGazebo環境に合うようパラメータを設定してあります。
両環境で変更するべきパラメータは以下の通りです。

| パラメータ名 | Gazebo環境用 | 実機環境用 | 参考情報 |
||(デフォルト)|||
|:-:|:-:|:-:|:-:|
| /move_base/DWAPlannerROS/max_vel_theta | 1.2 | 6.4 | [解説](https://wiki.ros.org/dwa_local_planner#line-463)([URL on Wayback Machine]({{ wayback_prefix }}https://wiki.ros.org/dwa_local_planner#line-463)) (旧称 max_rot_vel として記載)|
| /move_base/DWAPlannerROS/min_vel_theta | 1.0 | 4.6 | [解説](https://wiki.ros.org/dwa_local_planner#line-468)([URL on Wayback Machine]({{ wayback_prefix }}https://wiki.ros.org/dwa_local_planner#line-468)) (旧称 min_rot_vel として記載)|

これらのパラメータは `rqt_reconfigure` や `rosparam` コマンドを
使って変更できます。
但し `navigation.launch` を起動するとパラメータはリセット
されますので、 `navigation.launch` を起動した後で変更する
必要があります。

#### `rqt_reconfigure` でのパラメータ変更
1. 下記のコマンドで `rqt_reconfigure` を起動してください。
   ```bash
   $ rosrun rqt_reconfigure rqt_reconfigure &
   ```
1. 左のペインで `move_base` を展開し、
   その下にある `DWAPlannerROS` を選択してください。
1. 右のペインにパラメータ一覧が表示されるので
   その中の `max_vel_theta`, `min_vel_theta` の値を
   それぞれ設定してください。

#### `rosparam` コマンドでのパラメータ変更
下記のようなコマンドでパラメータを設定してください
(実機環境用の値を設定する例)。
```bash
$ rosparam set /move_base/DWAPlannerROS/max_vel_theta 6.4
$ rosparam set /move_base/DWAPlannerROS/min_vel_theta 4.6
```
現在のパラメータの値は下記のコマンドで確認できます。
```bash
$ rosparam get /move_base/DWAPlannerROS/max_vel_theta
$ rosparam get /move_base/DWAPlannerROS/min_vel_theta
```


### Costmap関連の主なパラメータ

**inflation_radius**  
地図上の障害物から設定された値分の空間を取り、ロボットが移動する経路を生成する際に最小限の安全距離を維持するために使用されます。この値をロボットの半径よりも大きい値に設定することで、ロボットと障害物の衝突を避けることが可能となります。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_inflation_radius.png)


**cost_scaling_factor** 
cost_scaling_factorは、以下のようにマイナスの指数値を利用して係数因子を生成するため、cost_scaling_factorの値を上げるほど計算されたcost値は小さくなります。  
costmap_2d::INSCRIBED_INFLATED_OBSTACLEの値は、254に定義されています。

- exp{-1.0 * cost_scaling_factor * (distance_from_obstacle - inscribed_radius)} * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_cost_scaling_factor.png)


### AMCL関連の主なパラメータ

**min_particles, max_particles**  
AMCLノードで使用するparticleの数を調整します。粒子の数が多くなると精度が高くなるものの、計算量が増加するため演算速度が低下します。

**initial_pose_x, initial_pose_y, initial_pose_a**  
ロボットの初期位置と方向を設定します。Rvizの`2D Pose Estimate`ボタンで、ロボットのPoseを手動で設定できます。

**update_min_d, update_min_a**  
パーティクルフィルタをアップデートするのに必要な最小並進距離(m)と最小回転角度(rad)を設定することができます。値が小さいほどアップデートが頻繁に行われますが、計算量が増加します。

以下のパラメータは、DWAの計算に使用されるパラメータで、`turtlebot3_navigation/param/dwa_local_planner_param_$(model).yaml`ファイルに保存され、プログラムを実行する際にロードされます。

### DWA関連の主なパラメータ
DWA関連のパラメータをロボットのハードウェアに合わせた適切な値に設定していない場合、ロボットが正常に動作しない可能性があります。

**acc_lim_x、acc_lim_y、acc_lim_th**  
ロボットのx、y、&theta;方向に、加速度の限界値をm/s^2とrad/s^2単位で設定します。

**max_trans_vel、min_trans_vel**  
ロボットの並進速度の最大値および最小値を絶対値で表します。単位はm/sです。

**max_vel_x、min_vel_x、max_vel_y、min_vel_y**  
ロボットがx、y方向に移動できる最大値、最小値をm/sで設定します。TurtleBot3の場合、y方向への移動が不可能なため、yの最大、最小値は0に設定されます。

**max_rot_vel、min_rot_vel**  
ロボットの最大、最小回転速度をrad/sで設定します。

**xy_goal_tolerance**  
ロボットが指定された目的地に到着した際の、x、y座標上の距離の誤差(許容誤差)を設定します。
 
**yaw_goal_tolerance**  
ロボットが指定された目的地に到着した際の、ロボットが向く角度の誤差(許容誤差)を設定します。
 
**sim_time**  
ロボットの現在の位置から数秒程度の予想経路をシミュレーションするかどうかを設定します。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_sim_time.png)

## Navigationチューニングのメモ書き（For TA）

Navigation時に狭いところを通れない、障害物に近づいて身動きできなくなる、などのトラブルが頻発することがある。
概ねlocal plannerとglobal planner、さらにCostmapのパラメータチューニングの問題と思われ、以下に試行錯誤のメモを載せておく。


### 2022/3/31における考察

壁際でロボットがスタックするのはglobal_plannerが作ったpath（rviz上での赤い線）の旋回半径よりも、dwa_local_plannerの作ったpath(rviz上での黄色い短い線）の旋回半径が大きすぎるため。
global planの軌道よりも大回りになってしまってオーバーステアになりカーブ外側にある壁に寄り添ってしまってそこで止まる。

ではなぜlocal plannerの出力する旋回半径が大きくなるか？
rosrun rqt_reconfigure rqt_reconfigure でparamを動的に変えながらチェックすると、min_vel_thetaが関係していそう。min_vel_thetaを「大きく」すると旋回半径が小さくなりやすくなり小回りが効くようになる。逆にmin_vel_thetaを小さくすると直線運動しかできなくなって旋回できないように見える。
max_vel_thetaは最大の旋回角速度を制御していてロボットハードの性能上限を記述しているが、min_vel_thetaはそもそも「最小の角速度」ではおかしい（０のはずだから）。そうではなく下のsimple_trajectory_generator.cppによれば、並進速度がmin_vel_transに指定された値より大きいか、旋回速度がmin_vel_thetaに指定された値より大きいか、どちらかが成立するようにplanを生成する、という条件設定がされているように見える。
これの意味はおそらく、移動中にゴール未達なのに停止してしまうplanが生成されないように、並進と旋回のどちらかがminで指定した値より大きい（つまり少なくとも並進か旋回のどちらかが行われている）状態を維持し続ける、という意図ではないか。

https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/base_local_planner/src/simple_trajectory_generator.cpp#L193

とすれば、min_vel_thetaをある程度大きい値にしておけば並進速度が小さくなっている状況では旋回速度を大きくとらねばならず、結果として旋回半径が小さくなり小回りが生成されやすくなるはず。

ちなみに、ROSのmelodicとkineticではmin_vel_theta、min_rot_velと変数名が違うので注意が必要。
実験用のブート環境ではmelodicの上にkinetic用のROBOTISパッケージが載っているので、どちらで反応するか、rqt_reconfigureを見てみないといけない。

---
### 2022/05/17のgazeboによる実験

Global costmapのパラメータであるinflation_radiusの値（現在0.25になっている）が小さすぎるので、コストマップが障害物から離れたところで急激にコストが下がるのでぎりぎりを狙ったplanができてしまう模様。元々はもう少し大きかったはず。 inflation_radiusの値が2.0程度の方がgazeboのgazebo_manipulator_world.launch（６角形）ではいい感じ見える。
gazeboであまりにもmin_vel_thetaを大きくしすぎると振動しだして不安定になる。inflation_radiusの値でコントロールできるならその方がいい。
min_vel_theta=1.7
max_vel_theta=2.0
acc_lim_theta=2.0程度で検証。 ->　inflation_radiusはlocal costmapには影響せず、global costmapにのみ関わる模様。2.0ではかなり大きくそれで小さく0.25にしていた経緯。なかなかいいパラメータがない。

DWAPlannerROSのパスプラニングのパラメータであるpath_distance_bias、goal_distance_bias、occdist_scaleをあまりいじらない方が良さそう。occdist_scaleを大きくすると障害物から離れた経路を作ろうとするが、十分空いているのにそこを通過しないでスタックしてしまうことがgazeboではよく起きた。上のinflation_ radiusを調整することでうまくできればその方が良い。

---
### 2023/05/17 Gazebo上のnavigation実験時のメモ

dynamic reconfigureのツールはrqt_reconfigureより，rqt_guiのようがよさk．
% rosrun rqt_gui rqt_gui

rqt_gui上でmove_baseの中のglobal_costmapとlocal_costmapをえらぶと（その下にみえるinflation_layerとかを選んではいけない．local_costmapのセレクタを選ぶ）以下のパラメータが見える．
footprint : ホームベース上のロボット形状を頂点リストで表現．これからロボットの大きさが計算される．
robot_radius : footprintが設定されてない状況で，ロボットの大きさ（半径）を表現する．
footprint_padding : footprintに対してさらに膨らませる半径を指定．
以上のパラメータを大きくすると，global costmap もしくはlocal costmapの水色の領域，つまりロボットの中心が侵入してはいけない領域（侵入すると障害物に接触する）の広さを調整できる．これを大きめにしておくとロボットが大回りしてくれることが期待できる．
ただし，min_vel_theta（平心速度と旋回速度の総和の最小値，低速だとできるだけ小さく旋回しようとする）の大きさが小さいと軌道より小回りしてしまって水色領域に入ってしまうことが観測される．調整できるかもしれない．
