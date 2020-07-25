---
layout: splash
lang: en
ref: phyexp3_navigation
permalink: /docs/navigation/
sidebar:
  title: 生成した環境地図に基づくナビゲーション
  nav: "phyexp3_navigation"
---

# [Class 5] 生成した環境地図に基づくナビゲーション
Navigationは、特定環境である位置で指定された対象へロボットを移動させるものです。そのために与えられた環境にある障害物や壁などのジオメトレック情報が含まれたマップが必要です。以前のSLAM部分で説明した通り、マップはセンサが獲得した距離情報やロボットのポーズ情報で作成されました。
Navigationを使用すると、マップ、ロボットエンコーダ、IMUセンサ及び距離センサを使用してロボットが現在位置からマップの指定された目標位置に移動することができます。

より良いNavigationのためには、costmapの計算が必要です。costmapはロボットの位置と方向を表すpose情報とセンサ値、障害物情報、SLAMによって得られた地図に基づいて計算することができます。計算された結果は、ロボットが衝突する領域、衝突可能性がある領域、自由に移動可能な領域などで表され、これらの計算結果をもとに、ロボットの安全な移動経路の生成を助けます。costmapは2種類に分類することができ、ロボットのスタート位置から目的地までの経路計画を策定するために使用されるglobal costmapと、ロボットの周辺において障害物などを回避するための安全な経路を計算するlocal costmapがあります。ROSでこのようなcostmapは値で表されますが、0はロボットが自由に移動できる領域で、255に近づくほどロボットの衝突が発生する可能性の高い領域として区分されます。

  - 0 : 自由領域
  - 1 ~ 127 : 低い衝突可能性のある領域
  - 128 ~ 252 : 高い衝突可能性のある領域
  - 253 ~ 254 : 衝突領域
  - 255 : ロボットが移動できない領域

![](http://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png)

> [http://wiki.ros.org/costmap_2d#Inflation](http://wiki.ros.org/costmap_2d#Inflation)

DWA(Dynamic Window Approach)は、障害物を回避する経路の生成に使用される代表的な方法です。DWAは速度空間内で並進速度(v)と回転速度(ω)を使用し、ロボットが移動できる経路を予測・計算し、ハードウェアが出しうるロボットの最大速度が限界値として定められます。dwa_local_plannerでは、ロボットが目的地に到着できるよう進む経路であるglobal planと、障害物の回避に使われるglobal costmapによってロボットに移動可能な速度コマンドを計算し、local planを作成します。DWAは以下の順序で動作します。

1. ロボットの制御位置(dx、dy、d&theta;)を個別に生成します。
2. 生成されたそれぞれのサンプルを、ロボットの現在地からシミュレーションし、短時間サンプルが適用される場合、ロボットの位置を予測します。
3. シミュレーションの結果を点数に換算します。このとき、障害物との距離、目的地までの距離、global planとの距離、速度などの要素が考慮され、物体と衝突するサンプルは結果の計算から排除されます。
4. 最も高いスコアを獲得したサンプルが選択され、ロボットに伝達されます。
5. 1 ~ 4のプロセスを繰り返します。

## ナビゲーションノードの実行 
### [Remote PC] roscoreを実行してください。
  ```bash
  $ roscore
  ```
### [TurtleBot3 SBC] TurtleBot3 アプリケーションを起動するための基本パッケージを起動します。
  ```bash
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```
  {% capture capture01 %}
  **roslaunch turtlebot3_bringup turtlebot3_robot.launch**
  1. turtlebot3_core.launch
      - subscribe : cmd_vel
      - publish : joint_states, odom

  2. turtlebot3_lidar.launch
      - publish : scan

  turtlebot3_robot.launchファイルを実行すると、turtlebot3_core.launchとturtlebot3_lidar.launchファイルが実行され、TurtleBot3の状態をチェックするノード(node)であるturtlebot3_diagnosticsが生成され、TurtleBot3の各種センサやハードウェアの状態についての情報をpublishします。turtlebot3_core.launchファイルでは、OpenCRと通信してjoint_states、odomをpublishし、cmd_velをsubscribeするノードが生成されます。turtlebot3_lidar.launchファイルでは、LIDARを作動させ、センサーから得られたscanデータをpublishするノード(node)が生成されます。
  {% endcapture %}
  <div class="notice--success">{{ capture01 | markdownify }}</div>

### [Remote PC] Navigationファイルを実行してください。
  当コマンドを行う前にTurtlebot3のモデル名を指定しなければなりません。$ {TB3_MODEL}は、burger、waffle、waffle_piの中で使用するモデル名を指定してください。exportの設定を永続化するためには、Export Turtlebot3_MODELのページを参照してください。

  ```bash
  $ export TURTLEBOT3_MODEL=${TB3_MODEL}
  $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
  ```

{% capture capture02 %}
**roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml**
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

4. move_base
  - subscribe : goal, cancel
  - publish : feedback, status, result, cmd_vel
  - move_baseパッケージのmove_baseノードは、ロボットのNavigation stackにアクセスするROSインターフェイスを提供します。move_baseノードは、global plannerとlocal plannerを接続してロボットを目的地まで移動させ、この時それぞれのplannerに合ったcostmapも保管します。ロボットの目的地(goal)をAction形態のトピックで受信すると、現在地(feedback)と状態(status)、移動の結果(result)をアップデートするため、同様にAction形態のトピックを使用します。また、現在の状態に合わせてロボットを動かすためのcmd_velトピックが持続的にpublishされます。

5. rviz
  - subscribe : tf, odom, map, scan
  - 最後にrvizの設定ファイルを適用したrvizが実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成されたマップを視覚化します。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

  上記の命令を実行すると、ビジュアル化ツールRvizが実行されます。Rvizを別途実行するためには、下記の命令のコマンドをを使用してください。
  ```bash
  $ rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
  ```

## ロボットの初期姿勢を設定する

Navigationを行う際に最も重要なことは、ロボットの正確な現在地を地図上に示すことです。ロボットのエンコーダとIMU、LDSなど各種センサから得られた情報をもとにTurtleBot3の位置を予測するには、確率をベースにしたAMCL(Adaptive Monte Carlo Localization)というparticle filterが使用されます。AMCLは、センサ情報に基づいてロボットの位置を予測します。また、アルゴリズムのパラメータに設定された移動量をロボットが移動するたびに、予測位置の値を更新し、更新が繰り返されるたびに予測位置の誤差が減少します。

[Remote PC]ロボットがNavigationを実行するための地図が実行され、ロボットのLDSセンサが正常に実行されれば、Rvizの`2D Pose Estimate`ボタンを使用し、地図上のロボットの位置を実際のロボットの位置と方向に合わせて設定します。`2D Pose Estimate`ボタンをクリックして、ロボットが実際に位置している点をRvizの地図上でクリックし、ロボットが向いている方向にマウスをドラッグすると、大きな緑色の矢印の方向をロボットが向いている方向に設定することができます。LDSセンーの値と地図で表示された障害物の位置が一致するほど、正確なNavigationが可能なため、初期位置を設定するための手順を繰り返し実行し、なるべく正確な位置に設定する必要があります。以下の手順で、ロボットの位置を設定します。

  1. Rvizで`2D Pose Estimate`ボタンをクリックします
  2. 地図上でロボットが実際に位置する点をクリックし、ロボットの前面が向いている方向に矢印をドラッグします。

これが完了すると、ロボットは緑の矢印で指定された位置と方向を初期ポーズとして使用し、実際の位置と方向を推定します。緑の矢印はタートルボット３の予想位置を表示します。レーザースキャナーはマップに大まかな壁を表示します。図に数字が誤って表示されていないことが確認できたら、2D Pose EstimateボタンをクリックしてTurtleBot３をローカリゼーションします。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_pose_estimate.png)

> 上の図で、ロボットの周辺に広く表示された多数の小さな緑の矢印は、ロボットの初期位置を指定した際にロボットのセンサーと各種情報を介してAMCLが計算したロボットの現在位置です。ロボットの移動に従って分布していた現在地の予想値がロボットに向かって近づき、収束する様子が分かります。

> **注意**：Navigationを実行する前にturtlebot3_teleop_keyboardノードが実行されている場合は、このノードを必ず終了させる必要があります。turtlebot3_teleop_keyboardノードでpublishされるcmd_velデータがNavigationのcmd_velデータと衝突し、ロボットが正常に動かない可能性があります。

## Navigation Goalを設定する
[Remote PC] Rvizで地図が表示され、ロボットの初期姿勢が正しく設定されると、ロボットが到着目的地の位置と方向を指定することができます。Rvizの`2D Nav Goal`ボタンをクリックし、ロボットが移動できる目的地をクリックし、ロボットが向いている方向をドラッグして、矢印の方向を指定します。

1. Rviz上部の `2D Nav Goal`ボタンをクリックします。
2. 地図上でロボットが移動する目的地をクリックし、ロボットが向いている方向をドラッグして、目的地設定を完了します。

目的地設定が完了すると、Navigationアルゴリズムはロボットの現在位置から目的地までの経路を生成し、経路を辿れるようにロボットに命令を出します。ロボットが動いている間に経路上に障害物が表示されると、障害物を回避できる経路を生成して移動を行います。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

目的地までの経路が生成できない場合、Navigation Goal設定が失敗する場合があります。ロボットが目的地まで移動する途中でロボットを停止させたい場合は、ロボットの現在位置を目的地として再設定する方法をとることができます。

## チューニングガイド 
Navigation stackには複数のパラメータがあり、これらのパラメータの設定によって、それぞれ異なる形態のロボットに最適化されたNavigationを適用が可能となります。
ここでは重要な、または頻繁に使用されるパラメータの説明を行います。様々なロボットや環境に応じた、より詳細なNavigationチューニングに関しては[Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)を参照してください。
以下のパラメータは、costmapの計算に使用されるパラメータで、`turtlebot3_navigation/param/costmap_common_param_$(model).yaml`ファイルに保存され、プログラムを実行する際にロードされます。

### Costmap関連の主なパラメータ

#### inflation_radius
地図上の障害物から設定された値分の空間を取り、ロボットが移動する経路を生成する際に最小限の安全距離を維持するために使用されます。この値をロボットの半径よりも大きい値に設定することで、ロボットと障害物の衝突を避けることが可能となります。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_inflation_radius.png)


#### cost_scaling_factor 
cost_scaling_factorは、以下のようにマイナスの指数値を利用して係数因子を生成するため、cost_scaling_factorの値を上げるほど計算されたcost値は小さくなります。  
costmap_2d::INSCRIBED_INFLATED_OBSTACLEの値は、254に定義されています。

- exp{-1.0 * cost_scaling_factor * (distance_from_obstacle - inscribed_radius)} * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_cost_scaling_factor.png)


### AMCL関連の主なパラメータ

#### min_particles, max_particles
AMCLノードで使用するparticleの数を調整します。粒子の数が多くなると精度が高くなるものの、計算量が増加するため演算速度が低下します。

#### initial_pose_x, initial_pose_y, initial_pose_a
ロボットの初期位置と方向を設定します。Rvizの`2D Pose Estimate`ボタンで、ロボットのPoseを手動で設定できます。

#### update_min_d, update_min_a
パーティクルフィルタをアップデートするのに必要な最小並進距離(m)と最小回転角度(rad)を設定することができます。値が小さいほどアップデートが頻繁に行われますが、計算量が増加します。

以下のパラメータは、DWAの計算に使用されるパラメータで、`turtlebot3_navigation/param/dwa_local_planner_param_$(model).yaml`ファイルに保存され、プログラムを実行する際にロードされます。

### DWA関連の主なパラメータ
DWA関連のパラメータをロボットのハードウェアに合わせた適切な値に設定していない場合、ロボットが正常に動作しない可能性があります。

#### acc_lim_x、acc_lim_y、acc_lim_th
ロボットのx、y、&theta;方向に、加速度の限界値をm/s^2とrad/s^2単位で設定します。

#### max_trans_vel、min_trans_vel
ロボットの並進速度の最大値および最小値を絶対値で表します。単位はm/sです。

#### max_vel_x、min_vel_x、max_vel_y、min_vel_y
ロボットがx、y方向に移動できる最大値、最小値をm/sで設定します。TurtleBot3の場合、y方向への移動が不可能なため、yの最大、最小値は0に設定されます。

#### max_rot_vel、min_rot_vel
ロボットの最大、最小回転速度をrad/sで設定します。

### xy_goal_tolerance 
ロボットが指定された目的地に到着した際の、x、y座標上の距離の誤差(許容誤差)を設定します。
 
### yaw_goal_tolerance 
ロボットが指定された目的地に到着した際の、ロボットが向く角度の誤差(許容誤差)を設定します。
 
### sim_time 
ロボットの現在の位置から数秒程度の予想経路をシミュレーションするかどうかを設定します。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_sim_time.png)
