---
layout: splash
lang: en
ref: phyexp3_2-1
permalink: /docs/slam/
sidebar:
  title: 環境地図の自動マッピング(SLAM)
  nav: "phyexp3_2-1"
---

# [Class 4] 環境地図の自動マッピング(SLAM)

## LRFセンサ(LDS)
![](/assets/images/ritsumeikan/011.png)

TurtleBot3に使われているLDS(LASER Distance Sensor, あるいはLASER Range Finder: LRF)は、2次元平面の360度について障害物までの距離を測れるセンサで、後述の自動環境地図生成（SLAM）とナビゲーションに必要な環境情報を取得します。
センサの特性上、直射日光が強く当たる屋外環境では使用が困難で、10,000lux以下の明るさの屋内空間で走行するロボットに適しています。
実験に用いているTurtlebot3はOpenManipulater-Xという「腕」を搭載しているため、センサーに対して後方が腕に邪魔されるため測定値が環境状態を正しく反映しません。そのため、センサー情報を取得したあとフィルタを使ってその部分を除去します。
LDSセンサはUSBインターフェースであるUSB2LDSボードを通じ、Raspberry Pi3とUSBケーブルを介して接続されています。本実験ではすでに設定済なので意識する必要はありません。


### LDSセンサーノードの起動

本来はTurtlebot3のPC(Raspberry Pi)にSSHログインしてroslaunchする必要がありますが、本実験では前章のTurtlebot3移動制御の時と同じく、Remote PC（各自が使っているUbuntu Linux端末）からmachine.launchファイルをroslaunchコマンドで起動することで自動的にノードが使用可能になります。

以下はTurtleboの番号が"09"の時の起動コマンド。
```bash
$ roslaunch exp3 machine.launch id:=09
```

{% capture capture00 %}
**roslaunch hls_lfcd_lds_driver hlds_laser.launch**
machine.launchファイル内部で呼び出され、Turtlebot3内部で起動されるノード。
1. hlds_laser_publisher
    - publish : scan, rpms

hlds_laser.launchを実行するとTurtlebot3内部でhlds_laser_publisherノードが起動し、センサから取得される距離データと回転速度をそれぞれscanとrpmsのトピック名でpublishされます。 sensor_msgsタイプのLaserScanメッセージであるscanトピックには獲得したロボット周辺の物体との距離データが配列の形で配信されます(rostopic echoコマンドで閲覧してみるとよい）。
{% endcapture %}
<div class="notice--success">{{ capture00 | markdownify }}</div>

### RVizの実行
```bash
$ roslaunch exp3 rviz.launch
```
{% capture capture00 %}
**roslaunch exp3 rviz.launch**
2. rviz
    - subscribe : scan

rviz.launchを実行すると、rvizノードが起動し、Turtlebot3の現在の位置・姿勢や距離センサーの情報が可視化されたウィンドウ画面がディスプレイ上に開きます。  
RVizノードでは、rvizの設定ファイルを読み込み（カスタマイズして別ファイルに保存することもできる）、scanデータをsubscribeし3次元グラフィックスで描画します（画面上の赤い四角）。
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>



## SLAM

SLAM(Simultaneous Localization and Mapping)とは、未知の領域をロボットが探索し、ロボットに取り付けられたセンサを通じて取得された環境情報を用いて、ロボットが現在の自己位置を推定することと、環境地図を作成することを同時に行う技術です。SLAMは、ナビゲーション（目的地への誘導）や無人自動車の自律走行に欠かせない重要な要素です。外部情報を取得するために使用されるセンサには、距離を測定することができるセンサや周辺の画像を取得できるセンサがあります。赤外線距離センサ(IR)、音波センサ(SONAR)、レーザーセンサ(LRF)などがよく使用されており、最近では映像を解析するアルゴリズムの発達によってRGBカメラが数多く使用されています。

次の瞬間のロボットの位置を予測するため、ロボットの車輪と接続されたエンコーダの値を読み取り、推測航法(Dead Reckoning)を使用して走行距離(odometry)を計算することになりますが、この時、車輪と地面の間の摩擦などによって誤差が生じます。実際のロボットの位置と予測されたロボットの位置の誤差を減らすために、慣性測定センサ(IMU)から得られたデータを利用して位置補正を行うことができます。

距離センサ値を用いて位置の誤差を低減するために使用される方法として、Kalman filter、Markov Localization、Monte Carlo Localizationなどがあります。

SLAMでマップを作成する際、いくつかの注意点があります。

広い倉庫やホールのようにセンサの範囲で届かない広い領域の地図を作成しようとすると地図が正しく作成できません。これは、センサの範囲を両腕の長さであると仮定したとき、ホールの中心で目を閉じて両腕を使って現在地を探ろうとするのに似ています。手（センサーのレーザ）が届かないor反射してこない領域について情報が得られないためです。
これと類似して、角などの特徴のない長い廊下も地図の描画が困難です。特徴のない両壁面からなる長い廊下で目を閉じ、壁を手でつたって歩くと、廊下のどの位置まで来たのか把握しづらいことと同様です。

このような場合、マッピングアルゴリズムが特徴点として参照できるよう、地図上のあちこちに物体や障害物を設置することで解決できます。このような作為的な特徴は、一定のパターンや対称の形で置くよりも、パターンがない形で置くこともよいでしょう（注：理由を考えてみましょう）

一般的によく使用される**Gmapping**と**Catographer**には、地図を作成する方法に若干の違いがあります。mapをすぐに出力するGmappingとは異なり、Catographerはsubmapを出力し、submapを集めてmapを生成します。これによって、互いに繋がった広範囲の地図を作成する場合には、Catographerがより正確な形の地図を作る助けになります。実験ではGmappingを用います。

### SLAMを実行する(Gazeboシミュレータ)
全て[Remote PC]で実行します。
1. roscoreを実行します。  
  ```bash
  $ roscore
  ```
  <br>
  <br>
2. Gazeboを起動します。  
  ```bash
  $ roslaunch exp3 gazebo_manipulator_world.launch
  ```
  <br>
  <br>
3. Gazeboのシミュレーションスタートボタン（ウィンドウ左下の三角）を押す。**忘れないこと！忘れるとシミュレーションやSLAMがスタートしません**
![](/assets/images/ritsumeikan/gazebo_playbutton2.png)
  <br>
  <br>
4. SLAMを実行する。  
  ```bash
  $ roslaunch exp3 slam.launch
  ```
  RVizが自動的に開いて地図が一部だけ生成されている様子が見えます。現在のロボット位置から見て物体の影になっている領域はグレーになって地図が生成できていないことがわかります。
![](/assets/images/ritsumeikan/gazebo_slam_initial.png)
  <br>
  <br>
5. teleopでロボットを手動で移動させてみる。  
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
  キーボードのキーで自由にGazebo内のロボットを動かしながら、どのように地図が生成されていくか確認する。
  <br>
  <br>
6. 十分環境内でロボットを移動させて地図が出来上がったら、RVizやslamを動かしたままで新しい端末を開き、save_map.launchを起動して地図を保存します。
  ```bash
  $ roslaunch exp3 save_map.launch map_name:=map1
  ```
引数map_nameに地図の名前を入れるとその名前(上の実行例では"map1")で地図ファイル（.pgmと.yamlの２つのファイル）が`~/exp3_ws/src/exp3/map`ディレクトリに保存されます。roscdコマンドを使うと簡単にアクセスができます。roscdコマンドは指定したROSパッケージのディレクトリに直接移動することができます。
  ```bash
  $roscd exp3
  $ls
  CMakeLists.txt  README  include  launch  map  package.xml  rviz  src  world
  $ cd map
  $ ls
  a.pgm  a.yaml
  ```

{% capture capture06 %}
**roslaunch exp3 slam.launch**
1. **urdf**
  - Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
2. **robot_state_publisher**
  - robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtf(座標変換)の形式でpublishします。
  - subscribe : joint_states 
  - publish : tf
3. **laser_filterノード**
  - LDSセンサの有効ではない値の範囲をフィルタリングするノードを実行します。ここでは、OpenMANIPULATORが設置されている後方部の角度を無視します。
4. **turtlebot3_gmapping.launch**
  - Gmappingを利用したSLAMを実行するために必要なパラメータ情報がパラメータサーバにロードされます。この設定を利用してgmappingを設定します。
5. **turtlebot3_gmapping.rviz**
  - Gmappingを適用したSLAMをRviz画面に表示するために必要なRvizのデフォルト設定を適用し、Rvizを実行します。

slam.launchファイルを実行すると、ロボットの情報(urdfファイル）を指定した位置にロードします。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します（この実験では意識する必要はありません）。  
{% endcapture %}
<div class="notice--success">{{ capture06 | markdownify }}</div>

### 地図
ROSにおいて地図は2次元Occupancy Grid map(OGM)を主に使用します。保存されたmap.pgmイメージファイルを開くと、以下のようにロボットが移動できる白い領域と、障害物として識別されロボットが移動できない黒い領域、ロボットが探索していない灰色の領域に区分されます。このように生成されたマップは、次に紹介するNavigationで使用することができます。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/map.png)

### 課題
{% capture staff01 %}
1. Gazebo起動時にgazebo_manipulator_world.launchを起動して、SLAMを実行し地図を作成しなさい。作成できた地図画像(`~/exp3_ws/src/exp3/map`ディレクトリにある）をノートブックに添付せよ。添付するにはファイルビューアを開いて、ノートブックのmarkdownセルに画像(pgmファイル）のアイコンをドロップすればよい。
2. Gazebo用に他の環境シーンのデータを用意してある。launchファイルを変えると別の環境シーンがGazeboに読み込まれるので次のうちの１つを読み込んで、SLAMを起動して地図を作成しノートブックに添付せよ。
  -- gazebo_manipulator_house.launch
  -- gazebo_manipulator_stage_4.launch
  -- gazebo_manipulator_willowgarage.launch (この環境シーンは非常に広大なので、地図を生成するのは一部でよい)
3. ロボットの初期位置座標はroslaunchでGazeboを起動する時に引数で指定することができる。単位はメートルなのであまり大きくすると視野の外におかれてしまうので注意。デフォルトの座標値はそれぞれのlaunchファイルに記載してある（`~/exp3_ws/src/exp3/launch/gazebo`ディレクトリに置いてある）。
  ```bash
  $ roslaunch exp3 gazebo_manipulator_world.launch x_pos:=0.5 y_pos=-0.1 z_pos:=0.0 
  ```
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


### SLAMを実行する(Turtlebot3実機)

全てRemote PCで実行します。
1. roscoreを実行します。  
  ```bash
  $ roscore
  ```

2. TurtleBot3実機に接続してローカルノードを起動します。(ロボット番号"09"の場合)
  ```bash
  $ roslaunch exp3 machine.launch id:=09
  ```
3. SLAMを実行します。
  ```bash
  $ roslaunch exp3 slam.launch
  ```
  
4. teleopでロボットを手動で移動させてみる。
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
  キーボードのキーで自由にGazebo内のロボットを動かしながら、どのように地図が生成されていくか確認する。
  
5. 十分環境内でロボットを移動させて地図が出来上がったら、RVizやslamを動かしたままで新しい端末を開き、save_map.launchを起動して地図を保存します。
  ```bash
  $ roslaunch exp3 save_map.launch map_name:=map1
  ```
{% capture capture02 %}
**roslaunch exp3 slam.launch
1. **roslaunch turtlebot3_bringup turtlebot3_remote.launch**
  - urdf：Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
  - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
  - subscribe : joint_states 
  - publish : tf

    turtlebot3_remote.launchファイルを実行すると、ロボットのurdfモデルを指定した位置に読み込みます。また、joint_statesとurdfを利用して、tf（座標変換）をpublishするrobot_state_publisherノードを生成します。  
    turtlebot3_slam.launchファイル内部にturtlebot3_remote.launchが含まれているのでturtlebot3_slam.launchが実行されると自動的にturtlebot3_remote.launchが最初に実行されます｡

2. **turtlebot3_gmapping.launch**
  - subscribe : scan, tf
  - publish : map, map_metadata

    地図生成アルゴリズムgmappingの実行ファイルturtlebot3_gmapping.launchが実行されます。このファイルの内部では、再びLDS設定が保存されたtu​​rtlebot3_lds_2d.luaを呼び出し、gmappingの使用に  必要な各種パラメータを定義して、gmappingパッケージのslam_gmappingノードを実行します。 slam_gmappingノードが生成されると、scanとtfトピックをsubscribeして、マップの生成に必要なmap_metadataとmapをpublishします。

3. **rviz**
  - subscribe : tf, scan, map

    最後にrvizの設定ファイルを適用したrvizが実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成されたマップを視覚化します。  
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

  **参考**  
  SLAMに限らずROSのモジュール連携では関係するコンピュータノードの時刻合わせが大変重要です。時刻が100msecほどズレているだけで正常に動作できないことがあります。時刻が合っていない時にはntpdateコマンドなどでNTPサーバに問い合わせて時刻合わせをすることができます。本実験ではNTPサービスをOS起動時に起動しているので意識する必要はないはずです。  
  ```bash
  $ sudo ntpdate ntp.ritsumei.ac.jp
  ```
{: .notice}

### 課題
{% capture staff01 %}
1. 教室内にスチロールブロックを使って作ったコースがいくつか設置してある。周りと相談して適宜変更して良いので、自分独自のコースアレンジをしてみよう。アレンジしたコースの写真を撮ってノートブックに添付せよ。
2. アレンジしたコースにTurtlebot実機を置いて、上の要領でSLAMを実行しコースの環境地図を作成しなさい。作成できた地図画像(`~/exp3_ws/src/exp3/map`ディレクトリにある）をノートブックに添付せよ。添付するにはファイルビューアを開いて、ノートブックのmarkdownセルに画像(pgmファイル）のアイコンをドロップすればよい。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

### （参考）アーム（OpenManipulator-X）搭載時の制限

OpenMANIPULATOR-Xを組み付けたTurtleBot3のSLAMでは、ロボットアームがLDSセンサーの一定部分を塞いでいるため、SLAMに使用されるLDSセンサーの範囲を制限することによって地図作成を行います。
以下の通り`scan_data_filter.yaml`設定ファイル（本実験ではこのファイルをとくに触る必要はありません）の記述によってLDSセンサーの設定された角度範囲データをフィルタリングすることによって、
有効でない角度値を取り除いて地図を生成します。

```
scan_filter_chain:

- name: Remove 120 to 240 degree
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: 2.0944
    upper_angle: 4.18879
```

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_slam.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/slam_running_for_mapping.png)


### (発展)Gmappingのチューニングガイド

Gmappingは、さまざまな環境に最適化されたパフォーマンスを実現するために、複数のパラメータの設定が可能です。Gmappingは一般的に別途設定なしに使用が可能であり、デフォルト設定で使用する場合が多いものです。設定可能なパラメータのリストは、[ROS wikiのGmappingパラメータ](http://wiki.ros.org/gmapping#Parameters)ページを参照してください。

以下のパラメータは、`turtlebot3_slam/launch/turtlebot3_gmapping.launch`ファイルに定義されており、ファイルを実行する際にroscoreのパラメータサーバーにロードされ、gmappingをベースにしたSLAMに適用されます。

**maxUrange**  
このパラメータはLDSセンサの最大使用可能範囲を設定します。 
 
**map_update_interval**  
マップをアップデートする期間(秒単位)この値が低いと、マップがもっと頻繁にアップデートされます。しかし、より大きな計算負荷が必要です。環境に基づいてこのパラメータを設定してください。 

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/tuning_map_update_interval.png)

**minimumScore**  
センサのscanデータ一致検査の成功および失敗を決定する最小点数値を設定します。広い空間でロボットの予想位置に生じる誤差を減少させることが可能です。適切に設定された場合、以下のような情報を見ることができます。 

```
Average Scan Matching Score=278.965
neff= 100
Registering Scans:Done
update frame 6
update ld=2.95935e-05 ad=0.000302522
Laser Pose= -0.0320253 -5.36882e-06 -3.14142
```
この値が高すぎると、下記の警告を見ることができます。 
```
Scan Matching Failed, using odometry. Likelihood=0
lp:-0.0306155 5.75314e-06 -3.14151
op:-0.0306156 5.90277e-06 -3.14151
```

**linearUpdate**  
ロボットがこの値よりも長い距離を並進運動したとき、scanプロセスを実行します。
 
**angularUpdate**  
ロボットがこの値よりも大きい角度を回転運動したとき、scanプロセスを実行します。
これをlinearUpdateより小さく設定することがいいです。 



