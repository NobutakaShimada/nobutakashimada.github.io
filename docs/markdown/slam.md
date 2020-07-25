---
layout: splash
lang: en
ref: phyexp3_2-1
permalink: /docs/slam/
sidebar:
  title: ３コマ目
  nav: "phyexp3_2-1"
---

# [Class 3] LRF(LDS)センサ

TurtleBot3に使われているLDS(LASER Distance Sensor)は、2次元平面の360度距離値を読み取ることができるセンサで、SLAMとNavigationに必要な重要情報を提供します。
LDSセンサはUSBインターフェースであるUSB2LDSボードを通じ、Raspberry Pi3とUSBケーブルを介して接続されますが、UARTインタフェースを介してOpenCRとも直接接続することができます。

センサの特性上、直射日光が強く当たる屋外環境では使用が困難で、10,000lux以下の明るさの屋内空間で走行するロボットに適しています。

TurtleBot3のLDSセンサは、パッケージの形でインストールするか、ソースコードをダウンロードしてビルドすることができます。

## センサパッケージをインストール
```bash
$ sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
```
TurtleBot3のLDSセンサを駆動するのに必要なドライバパッケージをインストールします。

### センサの接続ポート権限を設定
```bash
$ sudo chmod a+rw /dev/ttyUSB0
```
LinuxがインストールされたPCと接続する場合は、USBポートの権限を正しく設定することで、割り当てられたポートにアクセスすることができます。  
上のコマンドは、USB0番ポートに読み出し/書き込みアクセス権を設定します。

### hlds_laser_publisherノードを実行
```bash
$ roslaunch hls_lfcd_lds_driver hlds_laser.launch
```
{% capture capture00 %}
**roslaunch hls_lfcd_lds_driver hlds_laser.launch**
1. hlds_laser_publisher
    - publish : scan, rpms

hlds_laser.launchを実行するとhlds_laser_publisherノードが生成され、センサのデータと回転速度をそれぞれscanとrpmsのtopicでpublishします。 sensor_msgsタイプのLaserScanメッセージであるscanにはLDSセンサが回転し、獲得したロボット周辺の物体との距離データが配列の形で蓄積、保存されます。
{% endcapture %}
<div class="notice--success">{{ capture00 | markdownify }}</div>

### RVizとhlds_laser_publisherノードの実行
```bash
$ roslaunch hls_lfcd_lds_driver view_hlds_laser.launch
```

{% capture capture01 %}
**roslaunch hls_lfcd_lds_driver view_hlds_laser.launch**
1. hlds_laser_publisher
    - publish : scan, rpms

2. rviz
    - subscribe : scan

view_hlds_laser.launchを実行すると、hlds_laser.launchファイルとrvizノードが実行されます。  
hlds_laser.launchを実行するとhlds_laser_publisherノードが生成され、センサのデータと回転速度をそれぞれscanとrpmsのtopicでpublishします。 sensor_msgsタイプのLaserScanメッセージであるscanにはLDSセンサが回転し、獲得したロボット周辺の物体との距離データが配列の形で蓄積、保存されます。  
RVizノードでは、rvizの設定ファイルを読み込み、画面に表示してscanデータをsubscribeし、3次元グラフィックスで視覚化します。
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>

## センサのソースコードをダウンロード

ダウンロードされたLDS-01をサポートするドライバは、Windows、Linux、MacOSの開発環境で実行することができます。  
ソフトウェア条件は以下の通りです。

- GCC(Linux, MacOS使用時), MinGW(Windows使用時)
- Boost library(v1.66.0でテスト済み)

1. 以下のGitHubアドレスからソースコードをダウンロードします。
  ```bash
  $ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
  ```
  または、以下のサイトで緑の`Clone or download`ボタンをクリックし、ソースコードをダウンロードすることも可能です。

  - [https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)

2. 開発環境に必要なソフトウェアとライブラリをインストールします。
  - GCC(Linux, MacOS)またはMinGW(Windows)
  - Boost library

### ソースコードをビルド
以下のコマンドは、Linux環境に合うように設定されたmakefileを利用して、ソースコードをビルドします。WindowsとMacOS環境の場合、makefileを適切なものに修正してください。

```bash
$ cd hls_lfcd_lds_driver/applications/lds_driver/
$ make
```

### CLI環境で実行
ソースコードが正しくビルドされると、以下のように `./lds_driver`ファイルを実行してLDSセンサ値を確認することができます。

```bash
$ ./lds_driver
```
```
r[359]=0.438000,r[358]=0.385000,r[357]=0.379000,...
```

### GUI環境で実行
LDSセンサの値を視覚的に確認するためには、Qt CreatorとQt Libsを追加でインストールする必要があります。
- Qt Creator(v4.5.0でテスト済み)
- Qt Libs(v5.10.0でテスト済み)

1. 以下のQtサイトでOpen Source版をインストールしてください。
  - [https://www.qt.io/download](https://www.qt.io/download)

2. Qt Creatorを実行します。
3. ソースコードから、以下の位置にある`lds_polar_graph.pro`ファイルを開きます。  
  (hls_lfcd_lds_driver/applications/lds_polar_graph/lds_polar_graph.pro)
4. ソースコードでセンサと接続されたポートを確認し、他のポートに割り当てられている場合、該当するポート番号に変更します。
5. `CTRL` + `SHIFT` + `B`を押して、ソースコードをビルドします。
6. ビルドが正常に完了すると、`CTRL` + `R`を押してプログラムを実行します。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/appendix_lds/lds_gui.png)

### Embeddedボードで実行

LDS-01センサは、OpenCRまたはArduinoボードで動作させることが可能です。
この場合、センサデータの視覚化のため、LCDパネルが必要です。

LDS-01センサのTX、RXケーブルはembeddedボードのUARTピンと互換性があり、embeddedボードに電源とTX、RXピンを接続して使用することができます。  
実際のケーブルの色は下の図と異なる場合があるため、必ず製品のデータシートを参照してください。

![](/assets/images/ritsumeikan/lds_lines.png)

#### OpenCRからLDSセンサを読み込む
OpenCRボードからセンサの値を読み取るには、Arduinoの例題をOpenCRボードにアップロードする必要があります。

1. Arduino IDEのTools > Board > Boards ManagerでOpenCRを検索し、ライブラリをインストールします。
2. Tools > BoardからOpenCRを選択します。
3. Tools > PortでOpenCRが接続されたポートを選択します。
4. File > Examples > OpenCR > Etc > LDS > drawLDS例題を選択してOpenCRにアップロードします。

例題のアップロードが完了すると、OpenCRと接続されたLCDに、以下の画像のようにセンサの値が視覚的に確認できるようになります。

![](/assets/images/ritsumeikan/011.png)

# SLAM

SLAM(Simultaneous Localization and Mapping)とは、未知の領域を探索し、ロボットに取り付けられたセンサを通じて取得された情報を使用して、ロボットが自己位置推定及び環境地図の作成を同時に行うことを意味します。SLAMは、Navigationや無人自動車の自律走行に欠かせない重要な技術です。外部情報を取得するために使用されるセンサには、距離を測定することができるセンサや周辺の画像を取得できるセンサがあります。赤外線距離センサ(IR)、音波センサ(SONAR)、レーザーセンサ(LRF)などがよく使用されており、最近では映像を解析するアルゴリズムの発展によって、カメラが数多く使用されてもいます。  
ロボットの位置を予測するため、ロボットの車輪と接続されたエンコーダの値を読み取り、推測航法(Dead Reckoning)を使用して走行距離(odometry)を計算することになりますが、この時、車輪と地面の間の摩擦などによって誤差が生じます。実際のロボットの位置と予測されたロボットの位置の誤差を減らすために、慣性測定センサ(IMU)から得られたデータを利用して位置補正を行うことができます。  
また、距離センサ値を用いて位置の誤差を低減するために使用される方法として、Kalman filter、Markov Localization、Monte Carlo Localizationなどがあります。

SLAMでマップを作成する際、いくつかの注意点があります。  
広い倉庫やホールのようにセンサの範囲で届かない広い領域の地図を作成する場合、地図が正しく描画されません。これは、センサの範囲を両腕の長さであると仮定したとき、ホールの中心で目を閉じて両腕を使って現在地を探ろうとすることと同じです。  
これと類似して、特徴点のない長い廊下も地図の描画が困難です。特徴のない両壁面からなる長い廊下で目を閉じ、壁を手でつたって歩くと、廊下のどの位置まで来たのか把握しづらいことと同様です。

このような場合、マッピングアルゴリズムが特徴点として参照できるよう、地図上のあちこちに物体や障害物を設置する方法を検討できます。このような特徴点は、一定のパターンや対称の形で置くよりも、パターンがない形で置くことが最もよいでしょう。

一般的によく使用されるGmappingとCatographerには、地図を作成する方法に若干の違いがあります。mapをすぐにpublishするGmappingとは異なり、Catographerはsubmapをpublishし、submapを集めてmapを生成します。これによって、互いに繋がった広範囲の地図を作成する場合には、Catographerがより正確な形の地図を作る助けになります。


## SLAMを実行する
1. [Remote PC] roscoreを実行します。  
  ```bash
  $ roscore
  ```

2. [Turtlebot PC] TurtleBot3駆動のための基本パッケージを実行します。  
  ```bash
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```

3. [Remote PC] 新たにターミナルウィンドウを開き、SLAMを実行します。以下のコマンドで`${TB3_MODEL}`をTurtleBot3のモデル名のいずれかに変える必要があります。使用中のロボットに合わせて `burger`、`waffle`、`waffle_pi`の中から選択することができます。
  ```bash
  $ export TURTLEBOT3_MODEL=${TB3_MODEL}
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
{% capture capture02 %}
**roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping**
1. **roslaunch turtlebot3_bringup turtlebot3_remote.launch**
  - urdf：Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
  - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
  - subscribe : joint_states 
  - publish : tf

    turtlebot3_remote.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
    turtlebot3_slam.launchファイル内部にturtlebot3_remote.launchが含まれているのでturtlebot3_slam.launchが実行されると自動的にturtlebot3_remote.launchが最初に実行されます｡

2. **turtlebot3_gmapping.launch**
  - subscribe : scan, tf
  - publish : map, map_metadata

    実行文の末尾にslam_methods:=gmappingというオプションを入力したため、gmappingの実行に関連するファイルであるturtlebot3_gmapping.launchが実行されます。このファイルの内部では、再びLDS設定が保存されたtu​​rtlebot3_lds_2d.luaを呼び出し、gmappingの使用に必要な各種パラメータを定義して、gmappingパッケージのslam_gmappingノードを実行します。 slam_gmappingノードが生成されると、scanとtfトピックをsubscribeして、マップの生成に必要なmap_metadataとmapをpublishします。

3. **rviz**
  - subscribe : tf, scan, map

    最後にrvizの設定ファイルを適用したrvizが実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成されたマップを視覚化します。  
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

**参照**  
上のコマンドを実行した際SLAMがRviz画面上で正常に実行されない場合、Remote PCとTurtleBot3 SBCで以下のコマンドを実行してシステムクロックを同期化してください。  
**$ sudo ntpdate ntp.ubuntu.com**
{: .notice}

### TurtleBot3は、様々な方式のSLAMをサポートしています。
Gmapping、cartographerといった複数のSLAMをサポートし、コマンド上で`slam_methods`のパラメータ値として直接入力することにより、選択することができます。
入力可能なオプションには、`gmapping`、`cartographer`、`hector`、`karto`、`frontier_exploration`があります。

例としては、Gmappingの代わりにGoogle社のcartographerをSLAMとして使用する場合は、以下のようにslam_methodsの引数値でcartographerを伝達しSLAMノードを実行することもできます。 

```bash
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
```

{% capture capture03 %}
**roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer**
1. roslaunch turtlebot3_bringup turtlebot3_remote.launch
  - urdf：Unified Robot Description Formatの略で、ロボットの構成と接続形態を表すXML形式のファイルです。
  - robot_state_publisher : robot_state_publisherでは、ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
  - subscribe : joint_states 
  - publish : tf

    turtlebot3_remote.launchファイルを実行すると、ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
    turtlebot3_slam.launchファイル内部にturtlebot3_remote.launchが含まれているのでturtlebot3_slam.launchが実行されると自動的にturtlebot3_remote.launchが最初に実行されます｡

2. turtlebot3_cartographer.launch
  - subscribe : scan, imu, odom
  - publish : submap_list, map
  
    実行文の末尾にslam_methods:=cartographerというオプションを入力したため、cartographerの実行に関連するファイルであるturtlebot3_cartographer.launchが実行されます。このファイルの内部では、再びLDSの設定が保存されたtu​​rtlebot3_lds_2d.luaを呼び出し、cartographerの使用に必要な各種パラメータを定義して、cartographer_rosパッケージのcartographer_nodeノードを実行し、scan、imu、odomなどのtopicをsubscribeしてsubmap_listをpublishします。また、cartographer_rosパッケージのcartographer_occupancy_grid_nodeノードを実行し、cartographer_nodeでpublishされたsubmap_listをsubscribeしてmapをpublishします。

3. rviz
  - subscribe : tf, scan, map

    最後にrvizの設定ファイルを適用したrvizが実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成されたマップを視覚化します。
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

デフォルトで提供しているGmapping以外のSLAMを使用するには、以下のように関連パッケージをインストールする必要があります。
- cartographerをインストール：現時点で最新バージョンのcartographer(v1.0.0)は、2018年以来アップデートされておらず、TurtleBot3パッケージでシミュレーション行う場合、正常に駆動しない場合があります。TurtleBot3のSLAMをシミュレーションする際にはGmappingを使用してください。  
次の設置方法はROS1 Kineticでのみ使用できます。  
  ```bash
  $ sudo apt-get install ninja-build libceres-dev libprotobuf-dev protobuf-compiler libprotoc-dev
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/googlecartographer/cartographer.git
  $ git clone https://github.com/googlecartographer/cartographer_ros.git
  $ cd ~/catkin_ws
  $ src/cartographer/scripts/install_proto3.sh
  $ rm -rf protobuf/
  $ rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:xenial
  $ catkin_make_isolated --install --use-ninja
  $ source ~/catkin_ws/install_isolated/setup.bash
  ```
- Hector Mappingをインストール
  ```bash
  $ sudo apt-get install ros-kinetic-hector-mapping
  ```
- Kartoをインストール
  ```bash
  $ sudo apt-get install ros-kinetic-karto 
  ```
- Frontier Explorationをインストール
  ```bash
  $ sudo apt-get install ros-kinetic-frontier-exploration ros-kinetic-navigation-stage
  ```

SLAMノードが正常に駆動している場合、以下のように様々な方式のSLAMを適用した視覚化ツールであるRVizを別途実行することもできます。既に実行されているRVizがある場合、プログラムの衝突が発生することがあります。

```bash
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_gmapping.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_cartographer.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_hector.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_karto.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_frontier_exploration.rviz
```

## 遠隔操作ノードを実行する
[Remote PC] SLAMを行うためには、マップが描かれていない領域にTurtleBot3を動かす必要があります。そのために、TurtleBot3を遠隔で制御することのできるノードを以下のように実行します。

SLAMを行う際は、過度に激しい動きや急な方向転換などを避けることが正確な地図の作成に役立ちます。RViz上で描かれた地図を見ながら、未完成の領域にロボットを動かし、地図を完成させる必要があります。

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```
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

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/slam_running_for_mapping.png)


## チューニングガイド
Gmappingは、さまざまな環境に最適化されたパフォーマンスを実現するために、複数のパラメータの設定が可能です。Gmappingは一般的に別途設定なしに使用が可能であり、デフォルト設定で使用する場合が多いものです。設定可能なパラメータのリストは、[ROS wikiのGmappingパラメータ](http://wiki.ros.org/gmapping#Parameters)ページを参照してください。

以下のパラメータは、`turtlebot3_slam/launch/turtlebot3_gmapping.launch`ファイルに定義されており、ファイルを実行する際にroscoreのパラメータサーバーにロードされ、gmappingをベースにしたSLAMに適用されます。

### maxUrange
このパラメータはLDSセンサの最大使用可能範囲を設定します。 
 
### map_update_interval 
マップをアップデートする期間(秒単位)この値が低いと、マップがもっと頻繁にアップデートされます。しかし、より大きな計算負荷が必要です。環境に基づいてこのパラメータを設定してください。 

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/tuning_map_update_interval.png)

### minimumScore 
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

### linearUpdate
ロボットがこの値よりも長い距離を並進運動したとき、scanプロセスを実行します。
 
### angularUpdate 
ロボットがこの値よりも大きい角度を回転運動したとき、scanプロセスを実行します。
これをlinearUpdateより小さく設定することがいいです。 

## 地図を保存する
[Remote PC]地図が完成したら、map_saverノードを実行して生成されたマップを保存する必要があります。ロボットが動く際に発生した走行距離とtf、scanデータなどによってRViz上で完成された地図は、以下のコマンドを使用してファイルに保存することができます。
完成した地図は2つのファイルに分けて保存され、そのうち、pgmはPortable Gray Map形式の画像ファイルであり、yamlは地図の解像度など各種設定を保存するファイルです。

```bash
$ rosrun map_server map_saver -f ~/${map_name}
```

`-f`オプションは、地図ファイルを保存する場所とファイル名を指定します。上のコマンドでは、`～/${map_name}`オプションが使用されているため、ユーザーのhomeフォルダ(`～/`または`/home/\<username\>`)に `${map_name}.pgm`と`${map_name}.yaml`ファイルとして保存されます。${map_name}に保存したいファイル名を入力してください。

## 地図
ROSにおいて地図は2次元Occupancy Grid map(OGM)を主に使用します。保存されたmap.pgmイメージファイルを開くと、以下のようにロボットが移動できる白い領域と、障害物として識別されロボットが移動できない黒い領域、ロボットが探索していない灰色の領域に区分されます。このように生成されたマップは、次に紹介するNavigationで使用することができます。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/map.png)

下のイメージは、cartographerを利用して広い領域の地図を作成した例です。以下のようなマップの生成は、約1時間程度の間に計350mの距離をロボットを操縦して作成したものです。

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/large_map.png)
