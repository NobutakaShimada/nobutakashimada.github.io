---
layout: phyexp3-lesson
lang: en
ref: phyexp3\_keisho2023
permalink: /docs/keisho2023/
section_number: A
section_title: 立命館大学情報理工学部／立命館慶祥高校　体験授業2023 ROSロボットの自動走行を体験しよう
sidebar:
  title: 立命館大学情報理工学部／立命館慶祥高校　体験授業2023 ROSロボットの自動走行を体験しよう
  nav: "phyexp3\_keisho2023"
---
{% assign wayback_prefix = "https://web.archive.org/web/20200929183646/" %}


この体験授業2023はTurtlebot3+OpenManipulator-Xというロボットプラットフォームを用いて、
広くロボットの制御に用いられるROS(Robot Operating System)の分散ノード
アーキテクチャをつかった、距離センサやロボットの内界センサの計測値の取得と可視化、さらにそれらを応用した環境地図の
生成と誘導(Simultaneous Localization And Mapping: SLAM)を体験することが目的です。


## 使用ロボット
実験で用いるロボットはROBOTIS社製のTurtlebot3です。

![Turtlebot3+OpenManipulator-X](https://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/hardware_setup.png)

### 起動手順
{% capture capture02 %}
1. アームがホームポジション（　腕が本体上でたたまれている状態）になっていることを確認する。
2. バッテリーが接続され、本体前面のスロットに正しく装着されているか確認する。
3. 本体上面の距離センサに干渉する物がないことを確認する
4. 平らで開けた場所に置いて、本体前面のスイッチを入れる
5. アームがホームポジションに移動し、距離センサが回転し始める。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>
### 停止手順
{% capture capture02 %}
1. 走行している場合は付属リモコンや"teleop" ROSノードからメッセージを送り停止させる。方法がない時は本体を両手でピックアップする（アームを持たないこと）。
2. 電源が落ちるとアームが脱力するので支えられるように準備する。
3. 本体前面のスイッチを切る。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

### バッテリーについて

バッテリーの有効時間は待機状態で90分程度、走行させたりアームを稼働させると
稼働時間が短くなります。長く使わない時はスイッチを切るようにします。


## 課題１：はじめに〜付属コントローラでTurtlebot3を操作してみよう

![RC100Bリモコン](https://emanual.robotis.com/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

Turtlebot3には１台に１つBluetoothで接続されたリモコンが付属しています。裏蓋を開けて電池を入れ、中央のボタンを長押しすると電源が入って青いLEDが点灯します。
キーを押すと前進後退、左右の旋回をするはずです。確認してみてください。
ただし、ボタンを押している間だけ走るわけではありません。**リモコンの右人差し指のボタン**を押すと、**全ての動きが停止**します。まず停止させられるか確認をしてから
いろいろ試してみてください。

**注意**　リモコンの停止ボタンはリモコンから指示した制御信号だけをキャンセルします。後述のROSノード(teleop)やlaunchファイルから指示した制御信号はキャンセルされないので注意してください。

### リモコンによるロボットの操縦体験
{% capture staff01 %}
リモコン操作でロボットの動きをよく観察しましょう。リモコンからの入力はどういう信号に変換されて送られていると思うか（位置、速度、加速度など）。各キーにバインドされた制御信号を予想しながら操作しましょう。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


### ROSの基礎

![ROSの構造](/assets/images/ritsumeikan/ROS_architecture.png)

ROSの中核部分は、**ノード**と呼ばれる複数のプログラム（PythonやC++で実装される）が、特定の名前がついた**トピック**と呼ばれる通信の宛先を介して情報を送受信する仕組みを提供する、通信プラットフォームである。

- **roscore** ROS(version1)を使う時には必須のコアシステム。roscoreを共有するノード同士が互いに通信することができ、一群のノードの中でroscoreは１つだけ起動する必要がある。どのroscoreに接続するかは環境変数ROS_MASTER_URIにIPアドレスを含めたURIを記述することで指定することができる。今回の実験ではこのあたりのセットアップはすでに完了してあるので意識しなくてよい。教室には複数のロボットと端末が存在するが、同じネットワークにいてもROS_MASTER_URIが異なるのでそれぞれ別のroscoreを起動できるし、互いに混信せずにすむようになっている。

- **実機とシミュレータ** 今回の実験では各端末上のUbuntu Linuxにロボットのシミュレータ（Gazebo）をインストールしており、最初はシミュレータでロボットの操作の練習をして、うまくいったら実機ロボットに切り替えて同じプログラムを動作させる。この切り替えは**launchファイル**というROSノードの起動スクリプトの中で実行できる（切り替えの仕方については後述する）。

- **メッセージ** ROSによるロボットへの指令は全てメッセージと呼ばれるデータをネットワーク通信でノードに送信することで実現される。ROSにおけるメッセージ送受信のアーキテクチャはいくつかあるが、この実験では**Publisher/Subscriberモデル**という、一方向通信について学ぶ。このほかにリクエストに対するレスポンスを受け取ることができる**クライアント／サービスモデル**がある。

[ROS Wiki](https://wiki.ros.org/) にROSに関するドキュメントが
掲載されている。
その中のノードやトピックに関連するチュートリアルページを以下にまとめる。

| [ROS Wiki日本語版](https://wiki.ros.org/ja/) | [ROS Wiki](https://wiki.ros.org/) | [Wayback Machine](https://web.archive.org/) (日本語版) |
|---|---|---|
| [ROS チュートリアル](https://wiki.ros.org/ja/ROS/Tutorials/) | [URL](https://wiki.ros.org/ROS/Tutorials/) | [URL]({{ wayback_prefix }}https://wiki.ros.org/ja/ROS/Tutorials/) |
| →[ROSのノードを理解する](https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes) | [URL](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes) | [URL]({{ wayback_prefix }}https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes) |
| →[ROSトピックの理解](https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics) | [URL](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics) | [URL]({{ wayback_prefix }}https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics) |
| →[ROSのサービスとパラメータを理解する](https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams) | [URL](https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams) | [URL]({{ wayback_prefix }}https://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams) |
| →[シンプルな配信者(Publisher)と購読者(Subscriber)を書く(Python)](https://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29) | [URL](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) | [URL]({{ wayback_prefix }}https://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29) |

[ROS Wiki日本語版](https://wiki.ros.org/ja/)や
[ROS Wiki](https://wiki.ros.org/)はアクセス集中のため、一時的に
アクセスできなくなることがある。
その場合は上記の表の [Wayback Machine](https://web.archive.org/) のURLから
参照すればよい。
[Wayback Machine](https://web.archive.org/) は
[Internet Archive](https://archive.org/) による過去ページの
キャッシュデータを表示するサービスであり、表示には時間がかかる。

### 主なROS コマンド

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

**ROSトピックコマンドの詳細**

|コマンド|説明|
|:---|:---|
|rostopic bw|     トピックで使用されている帯域を表示する|
|rostopic echo|   スクリーンにメッセージを出力する|
|rostopic find|   メッセージの型からトピックを探す|
|rostopic hz|     トピックの配信頻度を表示する|
|rostopic info|   アクティブなトピックについての情報を出力する|
|rostopic list|   アクティブなトピックをリスト表示する|
|rostopic pub|    トピックへデータを配信する|
|rostopic type|   トピックの型を出力する|


## 課題２：実ロボットとPCとの接続  
この後のコマンドは、すべてHost PC端末(Remote PC / Raspberry PI4)上のターミナルでキーボードから打ち込んで実行します。
### ロボットとの接続手順
{% capture capture02 %}
1. roscoreの起動（すべての最初）
   ターミナルを開いて以下のコマンドを入力
   ```bash
   $ roscore
   ```
2. Turtlebot3のへの接続 : machine.launch  
    ロボットの電源を入れ、センサが回転しアームが持ち上がったら以下のコマンドを実行すると無線LAN経由でPCとロボットが接続されます。
    roscoreとは別のターミナルを開いて（新しいタブを開くとよい）、以下のコマンドを入力(接続しようとするTurtleBot3の番号が09であると仮定)  
    ```bash
    $ roslaunch exp3 machine.launch id:=09
    ```  
    他のユーザーがTurtleBot3を使用している場合、launchファイル実行時に以下のメッセージが出て終了する。
    ```bash
    RLException: remote roslaunch failed to launch: tb3
    The traceback for the exception was written to the log file
    ```  
    
3. 情報可視化アプリRvizの起動
   さらに新たなターミナルを開いて、以下のコマンドを入力すると、ロボット内蔵のカメラや距離センサーの情報をビジュアライズするツールが起動します。
   ```bash
   $ roslaunch exp3 rviz.launch
   ```
   ![イメージリンク](https://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)  

4. キーボードからロボットへの移動コマンドを送るteleopの起動：teleop.launch
   TurtleBot3をキーボードでコントロールするために、teleop.launchを起動する。
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
   キーボードのキーで自由に実機のturtlebot3ロボットを動かしてみましょう。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

### 課題内容
{% capture staff01 %}
1. Turtlebot実機に接続してノードを起動し（machine.launch)、教室のブロックフィールドに実機を置いてteleopで移動させてみよう。適当なところで停止させ、rvizの画面に映る障害物の様子などを確認してみよう。
2. 距離センサの反応する範囲にものを置いたり動かして、距離センサの反応がRVizの画面上で変化していることを比較して確認しよう。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


## 課題３：センサーに基づく環境地図の自動生成（SLAM）
SLAM(Simultaneous Localization and Mapping)とは、未知の領域をロボットが探索し、ロボットに取り付けられたセンサを通じて取得された環境情報を用いて、ロボットが現在の自己位置を推定することと、環境地図を作成することを同時に行う技術です。SLAMは、ナビゲーション（目的地への誘導）や無人自動車の自律走行に欠かせない重要な要素です。外部情報を取得するために使用されるセンサには、距離を測定することができるセンサや周辺の画像を取得できるセンサがあります。赤外線距離センサ(IR)、音波センサ(SONAR)、レーザーセンサ(LRF)などがよく使用されており、最近では映像を解析するアルゴリズムの発達によってRGBカメラが数多く使用されています。

次の瞬間のロボットの位置を予測するため、ロボットの車輪と接続されたエンコーダの値を読み取り、推測航法(Dead Reckoning)を使用して走行距離(odometry)を計算することになりますが、この時、車輪と地面の間の摩擦などによって誤差が生じます。実際のロボットの位置と予測されたロボットの位置の誤差を減らすために、慣性測定センサ(IMU)から得られたデータを利用して位置補正を行うことができます。

距離センサ値を用いて位置の誤差を低減するために使用される方法として、Kalman filter、Markov Localization、Monte Carlo Localizationなどがあります。

SLAMでマップを作成する際、いくつかの注意点があります。

広い倉庫やホールのようにセンサの範囲で届かない広い領域の地図を作成しようとすると地図が正しく作成できません。これは、センサの範囲を両腕の長さであると仮定したとき、ホールの中心で目を閉じて両腕を使って現在地を探ろうとするのに似ています。手（センサーのレーザ）が届かないor反射してこない領域について情報が得られないためです。
これと類似して、角などの特徴のない長い廊下も地図の描画が困難です。特徴のない両壁面からなる長い廊下で目を閉じ、壁を手でつたって歩くと、廊下のどの位置まで来たのか把握しづらいことと同様です。

このような場合、マッピングアルゴリズムが特徴点として参照できるよう、地図上のあちこちに物体や障害物を設置することで解決できます。このような作為的な特徴は、一定のパターンや対称の形で置くよりも、パターンがない形で置くこともよいでしょう（注：理由を考えてみましょう）

SLAMにはいくつかのアルゴリズムがありますが、この授業ではgmappingという手法と実装を使います。

### SLAMを実行する(Turtlebot3実機)
{% capture capture02 %}
0. 先の手順にしたがってroscore, machine.launchを起動しておきます。ただしrvizは停止させておきます(rviz.launchをしたターミナルタブ上でCtrl-C（CtrlキーとCを同時に押す）を入力すると停止してプロンプトが帰ってきます)。

1. SLAMを実行します。
  ```bash
  $ roslaunch exp3 slam.launch
  ```
2. teleopでロボットを手動で移動させてみる。
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
  キーボードのキーで自由にロボットを動かしながら、どのように地図が生成されていくか確認してみよう。
  
4. 十分環境内でロボットを移動させて地図が出来上がったら、RVizやslamを動かしたままで新しい端末を開き、save_map.launchを起動して地図を保存します。
  ```bash
  $ roslaunch exp3 save_map.launch map_name:=map1
  ```
　引数map_nameに地図の名前を入れるとその名前(上の実行例では"map1")で地図ファイル（.pgmと.yamlの２つのファイル）が`~/exp3_ws/src/exp3/map`ディレクトリに保存されます。roscdコマンドを使うと簡単にアクセスができます。roscdコマンドは指定したROSパッケージのディレクトリに直接移動することができます。
  ```bash
  $ roscd exp3
  $ ls
  CMakeLists.txt  README  include  launch  map  package.xml  rviz  src  world
  $ cd map
  $ ls
  a.pgm  a.yaml
  ```
作成した地図が格納されたpgmファイルは画像ファイルです。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

**参考**  
SLAMに限らずROSのモジュール連携では関係するコンピュータノードの時刻合わせが大変重要です。時刻が100msecほどズレているだけで正常に動作できないことがあります。時刻が合っていない時にはntpdateコマンドなどでNTPサーバに問い合わせて時刻合わせをすることができます。本実験ではNTPサービスをOS起動時に起動しているので意識する必要はないはずです。  
```bash
$ sudo ntpdate ntp.ritsumei.ac.jp
```
{: .notice}

### 課題内容
{% capture staff01 %}
1. 教室内の床面に物を置いてコースを作ってみよう。回転している距離センサの高さのところまで高くないと検知されないのである程度背の高いもの（鞄や立てかけた本などでもよい）を置いてみよう。必ずしも、壁で取り囲まなくても構わない。椅子や机の足のような細い棒状のものは小さくしか写らないので注意。
2. アレンジしたコースにTurtlebot実機を置いて、上の要領でSLAMを実行しコースの環境地図を作成してみよう。作成できた地図画像(`~/exp3_ws/src/exp3/map`ディレクトリにある）をファイルビューアからアクセスして確認してみよう。地図画像の画素には白と黒とグレーの３種類があるはずです。それぞれに意味がありますが、考えてみましょう。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

### （参考）アーム（OpenManipulator-X）搭載時の制限

OpenMANIPULATOR-Xを組み付けたTurtleBot3のSLAMでは、ロボットアームがLDSセンサーの一定部分を塞いでいるため、SLAMに使用されるLDSセンサーの範囲を制限することによって地図作成を行います。
![](https://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_slam.png)
![](https://emanual.robotis.com/assets/images/platform/turtlebot3/slam/slam_running_for_mapping.png)

## 課題４：地図に基づく自己位置推定と目的地誘導（ナビゲーション）
ナビゲーション（誘導）は、特定の環境で指定された位置にロボットを移動させることです。そのために与えられた環境にある障害物や壁などの幾何的な情報が含まれた地図が必要です。SLAMを実行することによって、
センサが獲得した距離情報やロボットの移動量情報から未知環境の地図を自動的に生成することができました。
ナビゲーションを利用すると、地図情報、ロボットの内界センサ（車輪に装着された回転計：エンコーダ）、IMUセンサ及び距離センサ）の情報を使って、ロボットが現在位置から地図上の目標位置に移動することができます。

{::comment}
The below image 'costmapspec.png' is retrieved from
https://wiki.ros.org/costmap_2d#Inflation by the following command;
curl --remote-name --remote-time --remote-header-name 'https://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png'
{:/comment}
![](/assets/images/ritsumeikan/costmapspec.png)

DWA(Dynamic Window Approach)は、障害物を回避する経路の生成に使用される代表的な方法です。DWAは速度空間内で並進速度(v)と回転速度(ω)を使用し、ロボットが移動できる経路を予測・計算し、ハードウェアが出しうるロボットの最大速度が限界値として定められます。dwa_local_plannerでは、ロボットが目的地に到着できるよう進む経路であるglobal planと、障害物の回避に使われるglobal costmapによってロボットに移動可能な速度コマンドを計算し、local planを作成します。DWAは以下の順序で動作します。

1. ロボットの制御位置(dx、dy、d&theta;)を個別に生成します。
2. 生成されたそれぞれのサンプルを、ロボットの現在地からシミュレーションし、短時間サンプルが適用される場合、ロボットの位置を予測します。
3. シミュレーションの結果を点数に換算します。このとき、障害物との距離、目的地までの距離、global planとの距離、速度などの要素が考慮され、物体と衝突するサンプルは結果の計算から排除されます。
4. 最も高いスコアを獲得したサンプルが選択され、ロボットに伝達されます。
5. 1 ~ 4のプロセスを繰り返します。

これら自己位置推定やそれに基づくcostmapの生成、さらに移動経路計画の立案はROSの標準モジュールによってある程度実現されています。この実験ではこれらを起動して、生成した地図を使ったナビゲーションを体験します。


### ナビゲーションの実行: navigation.launch
{% capture capture02 %}
SLAMで制作した地図をもとにナビゲーションを行うには以下のコマンドをターミナルから入力します。
slam.launchやteleop.launchは必ず停止しておいてください（teleopとnavigationはともにロボットに移動指令を出すので、両者が干渉してうまく動かなくなります）。
```bash
$ roslaunch exp3 navigation.launch map_name:=map1
```
上の例はSLAMで生成し保存した地図の名前（拡張子を除く）が"map1"の場合です。別の名前のときは適宜対応した名前を入力してください。

navigation.launchでは以下の多くのROSノードプロセスが起動し、互いに連携して動作します。
1. **turtlebot3_remote.launch**
  - robot_state_publisher : ロボットの各関節の情報を受信し、得られた関節についての情報をurdfを参考にtfの形式でpublishします。
  - ロボットのurdfを定義された位置から読み込みます。また、joint_statesとurdfを利用して、tfをpublishするrobot_state_publisherノードを生成します。  
2. map_serverノード
  - map_serverノードは、SLAMで作成した地図画像ファイルを読み込み、必要に応じて障害物などの情報を提供します。
3. amcl.launch
  - 地図とセンサーのscan値、ロボットの位置と各座標変換の情報を読み取り、particle filterを使用して地図上でロボットの位置を予測します。
  - amclはParticle Filterというモンテカルロ法に基づく確率的状態推定アルゴリズムによって、センサ情報と地図情報からロボットの自己位置（２次元平面上の座標(x,y)とロボットの向きθ）を推定するノードです。RVizを起動すると、細かい矢印が多数表示されますが、これがamclの推定している推定パーティクルで、１つ１つが確率値を持つ推定候補です。最初は広い範囲に散らばっていて自己位置姿勢の推定が不確かなことが分かりますが、移動しながら情報を集めることによって次第にパーティクルが集まってきて真の自己位置が確からしく推定できるようになることを目視でよく確認してください。
4. move_base
  - move_baseノードは、global plannerとlocal plannerを接続してロボットを目的地まで移動させ、この時それぞれのplannerに合ったcostmapも保管します。ロボットの目的地をAction形態のトピックで受信すると、現在地と状態、移動の結果をアップデートします。また、現在の状態に合わせてロボットを動かすためのcmd_velトピックが持続的にpublishされます。
5. rviz
  - 最後に可視化ツールrvizが自動的に実行され、tf、scan、mapデータをsubscribeしてロボットとセンサ値、gmappingによって生成された地図を表示し、現在の自己位置推定の様子が可視化されます。rvizの画面操作により、ロボットの初期位置の指定や、移動の目標地点を指定して、ロボットを自動的に誘導することができます。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>


### rviz画面上での初期位置指定

ナビゲーションを行う際に最も重要なことは、ロボットの正確な初期位置を地図上で示すことです。元々どこにいそうなのかがわからなければ、ロボットが自己位置を推定するにはたくさんの可能性を試さねばならず時間と計算コストが非常にかかります。  

ロボットのエンコーダとIMU、LDSなど各種センサから得られた情報をもとにTurtleBot3の位置を推定するには、確率をベースにしたAMCL(Adaptive Monte Carlo Localization)というparticle filterが使用されます。AMCLは、センサ情報に基づいてロボットの位置を推定します。また、アルゴリズムのパラメータに設定された移動量をロボットが移動するたびに、推定位置の値を更新し、更新が繰り返されるたびに推定位置の誤差が減少していきます。

RVizが起動し、ロボットがナビゲーションを実行するための地図が表示されたら、最初のロボットの位置は実際のロボットの現在位置とはズレています。  
そこで、Rviz画面の上にあるボタンの中から`2D Pose Estimate`ボタンでロボットの初期位置を実際のロボット位置姿勢に合わせます。  

{% capture capture02 %}
  1. Rvizで`2D Pose Estimate`ボタンをクリックする。
  2. 地図上でロボットが実際に位置する点をクリックし、ロボットの前面が向いている方向に矢印をドラッグして、大きな緑色の矢印の方向をロボットが向いている方向に設定する。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

これが完了すると、ロボットは緑の矢印で指定された位置と方向を初期ポーズとして使用し、実際の位置と方向を推定します。緑の矢印はタートルボット３の予想位置を表示します。レーザースキャナーは地図上に今ロボットが観測している壁の位置を表示します。地図とのズレが大きい場合はやり直します。多少ずれていてもロボットを移動させて観測情報を増やしていくと、次第に推定が確からしくなります。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_pose_estimate.png)

> 上の図で、ロボットの周辺に広く表示された多数の小さな緑の矢印は、ロボットの初期位置を指定した際にロボットのセンサーと各種情報を介してAMCLが計算したロボットの現在位置です。ロボットの移動に従って分布していた現在地の予想値がロボットに向かって近づき、収束する様子が分かります。

> **注意**： ナビゲーションを実行する前にteleop.launchが実行されている場合は、必ず終了させる必要があります。teleopは常に指定された速度(cmd_velトピック）を指令し続けているので、move_baseの指示する速度と衝突してうまく動作できなくなります）。

### 移動先ゴール（ロボットの位置と向き）の設定

次にロボットを目標位置に誘導するために、到着目的地の位置と方向を指定します。初期位置の指定と同様に、Rvizの`2D Nav Goal`ボタンをクリックし、ロボットが移動できる目的地をクリックし、ロボットが向いている方向をドラッグして、矢印の方向を指定します。

{% capture capture02 %}
1. Rviz上部の `2D Nav Goal`ボタンをクリック。
2. 地図上でロボットが移動する目的地をクリックし、ロボットが向いている方向をドラッグして、目的地設定を完了。
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

目的地設定が完了すると、Navigationアルゴリズムはロボットの現在位置から目的地までの経路を生成し、経路を辿れるようにロボットに命令を出します。ロボットが動いている間に経路上に**地図にはない障害物**が現れると、その障害物を回避できる経路を再度生成し直して移動を試みます。

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

目的地までの経路が生成できない場合、Navigation Goal設定が失敗する場合があります。ロボットが目的地まで移動する途中でロボットを停止させたい場合は、ロボットの現在位置を目的地として再設定します。


### 課題内容

{% capture staff01 %}
1. 実機を教室内のコースに置き、SLAMの時に作った地図を指定してナビゲーションを実行してみよう。ただしコースがSLAMの時と大きく変わっているとうまく動かない可能性があります。
2. ゴールを指定してナビゲーションを開始すると、移動経路がRViz上に示されます。緑の矢印で表示されているのはロボットの現在の位置と向きの推定候補です（パーティクルフィルタ）。これらが移動している間に次第に密集して、自己位置推定が確からしくなる様子を確かめましょう。
3. 実際にコース上に手や足、ブロックなどの地図には存在しない障害物をおいて、ロボットがどのように避けるか確認しましょう。地図にない物、あるいは動くものを置いても、（避けられる時は）回避するルートを自動的に再生成して、移動をつづけようとします。
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
