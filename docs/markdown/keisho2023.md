---
layout: phyexp3-lesson
lang: en
ref: phyexp3\_keisho2023
permalink: /docs/keisho2023/
section_number: 1
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

1. アームがホームポジション（　腕が本体上でたたまれている状態）になっていることを確認する。
2. バッテリーが接続され、本体前面のスロットに正しく装着されているか確認する。
3. 本体上面の距離センサに干渉する物がないことを確認する
4. 平らで開けた場所に置いて、本体前面のスイッチを入れる
5. アームがホームポジションに移動し、距離センサが回転し始める。

### 停止手順

1. 走行している場合は付属リモコンや"teleop" ROSノードからメッセージを送り停止させる。方法がない時は本体を両手でピックアップする（アームを持たないこと）。
2. 電源が落ちるとアームが脱力するので支えられるように準備する。
3. 本体前面のスイッチを切る。

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
{% capture capture02 %}
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
