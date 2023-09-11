---
layout: phyexp3-lesson
lang: en
ref: phyexp3\_keisho2023
permalink: /docs/keisho2023/
section_number: 1
section_title: 基本編(Turtlebot3の操作とROSの基礎)
sidebar:
  title: 基本編(Turtlebot3の操作とROSの基礎)
  nav: "phyexp3\_keisho2023"
---
{% assign wayback_prefix = "https://web.archive.org/web/20200929183646/" %}
# 立命館大学情報理工学部／立命館慶祥高校　体験授業2023 ROSロボットの自動走行を体験しよう

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
    ![イメージリンク](https://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)  
    他のユーザーがTurtleBot3を使用している場合、launchファイル実行時に以下のメッセージが出て終了する。
    ```bash
    RLException: remote roslaunch failed to launch: tb3
    The traceback for the exception was written to the log file
    ```  
    
3. 情報可視化アプリRvizの起動
   さらに新たなターミナルを開いて、以下のコマンドを入力する。
   ```bash
   $ roslaunch exp3 rviz.launch
   ```
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


### 
{% capture staff01 %}
1. Turtlebot実機に接続してノードを起動し（machine.launch)、教室のブロックフィールドに実機を置いてteleopで移動させてみよ。適当なところで停止させ、その様子を撮影した写真とその時のRVizの画面をノートブックに添付せよ。画面キャプチャには`gnome-screenshot`コマンドを使うことができます。
2. 距離センサの反応する範囲にものを置いたり動かして、距離センサの反応がRVizの画面上で変化していることを比較して確認せよ。この時の実環境の様子とRVizの画面をキャプチャしてノートブックに添付せよ（２つの異なる状況でセンサーの反応が異なっていることがわかる画像を２種類添付すること）。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


