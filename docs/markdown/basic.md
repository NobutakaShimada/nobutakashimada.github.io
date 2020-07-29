---
layout: splash
lang: en
ref: phyexp3\_basic
permalink: /docs/basic/
sidebar:
  title: 基本編(Turtlebot3の操作とROSの基礎)
  nav: "phyexp3\_basic"
---

# [Class 1] 基本編(Turtlebot3の操作とROSの基礎)

この実験はTurtlebot3+OpenManipulator-Xというロボットプラットフォームを用いて、
広くロボットの制御に用いられるROS(Robot Operating System)の分散ノード
アーキテクチャを学ぶとともに、シミュレータ(Gazebo)と実機による距離センサや
ロボットの内界センサの計測値の取得と可視化の仕方や、それらを応用した環境地図の
生成と誘導(Simultaneous Localization And Mapping)を体験することが目的です。


## 使用ロボット
実験で用いるロボットはROBOTIS社製のTurtlebot3です。

![Turtlebot3+OpenManipulator-X](https://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/hardware_setup.png)

### 起動手順

1. アームが図のようなホームポジションになっていることを確認する。
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


## ROSを学ぶ前に〜付属コントローラでTurtlebot3を操作してみよう

![RC100Bリモコン](https://emanual.robotis.com/assets/images/platform/turtlebot3/example/rc100b_with_bt410.png)

Turtlebot3には１台に１つBluetoothで接続されたリモコンが付属しています。裏蓋を開けて電池を入れ、中央のボタンを長押しすると電源が入って青いLEDが点灯します。
キーを押すと前進後退、左右の旋回をするはずです。確認してみてください。
ただし、ボタンを押している間だけ走るわけではありません。**リモコンの右人差し指のボタン**を押すと、**全ての動きが停止**します。まず停止させられるか確認をしてから
いろいろ試してみてください。

**注意**　リモコンの停止ボタンはリモコンから指示した制御信号だけをキャンセルします。後述のROSノード(teleop)やlaunchファイルから指示した制御信号はキャンセルされないので注意してください。

### [課題1-1]
{% capture staff01 %}
リモコン操作でロボットの動きをよく観察しなさい。リモコンからの入力はどういう信号に変換されて送られていると思うか（位置、速度、加速度など）。各キーにバインドされた制御信号を予想してノートブックに記載しなさい。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


## ROSの基礎

![ROSの構造](/assets/images/ritsumeikan/ROS_architecture.png)

ROSの中核部分は、**ノード**と呼ばれる複数のプログラム（PythonやC++で実装される）が、特定の名前がついた**トピック**と呼ばれる通信の宛先を介して情報を送受信する仕組みを提供する、通信プラットフォームである。

- **roscore** ROS(version1)を使う時には必須のコアシステム。roscoreを共有するノード同士が互いに通信することができ、一群のノードの中でroscoreは１つだけ起動する必要がある。どのroscoreに接続するかは環境変数ROS_MASTER_URIにIPアドレスを含めたURIを記述することで指定することができる。今回の実験ではこのあたりのセットアップはすでに完了してあるので意識しなくてよい。教室には複数のロボットと端末が存在するが、同じネットワークにいてもROS_MASTER_URIが異なるのでそれぞれ別のroscoreを起動できるし、互いに混信せずにすむようになっている。

- **実機とシミュレータ** 今回の実験では各端末上のUbuntu Linuxにロボットのシミュレータ（Gazebo）をインストールしており、最初はシミュレータでロボットの操作の練習をして、うまくいったら実機ロボットに切り替えて同じプログラムを動作させる。この切り替えは**launchファイル**というROSノードの起動スクリプトの中で実行できる（切り替えの仕方については後述する）。

- **メッセージ** ROSによるロボットへの指令は全てメッセージと呼ばれるデータをネットワーク通信でノードに送信することで実現される。ROSにおけるメッセージ送受信のアーキテクチャはいくつかあるが、この実験では**Publisher/Subscriberモデル**という、一方向通信について学ぶ。このほかにリクエストに対するレスポンスを受け取ることができる**クライアント／サービスモデル**がある。

[ROSチュートリアル：ROSノードについて http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingNodes)  
[ROSチュートリアル：ROSトピックについて http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics)  
[ROSチュートリアル：ROSサービスについて http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams)  
[ROSチュートリアル：シンプルな配信者(Publisher)と購読者(Subscriber)を書く(Python)](http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

### 課題1-2
{% capture staff01 %}
1. 上の[「シンプルな配信者(Publisher)と購読者(Subscriber)を書く」チュートリアル](http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29)のページを開いて、記載されている指示に従いtalker.pyとlistener.pyをダウンロードするか直接ファイルにコピーペーストして保存しなさい。
2. 保存した２つのプログラムを指示に従って動かして、どういう動作をするか確かめなさい(ターミナルを開いてrosrunコマンドを使う)。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

- **rosrun** ROSのノードプログラムを起動するためのコマンド。以下の形式で呼び出す。１つ目の引数はパッケージと呼ばれる一塊りのアプリケーション。２つ目の引数はそのパッケージの中で起動すべきノードプログラムの名前である。
```bash
$ rosrun beginner_tutorial listener.py
```

- **roslaunch** ROSのノードプログラムをrosrunで個別に起動するとノードがたくさんある時（実際のロボットではノードの数は数十にのぼる）いちいちコマンドを呼び出していられない。これをまとめて実行するためのスクリプトが**launchファイル**であり、launchファイルを起動するためのコマンドが**roslaunch**である。


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


## ROSノードをPythonで実装してみる

### 課題1-3
{% capture staff01 %}
1. 授業中の解説を参考に、キーボードで入力したテキスト文字を送受信するROSノードをpython(rospy)を使って実装しなさい。python2ではキーボード入力の取得にraw_input()が使える。
2. そのコード（サーバ：listener2.pyとクライアント：talker2.py）をノートブックに添付しなさい。
3. テキストの送受信に使用されるトピック名と、送受信メッセージの定義（rosmsgコマンドを使用せよ）をノートブックに添付せよ。
4. 実行した時の画面表示をキャプチャしてノートブックに添付しなさい。
5. rostopic echoコマンドを実行して、流れているROSメッセージを端末に表示し、それをキャプチャしてノートブックに添付しなさい。
6. rqt_graphコマンドを実行して、ノード・トピックの関係図を表示し、画像としてノートブックに添付しなさい。画面キャプチャには`gnome-screenshot`コマンドを使うことができる。
```bash
$ gnome-screenshot --area -f graph.png 
```
7. talker2.pyを複数起動すると、2ノード以上からのテキストをlistener2.pyは表示することができる。実際に試してみて、なぜそれが可能なのか考察しノートブックで説明せよ。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

### 課題1-4
{% capture staff01 %}
1. talker2.pyとlistener2.pyのpythonコードを参考に、双方向でテキストを送受信できるように改造した.pyを実装せよ。
2. 改造したコードをノートブックに添付しなさい。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>





