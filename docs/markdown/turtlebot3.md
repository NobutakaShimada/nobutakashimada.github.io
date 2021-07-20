---
layout: splash
lang: en
ref: phyexp3\_turtlebot3
permalink: /docs/turtlebot3/
sidebar:
  title: 実機Turtlebotへの接続と移動操作
  nav: "phyexp3\_turtlebot3"
---

# [Class 3] 実機Turtlebotへの接続と移動操作

## TurtleBot3(実ロボット), シミュレーションの切り替え方法  
すべてHost PC端末(Remote PC)で実行する。
1. TurtleBot3実行方法
    - roscore  
    ターミナルを開いて以下のコマンドを入力
        ```bash
        $ roscore
        ```

    - Turtlebot3 (リモート)駆動 : machine.launch  
    別のターミナルを開いて、以下のコマンドを入力(接続しようとするTurtleBot3の番号が09であると仮定)  
    ```bash
    $ roslaunch exp3 machine.launch id:=09
    ```  
    ![イメージリンク](http://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)  
    他のユーザーがTurtleBot3を使用している場合、launchファイル実行時に以下のメッセージが出て終了する。
    ```bash
    RLException: remote roslaunch failed to launch: tb3
    The traceback for the exception was written to the log file
    ```  
    
    - Rviz  
    新たにターミナルを開いて、以下のコマンドを入力する。
        ```bash
        $ roslaunch exp3 rviz.launch
        ```

    - teleop 
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
    キーボードのキーで自由に実機のturtlebot3ロボットを動かしてみる。


2. 使用切り替え
    - 切り替え方法
        - roscoreを含むすべてのnodeを終了(実行したターミナルで`CTRL` + `C`キーを入力)
        - 上記の実行方法に従って切り替え
    - TurtleBot3 (実ロボット)とシミュレーションの比較


## 課題3
{% capture staff01 %}
1. Turtlebot実機に接続してノードを起動し（machine.launch)、教室のブロックフィールドに実機を置いてteleopで移動させてみよ。適当なところで停止させ、その様子を撮影した写真とその時のRVizの画面をノートブックに添付せよ。画面キャプチャには`gnome-screenshot`コマンドを使うことができる。
```bash
$ gnome-screenshot --area -f graph.jpg
```
2. 距離センサの反応する範囲にものを置いたり動かして、距離センサの反応がRVizの画面上で変化していることを比較して確認せよ。この時の実環境の様子とRVizの画面をキャプチャしてノートブックに添付せよ（２つの異なる状況でセンサーの反応が異なっていることがわかる画像を２種類添付すること）。
3. 【追加課題】rqt_graphをつかって、実機を接続した状態のノードとトピックの全体図を画面に表示してみよ。gazeboでシミュレータ内のロボットと接続していたときと比較して、どこが異なるか、あるいは違いがないか、確認せよ。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>


## 実機とGazeboシミュレーションの差異

|      | TurtleBot3  | シミュレーション(Gazebo) |
|:----:|:----------------|:------------------------|
| 環境 | 多様な実際の環境 |Gazebo環境<br />- 提供<br />&nbsp;&nbsp;- Empty World<br />&nbsp;&nbsp;- Turtlebot3 World<br />&nbsp;&nbsp;- Turtlebot3 House<br />- ユーザーが制作した環境<br />![](/assets/images/ritsumeikan/008.png)|
|モデル|Burger<br/>Waffle<br/>Waffle Pi|Burger<br/>Waffle<br/>Waffle Pi|
|センサーおよびトピック名|LIDAR : /scan<br />IMU : /imu<br />CAMERA(Waffle Pi) : /raspicam_node/image/compressed|LIDAR : /scan<br />IMU : /imu<br />CAMERA(Waffle, Waffle Pi) : <br />&nbsp;&nbsp;/camera/rgb/image_raw,<br />&nbsp;&nbsp;/camera/rgb/image_raw/compressed
|使用機器|Turtlebot3(Burger, Waffle, Waffle Pi)<br />Remote PC(User PC)|Remote PC(User PC)|
