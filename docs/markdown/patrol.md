---
layout: splash
lang: en
permalink: /docs/patrol/
sidebar:
  title: Pythonプログラミングによる移動指示とマニピュレータ操作
  nav: "phyexp3_patrol"
---

{: .startsec6}
# [Class 6] Pythonプログラムを用いた移動指示とマニピュレータ操作

## Pythonプログラムでの地図上座標の取得
RVizの _Publish Point_ ボタンを押した後、地図上の点をクリックする
ことで、その位置の座標が `/clicked_point` というtopicに出力されます。
まずはこれを確認しましょう。

### `rostopic echo` を用いる場合 {#check-clicked-point-by-rostopic}
1. [前節5.](/docs/navigation)に記載の方法でRVizを起動する。
1. 新たな端末で `rostopic echo /clicked_point` を実行し `/clicked_point` に
   出力される情報を待ち受ける状態にする。
1. RViz上部の **Publish Point** ボタンをクリックする。
1. 地図上の点をクリックする。
   `/clicked_point` のtopicに座標情報が出力されるのを確認する。

複数の点について座標を確認し、RViz上でのX軸、Y軸、Z軸の向きを確認してください。

### 課題6-1 Pythonプログラムを用いた座標の取得
{% capture staff01 %}
`rostopic echo` コマンドを用いてtopicの情報を表示する代わりに
Pythonプログラム中でtopicの情報を取得することもできます。
プログラム内で利用できるデータとして取得できるので、それに応じて
プログラムの動作を変えることができます。
[上記の手順](#check-clicked-point-by-rostopic) で
`rostopic echo /clicked_point` の代わりに
```bash
$ rosrun exp3 print_clicked_point.py
```
を実行し、topicの情報が表示されることを確認してください。
これを複数の位置について行い、指定した位置と座標値を報告してください。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

### ファイルの内容確認
先程実行したPython scriptは `~/exp3_ws/src/exp3/scripts/` という
ディレクトリ(フォルダ)にある `print_clicked_point.py` というファイルに
記述されています。
ファイルの内容を確認するには以下のような方法があります。
- `cat` コマンドで端末上に表示する。
```bash
$ cat ~/exp3_ws/src/exp3/scripts/print_clicked_point.py
```
このコマンドは端末上に内容が出力された後に終了し、検索したり遡って
見ることはできません(端末自身のスクロールバーである程度は遡れます)。
- `less` コマンドで表示する。
```bash
$ less ~/exp3_ws/src/exp3/scripts/print_clicked_point.py
```
`cat` と異なり矢印キーや`j`, `k`キーの
入力で表示位置を移動できます。`q`を入力するとコマンドを終了します。
(参考: [manpage of less](http://manpages.ubuntu.com/manpages/bionic/ja/man1/less.1.html) )
- エディタで開く。
```bash
$ gedit ~/exp3_ws/src/exp3/scripts/print_clicked_point.py &
```
端末とは別にウィンドウが開いて表示、編集できます。
- GUIを利用した方法を使い、エディタで開く。
  まず下記の方法などでディレクトリを表示するウィンドウを開きます。

  {: type="a"}
  1. 画面左端に並んだアイコンの中にある `Files` をクリックして
     ウィンドウを開き、 `exp_3_ws` → `src` → `exp3` → `scripts`
     と辿る。
  1. `xdg-open` コマンドで直接ウィンドウを開く。
     ```bash
     $ xdg-open ~/exp3_ws/src/exp3/scripts/ &
     ```

  開いたウィンドウ内で `print_clicked_point.py` を右クリックし、
  **Open With Text Editor** を選択する。

`print_clicked_point.py` の内容は以下のようになっています。
~~~ python
#!/usr/bin/env python
import rospy

import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('print_clicked_point', anonymous=False)
    topic_name = '/clicked_point'

    rospy.loginfo('Waiting for a point to be clicked...')

    # rospy.wait_for_message
    # https://docs.ros.org/en/melodic/api/rospy/html/rospy.client-module.html#wait_for_message
    point = rospy.wait_for_message(
        topic_name, geometry_msgs.msg.PointStamped
    )
    rospy.loginfo('The clicked point information: %s' % (point,))
~~~
このプログラムでは `rospy.wait_for_message()` 関数を用いてtopicからの情報を
待ち、得られた情報を `point` という変数に代入してそれを表示しています。
この関数については
[rospy.clientのマニュアルの `wait_for_message` の項](https://docs.ros.org/en/melodic/api/rospy/html/rospy.client-module.html#wait_for_message)
を参照してください。


## プログラム内に記述された座標への移動
[前節5.](/docs/navigation)
ではRVizの **2D Nav Goal** ボタンを使ってRViz上の指定した位置へロボットを
移動させていましたが、Pythonプログラム上でこのような移動指示を行うことも
できます。

実験室のLinux環境の `~/exp3_ws/src/exp3/scripts/` という
ディレクトリに `go_to_fixed_point.py` というファイルがあります。
これをエディタで開いてください。
内容は以下のようになっています。
~~~ python
#!/usr/bin/env python
import rospy

import exp3_turtlebot3

if __name__ == '__main__':
    rospy.init_node('patrol')
    robot = exp3_turtlebot3.Turtlebot3(robot_origin_frame_id='base_link')
    robot.admissible_distance = -1.0

    # Specify the frame (that represents the coordinate system).
    frame_id = 'map'

    # Specify the X,Y,Z coordinates of the goal in [m] unit.
    point_xyz = [0.5, 0.5, 0.0]

    # Specify the angle to which the robot will be directed at the goal
    # in [degree] unit.
    angle = 0

    goal = robot.make_goal(frame_id, point_xyz, angle)
    rospy.loginfo('Go to the goal:\n%s' % (goal, ))
    robot.go_to_goal(goal)
~~~
変数 `point_xyz` は3つの要素を持つlistで、これでX座標,Y座標,Z座標を
指定します。
座標系は `frame_id` で指定されていて、 `map` です(地図を基準とした座標)。
また、移動後のロボットの向きは変数 `angle` で指定しています。
この変数はXY平面内の向きを度([degree])単位で表現した数値で、
X軸の正の方向が0度となっています。


{% capture staff01 %}
**課題6-2**

地図上の適当な位置の座標を確認して `point_xyz` に設定した上で
```bash
$ rosrun exp3 go_to_fixed_point.py
```
を実行し、ロボットが指定した位置に移動することを確認してください。
{% endcapture %}
<div class="notice--danger">{{ staff01 | markdownify }}</div>

{: .notice--info}
`go_to_fixed_point.py` で使われている `Turtlebot3` というクラスや
このクラスの `make_goal()` , `go_to_goal()` などのメソッドは
`exp3_turtlebot3.py` というファイルで定義されています。
指定した位置へ移動するようロボットに指示する方法として
[ROSのactionという機構](http://wiki.ros.org/ja/actionlib)を利用して
いて、これを参考にすればより細かい制御を行うことも可能です。


## 課題6-3 RViz上で指定した位置への移動
`go_to_fixed_point.py` では移動先の位置はプログラム内に
直接書かれていました。
このように、何らかの情報をプログラム上に直接書いておくことは
[ハードコーディング](https://e-words.jp/w/%E3%83%8F%E3%83%BC%E3%83%89%E3%82%B3%E3%83%BC%E3%83%89.html)
などと呼ばれます。
移動先を変更するにはその都度プログラムを修正する必要があり、面倒です。
そこで **「RViz上でクリックした点の座標を取得する機能」** と
**「座標を与えるとその位置にロボットを移動させる機能」** を合体させて
**「RViz上でクリックした点にロボットを移動させるプログラム」** を
作ってみましょう
( RViz上の **2D Nav Goal** の機能を自分で作ることに相当します)。

{: .nonumber}
## 課題6-3

{: .notice--danger}
実験室のLinux環境の `~/exp3_ws/src/exp3/scripts/` のディレクトリに
あらかじめ `go_to_clicked_point.py` という名前でプログラムの雛形を
用意してあります。
これを修正してRVizの **Publish Point** ボタンの機能で位置を指定すると
ロボットがその位置に移動するプログラムを作成してください。


## 課題6-4 アームの操作
実験室のLinux環境の `~/exp3_ws/src/exp3/scripts/` のディレクトリに
`move_arm.py` という名前のプログラムを用意してあります。
このプログラムでは3つの関数が定義されていて、それぞれの機能は以下の
通りです。
-  `rotate_arm()` : アームを回転させる
-  `extend_arm()` : アームを伸ばす
-  `initialize_arm_joint()` : アームを初期姿勢に戻す

`move_arm.py` は以下のようにして実行できますが、そのままでは
何もしないプログラムとなっています。
```bash
$ rosrun exp3 move_arm.py
```
まずは、上記の関数を使ってアームを動かすようプログラムを修正して
どのような動作が行われるか確認してください。

{: .nonumber}
## 課題6-4

{: .notice--danger}
`move_arm.py` で定義されている関数を参考にして、アームをある軌道で
動かし最後には初期状態に戻すような関数を作成し、動作させてその結果を
報告してください。
但し、アームの軌道は少なくとも3つの中継点を通るようなもので、
`rotate_arm()` とは異なるものにしてください。


## 課題6-5 RVizで指定された点にある棒を倒すプログラム
これまでに作成したロボットを移動させるプログラムとアームを動かす
プログラムを組み合わせて、地図領域内に立てた棒をアームで倒す
プログラムを作ってみましょう。

なるべく、ロボット本体の接触ではなくアームの動作で倒せるよう
考えてみてください。
棒の近くまでの移動は直線的に行えるものと仮定しても構いません。
見通せない場所にある棒にも接近できる工夫があれば加点対象となります。
アイデアや作戦をレポートに記載してください。

シミュレーション用に細い棒を設置した環境を用意してあります。
Gazebo起動時に `gazebo_manipulator_stage_4_obstacles.launch` の
launchファイルを指定することでその環境を利用できます。

プログラムの雛形として `go_to_clicked_point_and_move_arm.py` という
ファイルを用意してありますのでこれも活用してください。

{: .nonumber}
## 課題6-5

{: .notice--danger}
RVizで棒の立っている位置を指定するとその棒の近くに移動し、
アームを伸ばしてから回転させ、棒を倒すプログラムを作成、実行してください。
レポートにはプログラム作成時のアイデアや作戦についての説明、
動作させたときの様子、及び実行結果についての考察を含めてください。


## 課題 円形障害物を検知しそこに移動するプログラム(押し倒し可)
```bash
$ rosrun exp3 obstacle_detector.launch
```
go_to_obstacle.py

## 課題 円形障害物を検知しそこに近付いて止まるプログラム(押し倒し不可)
go_to_obstacle_and_move_arm.py


