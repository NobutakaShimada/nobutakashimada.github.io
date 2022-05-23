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
[課題6-1](#exercise6-1)
[課題6-2](#exercise6-2)
[課題6-3](#exercise6-3)
[課題6-4](#exercise6-4)
[課題6-5](#exercise6-5)
[課題6-6](#exercise6-6)

[発展課題6-EX1](#exercise6-ex1)
[発展課題6-EX2](#exercise6-ex2)



## Pythonプログラムでの地図上座標の取得
RVizの **Publish Point** ボタンを押した後、地図上の点をクリックする
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


### Pythonプログラムを用いた座標の取得
`rostopic echo` コマンドを用いてtopicの情報を表示する代わりに
Pythonプログラム中でtopicの情報を取得することもできます。
プログラム内で利用できるデータとして取得できるので、それに応じて
プログラムの動作を変えることができます。

{:id="exercise6-1"}
{% capture exercise6-1 %}
1. [上記の手順](#check-clicked-point-by-rostopic) で
   `rostopic echo /clicked_point` の代わりに
   ```bash
   $ rosrun exp3 print_clicked_point.py
   ```
   を実行し、topicの情報が表示されることを確認してください。
   これを複数の位置について行い、RViz上でのX軸、Y軸の向きを確認
   してください。
1. `teleop` などを使ってロボットを初期位置からある程度離れた
   位置に移動した後、
   RVizの **Publish Point** の機能でロボットの中心位置を指定して
   その座標を取得し、レポートで報告してください。
1. RViz上の表示を以下のように調整した後の状態で
   RVizのウィンドウをキャプチャしレポートに貼り付けてください。
   - `map` frame における原点(X,Y) = (0,0)が
     地図表示パネルの中心に位置するようにする。
   - `map` frame におけるX軸が画面の水平方向であり、X軸の正の方向が
     右向きとなり、Y軸の正の方向が垂直上向きとなるように地図を回転させる。
   - ロボットがパネル内に写るよう地図を拡大縮小させる。
   - 地図表示パネル内のグリッド線はデフォルトの通り1m間隔で表示させた
     ままとする。
   - RVizの左側に表示される表示内容選択パネル( `Displays` )を畳んで
     地図を見やすくしておく。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-1 title="課題6-1" %}


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
- エディタ(gedit)で開く。
  ```bash
  $ gedit ~/exp3_ws/src/exp3/scripts/print_clicked_point.py &
  ```
  端末とは別にウィンドウが開いて表示、編集できます。

  {: .notice--info}
  実験室のLinux環境には **gedit** の他にも **vi** や **VSCode** などの
  エディタも導入されています。好みのものを使ってください。
- GUIを利用してファイルを指定しエディタ(gedit)で開く。

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
前節の[5.生成した環境地図に基づくナビゲーション](/docs/navigation)
ではRVizの **2D Nav Goal** ボタンを使ってRViz上の指定した位置へロボットを
移動させていましたが、Pythonプログラム上でこのような移動を指示することも
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

地図上の適当な位置の座標を確認して `point_xyz` に設定した上で
```bash
$ rosrun exp3 go_to_fixed_point.py
```
を実行し、ロボットが指定した位置に移動することを確認してください。
また、目標位置の座標を変更して複数回実験を行い、座標に応じた位置に
移動していることを確認してください。

{:id="exercise6-2"}
{% capture exercise6-2 %}
1. 地図上でロボットが到達可能な移動先を設定し、目標位置の `map` frameに
   おける座標をレポートで報告してください。
1. 変数 `point_xyz` に目標位置の `map` frameにおける座標が代入される
   ように `go_to_fixed_point.py` を修正して、
   修正後のプログラムをレポートに貼り付けてください。
1. 移動前の状況として、RViz上の表示を以下のように調整した後の状態で
   RVizのウィンドウをキャプチャしレポートに貼り付けてください。
   - `map` frame における原点(X,Y) = (0,0)が
     地図表示パネルの中心に位置するようにする。
   - `map` frame におけるX軸が画面の水平方向であり、X軸の正の方向が
     右向きとなり、Y軸の正の方向が垂直上向きとなるように地図を回転させる。
   - **ロボットと目標位置の両方** がパネル内に写るよう地図を拡大縮小させる。
   - 地図表示パネル内のグリッド線はデフォルトの通り1m間隔で表示させた
     ままとする。
   - RVizの左側に表示される表示内容選択パネル( `Displays` )を畳んで
     地図を見やすくしておく。

   実機で実験を行った場合は移動前の周囲状況の写真も貼り付けてください。
1. 修正した `go_to_fixed_point.py` を下記のコマンドで実行し、
   ロボットが指定した位置に移動することを確認してください。
   ```bash
   $ rosrun exp3 go_to_fixed_point.py
   ```
1. 移動後の状況として、RVizのウィンドウをキャプチャしレポートに
   貼り付けてください。
   但し、移動前の状況と比較できるようRViz内の地図表示パネルの
   表示範囲や角度が移動前のものと変わらないようにしてください。

   実機で実験を行った場合は移動後の周囲状況の写真も貼り付けてください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-2 title="課題6-2" %}

{: .notice--info}
`go_to_fixed_point.py` で使われている `Turtlebot3` というクラスや
このクラスの `make_goal()` , `go_to_goal()` などのメソッドは
`exp3_turtlebot3.py` というファイルで定義されています。
それほど長くないプログラムなので、より細かい動作を確認したい方は
是非 `exp3_turtlebot3.py` の中身もチェックしてみてください。
指定した位置へ移動するようロボットに指示する方法として
[ROSのactionという機構](http://wiki.ros.org/ja/actionlib)を利用して
いて、これを参考にすればより細かい制御を行うことも可能です。



## RViz上で指定した位置への移動
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

{:id="exercise6-3"}
{% capture exercise6-3 %}
1. 実験室のLinux環境の `~/exp3_ws/src/exp3/scripts/` のディレクトリに
   あらかじめ `go_to_clicked_point.py` という名前でプログラムの雛形を
   用意してあります。
   これを修正してRVizの **Publish Point** ボタンの機能で位置を指定すると
   ロボットがその位置に移動するプログラムを作成しレポートに貼り付けて
   ください。
1. 指定する位置を変えてプログラムを実行することを複数回行い、指定した
   位置に移動できることを確認してください。
1. 下記の条件で移動前のRVizの画面をキャプチャし
   レポートに貼り付けてください([課題6-2](#exercise6-2) と同様)。
   - `map` frame における原点(X,Y) = (0,0)が
     地図表示パネルの中心に位置するようにする。
   - `map` frame におけるX軸が画面の水平方向であり、X軸の正の方向が
     右向きとなり、Y軸の正の方向が垂直上向きとなるように地図を回転させる。
   - **ロボットと目標位置の両方** がパネル内に写るよう地図を拡大縮小させる。
   - 地図表示パネル内のグリッド線はデフォルトの通り1m間隔で表示させた
     ままとする。
   - RVizの左側に表示される表示内容選択パネル( `Displays` )を畳んで
     地図を見やすくしておく。

   実機で実験を行った場合は周囲状況の写真も貼り付けてください。
1. 移動前と同様の条件で移動後についてもRVizの画面をキャプチャし
   レポートに貼り付けてください。
   実機で実験を行った場合は周囲状況の写真も貼り付けてください。
1. ロボットが意図通りに動いたかを考察してください。
   - 意図通りに動いたとしたら、どのような状況からそう判断したかを
     画像を参照しながら説明してください。
   - 意図通りに動かなかったとしたら、その理由を説明し改善方法を
     考えて挙げてください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-3 title="課題6-3" %}



## アームの操作
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
まずは上記の3つの関数を使ってアームを動かすようプログラムを修正して
どのような動作が行われるか確認してください。

`move_arm.py` の内容は以下の通りです。
~~~ python
#!/usr/bin/env python
import rospy

import geometry_msgs.msg
import exp3_turtlebot3

def rotate_arm(robot):
    # Each entry means a pair of joint angles[rad] and time[sec].
    joint_waypoints = [
        [{'joint1': 0.0}, 0.0],
        [{'joint1': -0.85*math.pi}, 3.0],
        [{'joint1': 0.85*math.pi}, 6.0],
        [{'joint1': 0.0}, 9.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)

def extend_arm(robot):
    joint_waypoints = [
        [{'joint2': 0.4*math.pi, 'joint3': -0.3*math.pi}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)

def initialize_arm_joint(robot):
    joint_waypoints = [
        [{'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)


if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=False)
    robot = exp3_turtlebot3.Turtlebot3(robot_origin_frame_id='base_link')
    robot.admissible_distance = -1.0

    ########################################################
    # Move arm
    #

    # WRITE CODE HERE

    ########################################################


~~~

{:id="exercise6-4"}
{% capture exercise6-4 %}
1. `move_arm.py` であらかじめ定義されている関数を参考にして
   「アームをある軌道で動かし最後には初期状態に戻すような関数」を
   作成し、レポートに貼り付けてください。
   但し、アームの軌道は少なくとも3つの中継点を通るようなもので、
   `rotate_arm()` とは異なるものにしてください。
1. 作成したプログラムによって実現されるアームの動作を説明する
   文章を作成しレポートで報告してください。
1. 実際に実機で動作させてその様子を撮影し、写真をレポートに
   貼り付けてください。
   アーム動作の流れが分かりやすいよう特徴的なタイミングを
   選び、複数の写真を時系列順に貼り付けてください。
1. ロボットが意図通りに動いたかを考察してください。
   - 意図通りに動いたとしたら、どのような状況からそう判断したかを
     画像を参照しながら説明してください。
   - 意図通りに動かなかったとしたら、その理由を説明し改善方法を
     考えて挙げてください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-4 title="課題6-4" %}



## RVizで指定された点にある棒を倒すプログラム
これまでに作成したロボットを移動させるプログラムとアームを動かす
プログラムを組み合わせて、地図領域内に立てた棒をアームで倒す
プログラムを作ってみましょう。

ロボット本体の接触ではなくアームの動作で倒せるよう考えてみてください。
棒の近くまでの移動は直線的に行えるものと仮定しても構いません。
見通せない場所にある棒にも接近できる工夫があれば加点対象となります。
アイデアや作戦をレポートに記載してください。

シミュレーション用に細い棒を設置した環境を用意してあります。
Gazebo起動時に `gazebo_manipulator_stage_1_obstacles.launch` の
launchファイルを指定することでその環境を利用できます。

プログラムの雛形として `go_to_clicked_point_and_move_arm.py` という
ファイルを用意してありますのでこれも活用してください。

{:id="exercise6-5"}
{% capture exercise6-5 %}
1. 「RVizで棒の立っている位置を指定するとその棒の近くに移動し、
   アームを伸ばしてから回転させて棒を倒すプログラム」を作成し
   レポートに貼り付けてください。
1. プログラム作成時のアイデアや目標(棒の近くに移動する、
   アーム動作で棒を倒す)を達成するための作戦についての説明を
   レポートで報告してください。
1. 作成したプログラムを実行し、その様子を報告してください。
   レポートには下記の情報を含めてください。
   - 様子を説明する文章
     (添付画像を適宜参照すると伝わりやすいです。
     説明しやすい状況となるよう添付する画像を選ぶと
     より伝わりやすいレポートとなります。)
   - 動作前のRVizの画像と、動作前の状況の画像
     (シミュレーションならGazeboの画像、実機実験なら写真)
   - 動作後のRVizの画像と、動作後の状況の画像
     (シミュレーションならGazeboの画像、実機実験なら写真)
   - 意図通りに動作しなかった場合は、その状況が分かりやすい
     RVizの画像と周囲状況の画像
     (シミュレーションならGazeboの画像、実機実験なら写真)
1. 動作結果について考察を行いレポートで報告してください。
   読者に伝わりやすいよう、添付画像を適宜参照すること。
   - 意図通りに動いたとしたら、動作のどの部分からそう判断したかを
     文章で報告してください。
   - 意図通りに動かなかったとしてもその理由を考えて、
     理由や改善するための案などを文章で報告してください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-5 title="課題6-5" %}



## RVizで指定された点にある棒を倒すプログラム(連続動作版)
{:id="exercise6-6"}
{% capture exercise6-6 %}
1. [課題6-5](#exercise5-6) で作成したプログラムをもとに、
   「RVizで位置を指定するとその位置の棒を倒す」という一連の動作を
   繰り返し行えるプログラムを作成し、レポートに貼り付けてください。
1. プログラム作成時のアイデアや目標(棒の近くに移動する、
   アーム動作で棒を倒す)を達成するための作戦についての説明を
   レポートで報告してください。
1. 作成したプログラムを実行し、その様子を報告してください。
   - 様子を説明する文章
     (添付画像を適宜参照すると伝わりやすいです。
     説明しやすい状況となるよう添付する画像を選ぶと
     より伝わりやすいレポートとなります。)
   - 動作前のRVizの画像と、動作前の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 動作後のRVizの画像と、動作後の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 意図通りに動作しなかった場合は、その状況が分かりやすい
     RVizの画像と周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
1. 動作結果について考察を行いレポートで報告してください。
   - 意図通りに動いたとしたら、動作のどの部分からそう判断したかを
     文章で報告してください。
   - 意図通りに動かなかったとしてもその理由を考えて、
     理由や改善するための案などを文章で報告してください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-6 title="課題6-6" %}

{: .notice--info}
1回の動作後、アームを伸ばしたまま移動しようとすると障害物に接触する
可能性があります。



## 発展課題 円形障害物を検知しその近くに移動するプログラム
上記の課題では倒すべき棒の位置は人がRVizで指定していましたが、
ロボットのセンサにも棒は写るのでセンサで得られた情報から棒を
発見することも原理的には可能です。
実験室のLinux環境には、センサ情報から円筒状の障害物を検出するための
ROSプログラムを導入してあります。
これを使って、ロボット自身が棒を発見してその近くに移動する
動作を行うプログラムを作ってみましょう。

円筒状の障害物を検知するためのROSプログラムは下記のコマンドで実行できます。
```bash
$ rosrun exp3 obstacle_detector.launch
```
これを実行すると、検出された円筒状障害物の情報が
`/tracked_obstacles` というトピックに出力されるようになります。
これは[obstacle_detector](https://github.com/tysik/obstacle_detector)という
ROSパッケージの機能です。
`go_to_obstacle.py` にプログラムの雛形が用意してあり、
このトピックから情報を取得する例を載せてあります。
詳細は https://github.com/tysik/obstacle_detector を参照してください。

{:id="exercise6-ex1"}
{% capture exercise6-ex1 %}
1. センサ情報から円筒状の物体を検知しその近くに移動する
   プログラムを作成し、レポートに貼り付けてください。
1. プログラム作成時のアイデアや目標(棒を検知する、棒の近くに移動する)を
   達成するための作戦についての説明をレポートで報告してください。
1. 作成したプログラムを実行し、その様子を報告してください。
   - 様子を説明する文章
     (添付画像を適宜参照すると伝わりやすいです。
     説明しやすい状況となるよう添付する画像を選ぶと
     より伝わりやすいレポートとなります。)
   - 動作前のRVizの画像と、動作前の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 動作後のRVizの画像と、動作後の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 意図通りに動作しなかった場合は、その状況が分かりやすい
     RVizの画像と周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
1. 動作結果について考察を行いレポートで報告してください。
   - 意図通りに動いたとしたら、動作のどの部分からそう判断したかを
     文章で報告してください。
   - 意図通りに動かなかったとしてもその理由を考えて、
     理由や改善するための案などを文章で報告してください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-ex1 title="発展課題6-EX1" %}



## 発展課題 円形障害物を検知しそれを倒すプログラム
{:id="exercise6-ex2"}
{% capture exercise6-ex2 %}
1. センサ情報から棒(円筒状の物体)を検知しその近くに移動した後に
   アームを動作させて棒を倒す
   プログラムを作成し、レポートに貼り付けてください。
1. プログラム作成時のアイデアや目標(棒を倒せる位置に移動する、棒を倒すなど)を
   達成するための作戦についての説明をレポートで報告してください。
1. 作成したプログラムを実行し、その様子を報告してください。
   - 様子を説明する文章
     (添付画像を適宜参照すると伝わりやすいです。
     説明しやすい状況となるよう添付する画像を選ぶと
     より伝わりやすいレポートとなります。)
   - 動作前のRVizの画像と、動作前の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 動作後のRVizの画像と、動作後の周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
   - 意図通りに動作しなかった場合は、その状況が分かりやすい
     RVizの画像と周囲状況の画像(シミュレーションならGazeboの画像、実機実験なら写真)
1. 動作結果について考察を行いレポートで報告してください。
   - 意図通りに動いたとしたら、動作のどの部分からそう判断したかを
     文章で報告してください。
   - 意図通りに動かなかったとしてもその理由を考えて、
     理由や改善するための案などを文章で報告してください。

   どちらの場合でもその他に気付いた点があれば文章で報告してください。
{% endcapture %}
{% include phyexp3-exercise.html content=exercise6-ex2 title="発展課題6-EX2" %}

{: .notice--info}
プログラムの雛形を`go_to_obstacle_and_move_arm.py`として用意してあります。
