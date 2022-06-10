---
layout: phyexp3-lesson
lang: en
permalink: /docs/patrol/
section_number: 6
section_title: Pythonプログラムを用いた移動指示とマニピュレータ操作
sidebar:
  title: Pythonプログラミングによる移動指示とマニピュレータ操作
  nav: "phyexp3_patrol"
---

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

ファイルの内容を確認する最も単純なコマンドとして `cat` コマンドが
あります。
下記のようなコマンドで `print_clicked_point.py` の内容を確認する
ことができます。
```bash
$ cat ~/exp3_ws/src/exp3/scripts/print_clicked_point.py
```
このコマンドは端末上に内容が出力された後に終了し、検索したり遡って
見ることはできません(端末自身のスクロールバーである程度は遡れます)。

上記のコマンドでは `print_clicked_point.py` の場所を
[絶対パス(absolute path)](https://e-words.jp/w/%E7%B5%B6%E5%AF%BE%E3%83%91%E3%82%B9.html) で指定しています(フルパス, full pathとも呼ばれる)。
絶対パスは非常に長いので、
**ディレクトリを移動して現在のディレクトリからの[相対パス](https://e-words.jp/w/%E7%9B%B8%E5%AF%BE%E3%83%91%E3%82%B9.html)で指定**
する方法もあります。
以下の例では `~/exp3_ws/src/exp3/scripts` というディレクトリに
移動してから相対パスで `cat` コマンドを起動しています。
```bash
$ cd ~/exp3_ws/src/exp3/scripts
$ cat print_clicked_point.py
```

上記では `cd` コマンドでディレクトリを移動しています。
実験室のLinux環境ではあらかじめ `exp3` というROSパッケージを
登録してありますので `roscd` で移動することもできます。
例えば下記のように `roscd` コマンドを使うと上記と同じようなことが
できます。
```bash
$ roscd exp3/scripts
$ cat print_clicked_point.py
```

ファイルの内容を確認するには `cat` コマンドの他にも色々な方法があります。
以下にその例を示します。
- `less` コマンドで表示する。
  ```bash
  $ less ~/exp3_ws/src/exp3/scripts/print_clicked_point.py
  ```
  `cat` と異なり矢印キーや`j`, `k`キーの
  入力で表示位置を移動できます。`q`を入力するとコマンドを終了します。
  (参考: [manpage of less](http://manpages.ubuntu.com/manpages/bionic/ja/man1/less.1.html) )

  相対パスを使う場合は以下のようになります。
  ```bash
  $ roscd exp3/scripts
  $ less print_clicked_point.py
  ```

- エディタ(gedit)で開く。
  ```bash
  $ gedit ~/exp3_ws/src/exp3/scripts/print_clicked_point.py &
  ```
  端末とは別にウィンドウが開いて表示、編集できます。
  行末の「&」はバックグラウンド実行(端末とは別に同時平行的に実行される)を
  意味します。

  相対パスを使う場合は以下のようになります。
  ```bash
  $ roscd exp3/scripts
  $ gedit print_clicked_point.py &
  ```

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

### Pythonプログラムのインタラクティブな実行
Pythonは [インタプリタ](https://ja.wikipedia.org/wiki/%E3%82%A4%E3%83%B3%E3%82%BF%E3%83%97%E3%83%AA%E3%82%BF) なので、
**プログラムをその場で入力しながら実行** することができます。

`print_clicked_point.py` をインタラクティブ(interactive)実行して
`point` という変数にどういう情報がどのように入っているか確認して
みましょう。
`~/exp3_ws/src/exp3/scripts/` のディレクトリに移動してから
`python -i` コマンドでプログラムを実行してみてください。
```bash
$ roscd exp3/scripts
$ python -i print_clicked_point.py
```
実行後、 `Waiting for a point to be clicked...` のメッセージが
表示されてからRVizの **Publish Point** の機能で適当な位置を指定
してください。
そうすると `/clicked_point` というROS topicに送られた情報が
`point` 変数に代入された状態でPythonプログラムの入力を待ち受ける
状態になります。
下記の行のそれぞれを実行して、 `point` 変数内の情報にアクセスする
方法を確認してみてください。
~~~ python
print(point)

print(point.header)

print(point.point)

print(point.point.x)
~~~
プログラムの動作確認やデバッグ、改良の際にインタラクティブ実行は
大変便利です。
以下の課題に取り組む際も是非活用してください。



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
    rospy.init_node('go_to_fixed_point')
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

`teleop.launch` を実行しているとモータの挙動が干渉してしまうので、
まずは `teleop.launch` が動いていない状態にしてください。
地図上の適当な位置の座標を確認し、 `go_to_fixed_point.py` を編集して
その座標を `point_xyz` に設定してください。
その状態で
```bash
$ rosrun exp3 go_to_fixed_point.py
```
を実行し、ロボットが指定した位置に移動することを確認してください。
また、目標位置の座標を変更して複数回実験を行い、座標に応じた位置に
移動していることを確認してください。

### 目標位置付近で停止せず回転し続ける問題
上記のプログラムでロボットに移動指示を出すと、目標位置付近まで
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
| /move_base/DWAPlannerROS/max_vel_theta | 1.2 | 6.4 | [解説](http://wiki.ros.org/dwa_local_planner#line-463) (旧称 max_rot_vel として記載)|
| /move_base/DWAPlannerROS/min_vel_theta | 1.0 | 4.6 | [解説](http://wiki.ros.org/dwa_local_planner#line-468) (旧称 min_rot_vel として記載)|

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


### 課題

{:id="exercise6-2"}
{% capture exercise6-2 %}
1. 地図上でロボットが到達可能な移動先を設定し、目標位置の `map` frameに
   おける座標をレポートで報告してください。
1. 変数 `point_xyz` に目標位置の `map` frameにおける座標が代入される
   ように `go_to_fixed_point.py` を修正して、
   修正後のプログラムをレポートに貼り付けてください。
1. プログラム動作中に `rqt_graph` を実行し、その画像をレポートに貼り
   付けてください。
   加えて、作成したプログラム実行中のノード間の情報のやりとりに
   ついて貼り付けた画像を参照しながら説明する文章を作成しレポートで
   報告してください。

   ROSのaction機構による情報の送受信が `rqt_graph` の画面上でどの
   ように表示されるかについては
   [`rqt_graph`とactionについてのヒント](#exercise6-2-hint)を
   参照してください。
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

{: .notice--info id="exercise6-2-hint"} 
**`rqt_graph`とactionについてのヒント:**\\
`go_to_fixed_point.py` がロボットを移動させる際にはROSの
[actionという機能](http://wiki.ros.org/actionlib#Overview)を
使っています。
これは他のROS node(プログラム)に何らかの指示(リクエスト)を送り、
その指示の実行状態(実行中である、完了した、失敗した、など)を
監視できるようにするための機能です。
この機能に関するリクエストやステータス(状態)の送受信にはROS topicが
使われていて、具体的なtopic名やその機能は
[ROS Wikiのactionlib/DetailedDescriptionのページ](http://wiki.ros.org/actionlib/DetailedDescription#Action_Interface_.26_Transport_Layer)
に説明があります。
例えばロボットを移動させるための `/move_base` という action の場合
`/move_base/goal`, `/move_base/status` などのtopicが使われます。
\\
但し、これらのactionに関連するtopicは `rqt_graph` 上では
**個別には表示されず**、
**/move_base/action_topicsのようにまとめて表示される**ので注意してください。

{: .notice--info}
**プログラム中で使用しているクラスやメソッドについて:**\\
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
1. プログラム動作中に `rqt_graph` を実行し、その画像をレポートに貼り
   付けてください。
   加えて、作成したプログラム実行中のノード間の情報のやりとりに
   ついて貼り付けた画像を参照しながら説明する文章を作成しレポートで
   報告してください。

   ROSのaction機構による情報の送受信が `rqt_graph` の画面上でどの
   ように表示されるかについては
   [`rqt_graph`とactionについてのヒント](#exercise6-2-hint)を
   参照してください。
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
### アーム動作の前準備
以下に紹介するプログラムでロボットのアームを動かす場合は事前に
`navigation.launch` を地図情報付きで起動しておく必要があります。
これはロボット本体は移動させない場合でも必要で、Gazebo環境と実機環境の
どちらでも必要です。

また、実機環境でアームを動かすには `robot_manipulation.launch` も
起動しておく必要もあります。
以下に必要な手順とコマンド例をまとめます。

#### Gazeboの場合
1. `roscore`を起動する。
1. Gazeboを起動する。

   起動コマンドの例:
   ```bash
   $ roslaunch exp3 gazebo_manipulator_stage_4.launch
   ```

1. Gazeboのシミュレーション環境の時間を開始させる。

   GazeboのPlayボタンをクリックするか、下記のコマンドを実行する。
   ```bash
   $ rosservice call gazebo/unpause_physics
   ```

1. `navigation.launch` を起動する。

   起動コマンドの例:
   ```bash
   $ roslaunch exp3 navigation.launch map_name:=stage4
   ```

#### 実機の場合
1. `roscore`を起動する。
1. `machine.launch` を起動する。

   起動コマンドの例:
   ```bash
   $ roslaunch exp3 machine.launch id:=25
   ```

1. `navigation.launch` を起動する。

   起動コマンドの例:
   ```bash
   $ roslaunch exp3 navigation.launch map_name:=stage4
   ```

1. `robot_manipulation.launch` を起動する。

   起動コマンドの例:
   ```bash
   $ roslaunch exp3 robot_manipulation.launch
   ```

### `move_arm.py`
実験室のLinux環境の `~/exp3_ws/src/exp3/scripts/` のディレクトリに
`move_arm.py` という名前のプログラムを用意してあります。
このプログラムでは3つの関数が定義されていて、それぞれの機能は以下の
通りです。
-  `rotate_arm()` : アームを回転させる。
-  `extend_arm()` : アームを伸ばす。
-  `initialize_arm()` : アームを初期姿勢に戻す。

これらの関数は `robot` オブジェクトのメソッドを使って実装されています。
`robot` オブジェクトのメソッドのいくつかの機能を以下に示します。
これらのメソッドは `exp3_turtlebot.py` で定義されています。
- `robot.follow_joint_trajectory(joint_waypoints)` : 引数で与えられた
  関節角度列の通りにアームの関節を動かす。
- `robot.get_current_joint_positions()` : 現在の関節角度を返す。
  返り値は4つの値を持つリストで、それぞれ `joint1` から `joint4` の
  関節角度(radian単位)に対応しています。

`move_arm.py` は以下のようにして実行できますが、そのままでは
何もしないプログラムとなっています。
```bash
$ rosrun exp3 move_arm.py
```
まずは上記の3つの関数を使ってアームを動かすようプログラムを修正して
どのような動作が行われるか確認してください。
ロボット実機でアームを動作させる際には事前に
`robot_manipulation.launch` を起動しておく必要があります。
```bash
$ roslaunch exp3 robot_manipulation.launch
```

インタラクティブ実行を用いて `rotate_arm(robot)` などを実行させて
動作を確認することもできます。
```bash
$ roscd exp3/scripts
$ python -i move_arm.py
```
現在のアームの姿勢の情報は `robot` オブジェクトの
`get_current_joint_positions()` メソッドで取得する
こともできます。
~~~ python
print(robot.get_current_joint_positions())
~~~

`move_arm.py` の内容は以下の通りです。
~~~ python
#!/usr/bin/env python
import rospy
import math

import geometry_msgs.msg
import exp3_turtlebot3

def rotate_arm(robot):
    # Each entry means a pair of joint angles[rad] and time[sec].
    joint_waypoints = [
        [{'joint1': -0.85*math.pi}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)
    rospy.sleep(3)
    joint_waypoints = [
        [{'joint1': 0.85*math.pi}, 6.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)
    rospy.sleep(3)
    joint_waypoints = [
        [{'joint1': 0.0}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)

def extend_arm(robot):
    joint_waypoints = [
        [{'joint2': 0.25*math.pi, 'joint3': -0.25*math.pi}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)

def initialize_arm(robot):
    # Initial joint angles of the robot on the Gazebo environment.
    joint_waypoints = [
        [{'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0}, 3.0]
    ]
    robot.follow_joint_trajectory(joint_waypoints)

def initialize_arm_real(robot):
    # Initial joint angles of the real robot
    joint_waypoints = [
        [{'joint1': -0.0015339808305725455,
          'joint2': -1.575398325920105,
          'joint3': 1.2317866086959839,
          'joint4': 0.6089903712272644
         }, 3.0]
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
1. プログラム動作中に `rqt_graph` を実行し、その画像をレポートに貼り
   付けてください。
   加えて、作成したプログラム実行中のノード間の情報のやりとりに
   ついて貼り付けた画像を参照しながら説明する文章を作成しレポートで
   報告してください。

   ROSのaction機構による情報の送受信が `rqt_graph` の画面上でどの
   ように表示されるかについては
   [`rqt_graph`とactionについてのヒント](#exercise6-2-hint)を
   参照してください。
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
Gazebo起動時に `gazebo_manipulator_stage_1_obstacles.launch` や
`gazebo_manipulator_stage_4_obstacles.launch` の
launchファイルを指定することでその環境を利用できます。

プログラムの雛形として `go_to_clicked_point_and_move_arm.py` という
ファイルを用意してありますのでこれも活用してください。

{: .notice--info}
**ヒント1:** \\
指定された物体位置の手前で停止するにはいくつかの方法があります。
その方法のひとつとして現在位置と物体位置の
関係を見て、その間の点に移動目標位置を設定するというものがあります。
ロボットの現在位置は `robot.get_current_point()` で
取得することができます。
このメソッドの返り値の型は `geometry_msgs.msg.PointStamped` で
`/clicked_point` のtopicで取得できるものと同じ型です。
この返り値から現在位置の座標を取り出して物体位置を結ぶ直線を求め、
その直線上で物体位置に十分近い点の座標を計算し、その座標を移動の
目標位置とすれば物体の手前で停止させられます。

{: .notice--info}
**ヒント2:** \\
物体位置の手前に目標位置を設定する他にも
「物体に十分近付いたら停止する」という方法もあります。
`robot.admissible_distance` はこのような動作を実現するための変数です。
例えばこの変数に 0.2 という値を設定しておくと目標位置との距離が 0.2[m]
以下になったときに `robot.go_to_goal()` の処理が終了して次の
処理を行うことができます。
但し、目標位置は有効なままなので移動し続ける状態となります。
そこで `robot.go_to_goal()` の次の命令として目標位置を取り消して
停止させるメソッド `robot.stop_move_base_and_cancel_goals()` を
呼び出すことで、当初設定した目標位置の手前で停止させることができます。
これを用いれば物体位置の近くを目標位置として移動させて、十分近付いたら
停止させるという動作が実現できます。
\\
**!!注意!!** \\
物体位置をそのまま `robot.go_to_goal()` での移動目標位置に
設定すると経路計画を立てられず、移動できない場合があります。
ロボットのセンサが物体を捉えるとその物体の位置に障害物が
存在すると認識されるので
「移動目標位置に障害物が存在する状態」となります。
これにより「障害物に接触せずに移動目標位置に到達することが不可能」
となってそれ以降の経路計画が立てられなくなるため、
移動できなくなってしまいます。
\\
物体位置そのものを移動目標とするのではなく、そこから少し
ずれた位置を目標とするなどの工夫が必要です。
例えば棒からの距離0.3[m]の範囲は障害物がないことを前提として
その範囲の適当な点を移動目標とする、などです。

### Gazebo環境の物体配置初期化
Gazeboによるシミュレーション環境で棒を倒した後、棒を再度立てるのは
操作が難しいので、続けて実験する際には物体配置の初期化を行って
ください。

Gazeboのウィンドウの `Edit` メニューの中に `Reset Model Poses`
という項目があります。
これを選択するとシミュレーション環境内の全ての物体(ロボット含む)が
初期配置に戻り、続けて実験することができます。
この項目と同じ効果を持つ下記のコマンドを実行することでも物体配置を
初期化できます。
```bash
$ rosservice call /gazebo/reset_world
```

`Edit` メニューの中には `Reset World` という項目もありますが
こちらはシミュレーション環境の **時計も初期化してしまいます** 。
その影響でRViz上でロボットとアームが外れた状態になるなど
ロボット情報を管理するROSノードなどの動作がおかしくなります。
**メニューの `Reset World` は選択しないように注意してください**。

### 課題

{:id="exercise6-5"}
{% capture exercise6-5 %}
1. 「RVizで棒の立っている位置を指定するとその棒の近くに移動し、
   アームを伸ばしてから回転させて棒を倒すプログラム」を作成し
   レポートに貼り付けてください。
1. プログラム作成時のアイデアや目標(棒の近くに移動する、
   アーム動作で棒を倒す)を達成するための作戦、アルゴリズムについての
   説明をレポートで報告してください。
   アルゴリズムが前提としている仮定(ロボットの初期位置から棒の位置
   までは一直線で移動できる、など)があればそれを明示すること。

   フローチャートなどの画像を使うと伝わりやすいです。
   レポートのノートブックには適宜セルを追加して図などの説明資料を
   挿入し、それを参照しながら説明する文章を作成してください。
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
   アーム動作で棒を倒す、これらの動作を繰り返す)を達成するための
   作戦、アルゴリズムについての説明をレポートで報告してください。
   アルゴリズムが前提としている仮定(ロボットの初期位置から棒の位置
   までは一直線で移動できる、など)があればそれを明示すること。

   フローチャートなどの画像を使うと伝わりやすいです。
   レポートのノートブックには適宜セルを追加して図などの説明資料を
   挿入し、それを参照しながら説明する文章を作成してください。
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
$ roslaunch exp3 obstacle_detector.launch
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
   達成するための作戦、アルゴリズムについての説明をレポートで報告して
   ください。
   アルゴリズムが前提としている仮定(ロボットの初期位置から棒の位置
   までは一直線で移動できる、など)があればそれを明示すること。

   フローチャートなどの画像を使うと伝わりやすいです。
   レポートのノートブックには適宜セルを追加して図などの説明資料を
   挿入し、それを参照しながら説明する文章を作成してください。
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
1. プログラム作成時のアイデアや目標(棒を検知する、棒を倒せる位置に
   移動する、棒を倒すなど)を
   達成するための作戦、アルゴリズムについての説明をレポートで報告
   してください。
   アルゴリズムが前提としている仮定(ロボットの初期位置から棒の位置
   までは一直線で移動できる、など)があればそれを明示すること。

   フローチャートなどの画像を使うと伝わりやすいです。
   レポートのノートブックには適宜セルを追加して図などの説明資料を
   挿入し、それを参照しながら説明する文章を作成してください。
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
