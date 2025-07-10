---
title: "TurtleBot3 ファームウェアアップデート手順"
date: 2025-07-09
---

# TurtleBot3 ファームウェアアップデート手順

TurtleBot3 のファームウェアを安定版に更新する方法についてまとめる．

---

## 一覧


1. [開発環境](#1-開発環境)
2. [アップデートの手順](#2-アップデートの手順)
3. [動作確認](#3-動作確認)
4. [トラブルシューティング](#4-トラブルシューティング)
5. [参考リンク](#5-参考リンク)

---

## 1. 開発環境
- 使用するハードウェア：TurtleBot3 Waffle Pi + OpenMANIPULATOR
- ROS： Kinetic Kame
- 書き込み先マイコン：OpenCR 1.0

## 2. アップデートの手順
- TurtleBot3に搭載されているRaspberry PiとOpenCRをUSBケーブルで接続する
- Raspberry Pi内のターミナルを起動する（対象のTurtlebot3のIDを指定してSSHで外部から入る or Raspberry PiにHDMIケーブルと画面を接続して直接入る）

  ```bash
  $ ssh 25 // IDが25のTurtlebot3にSSHで入る例
  ```
- Raspberry Piのホームディレクトリに`opencr_update`と`opencr_update.tar.bz2`が存在する場合はデフォルトでROS2に対応するファームウェアしか入っていないため，削除する．
  ```bash
  $ rm -R opencr_update
  $ rm -R opencr_update.tar.bz2
  ```
- ROS1に対応したファームウェアをダウンロードし展開する
  ```bash
  $ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
  $ tar -xvf opencr_update.tar.bz2
  ```
- ファームウェアを書き込む
  ```bash
  $ cd ./opencr_update
  $ ./update.sh /dev/ttyACM0 om_with_tb3.opencr // 本機の適合はom_with_tb3.opencr
## 3. 動作確認
### 以下の項目について確認する．
- 電源を入れなおした時にLiDARが回転し，ロボットアームが初期位置にセットされる
- `roslaunch exp3 teleop.launch`を実行してTurtlebot3を動作させることができる
## 4. トラブルシューティング
### ファームウェア書き込み時に失敗した
- Raspberry PiとOpenCRが確実にUSBケーブルで接続されているか確認する
- Raspberry Piで書き込んだ場合はOpenCRとコンピュータを接続し，直接ファームウェアを書き込む．書き込み方は[公式ページ](https://emanual.robotis.com/docs/en/parts/controller/opencr10_jp/)を参考にする
### ファームウェア書き込み後Turtlebot3がteleopコマンドやBluetoothコントローラで動かせない
- Turtlebot3に搭載されている全てのDynamixelがOpenCRに接続されているか確認する
## 5. 参考リンク
- [Turtlebot3 OpenCR setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/)
- [OpenCR 1.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10_jp/)
