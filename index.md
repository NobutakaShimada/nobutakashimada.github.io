---
layout: splash
lang: en
ref: main
permalink: /
header:
  overlay_color:
  overlay_image:
  cta_label:
  cta_url:
  caption:
  intro:
    - excerpt: '実世界情報実験３ロボットテーマ'
without_content_table: true
---
<!-- {% assign GITHUB_PAGES_BRANCH = "master" %} -->
{% assign COLAB_PREFIX = "https://colab.research.google.com/github/"
   | append: site.repository | append: "/blob/"
   | append: GITHUB_PAGES_BRANCH %}

{: .nonumber}
# 立命館大学情報理工学部 実世界情報実験３ロボットテーマレジュメ

{: .counter-style-upper-alpha}
# 2025年度春学期実験用レジュメ
- [受講上の注意（最初に目を通すこと）](/docs/remarks)

1. [基本編(Turtlebot3の操作とROSの基礎)](/docs/basic)
1. [Gazeboシミュレータでのロボット操作](/docs/gazebo)
1. [ロボット実機との接続と移動操作](/docs/turtlebot3)
1. [環境地図の自動マッピング(SLAM)](/docs/slam)
1. [生成した環境地図に基づくナビゲーション](/docs/navigation)
1. [Pythonプログラミングによる移動指示とマニピュレータ操作](/docs/patrol)
1. [（予備）OpenMANIPULATOR-Xを使ったマニピュレーション](/docs/manipulation)
1. [2023立命館慶祥高校R-CONFERENCE体験授業コンテンツ](/docs/keisho2023)

{: .counter-style-upper-alpha}
# レポートファイル（docxファイル）
- [レポートについて](/docs/report)
- 2025年度からレポートをLaTeX(overleaf)にする予定です。後日説明をします。
- ~~[レポートWordファイル１（ワードファイル）](2024exp3report1-3.docx)
  ([課題1](/docs/basic), [課題2](/docs/gazebo), [課題3](/docs/turtlebot3))~~
- ~~[レポートWordファイル２（ワードファイル）](2024exp3report4-5.docx)
 ([課題4](/docs/slam), [課題5](/docs/navigation))~~
- ~~[レポートWordファイル３（ワードファイル）](2024exp3report6.docx)~~
  ([課題6](/docs/patrol))
#- ~~[レポートipynbファイル１ (Google Colaboratoryで開く)]({{ "2023exp3report1-3.ipynb" | relative_url | prepend: COLAB_PREFIX}})~~
#  ~~([課題1](/docs/basic), [課題2](/docs/gazebo), [課題3](/docs/turtlebot3))~~
#- ~~[レポートipynbファイル２ (Google Colaboratoryで開く)]({{ "2023exp3report4-5.ipynb" | relative_url | prep#end: COLAB_PREFIX}})~~
#  ~~([課題4](/docs/slam), [課題5](/docs/navigation))~~
#- ~~[レポートipynbファイル３ (Google Colaboratoryで開く)]({{ "2023exp3report6.ipynb" | relative_url | prepend: COLAB_PREFIX}})~~
#  ~~([課題6](/docs/patrol))~~

{: .notice--info}
上記のipynbファイルへのリンクを開くと
Google Colaboratory上にノートブックが表示されますが
**そのまま保存することはできません**。
「画面上部の `ドライブにコピー` を指示する」か
「メニューの `ファイル` から `保存` を選択して
`ドライブにコピーを保存` を指示する」
のいずれかを行い、 **Google ドライブに保存する必要があります** 。
その際にGoogleアカウントにログインしていない状態であれば
ログインを求められますので **自分のGoogleアカウントでログイン** してください。

{: .counter-style-upper-alpha}
# 参考URL
- [gedit Text Editor (English)](https://help.gnome.org/users/gedit/stable/)
- [Turtlebot3 e-Manual(English)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [OpenMANIPULATOR-X e-Manual(English)](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)
- [ROS公式ドキュメント](http://wiki.ros.org/ja/ROS) 
<!-- （[ミラー１](https://ghostarchive.org/ros/wiki.ros.org/ja.html)）（[ミラー２](http://mirror.umd.edu/roswiki/ja.html)）-->
- [ROS公式 トピックの理解](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingTopics)
- [ROS公式 サービスとパラメータの理解](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams)
- [ROS公式ページヘのアクセスができないときのarchive.orgによるキャッシュ版ROSチュートリアル](http://web.archive.org/web/20200920235105/https://wiki.ros.org/ja/ROS/Tutorials)
- [Gazebo公式(English)](http://gazebosim.org/)
- [ROS Navigationスタックのわかりやすいかつ詳しい解説記事](https://qiita.com/MoriKen/items/0b75ab291ab0d95c37c2)
  - [AMCLによる自己位置推定の原理説明](https://qiita.com/MoriKen/items/dfb6eb168649873589f0)
  - [gmappingによる地図生成の原理説明](https://qiita.com/MoriKen/items/0f2550a2adbdcd3da04e)
  - [ROSナビゲーションのチューニングガイド（英文）](https://kaiyuzheng.me/documents/navguide.pdf)
- [MoveIt公式(English)](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

{: .counter-style-upper-alpha}
# ROBOTIS製教材レジュメ

- [Week 1-1](/docs/week1-1)
- [Week 1-2](/docs/week1-2)
- [Week 2-1](/docs/week2-1)
- [Week 2-2](/docs/week2-2)
- [Week 3-1](/docs/week3-1)
- [Week 3-2](/docs/week3-2)
- [Week 4](/docs/week4)

{: .counter-style-upper-alpha}
# メンテナンス用
- [backup script](backup.sh)
- [launch file updates](launch.tar.gz)


