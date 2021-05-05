# Error_ellipse_SLAM

## Error_ellipse_ SLAMについて

Error_ellipse_ はSLAM学習用プログラムです。
2D レーザスキャナのデータ（スキャン）とオドメトリデータを格納したファイルを入力し、
ロボット位置の軌跡と 2D 点群地図を gnuplot 上に出力します。

Error_ellipse_SLAMは、スキャンマッチングに基づく位置合せ、レーザスキャナとオドメトリのセンサ融合、
Graph-based SLAM に基づくループ閉じ込みなどの要素技術から構成されています。

## 実行環境

Error_ellipse_SLAMはプログラミング言語 C++で記述されています。
動作を確認した実行環境は下記のものです。いずれも 64 ビット版です。

|           OS           |                      C++                       |
| :--------------------: | :--------------------------------------------: |
|       Windows 7        | Visual C++ 2013 (Visual Studio Community 2013) |
|       Windows 10       | Visual C++ 2015 (Visual Studio Community 2015) |
| Linux Ubuntu 14.04 LTS |                   gcc 4.8.4                    |
| Linux Ubuntu 16.04 LTS |                   gcc 5.4.0                    |

32 ビット OS での動作確認はしていないので、必要な場合はご自分で試してください。

## 必要なソフトウェア

Error_ellipse_SLAMの実行には、下記のソフトウェアが必要です。

| ソフトウェア |          内容           | バージョン |
| :----------: | :---------------------: | :--------: |
|    Boost     |    C++汎用ライブラリ    |   1.58.0   |
|    Eigen3    |   線形代数ライブラリ    |   3.2.4    |
|   gnuplot    |    グラフ描画ツール     |    5.0     |
|    CMake     |    ビルド支援ツール     |   3.2.2    |
|     p2o      | Graph-based SLAM ソルバ |    beta    |

バージョンは Error_ellipse_SLAM の開発で使用したものであり、明確な条件ではありません。
これ以上のバージョンであれば通常は動作します。
これ以下のバージョンでも動作する可能性はあります。

## データセット

| ファイル名          | 内容               |
| :------------------ | :----------------- |
| corridor.lsc        | 廊下（単一ループ） |
| corridor-degene.lsc | 廊下（退化）       |


![Mike_SLAM_fuse](https://user-images.githubusercontent.com/65348333/117123258-c59b6e00-add1-11eb-8e61-f75f2ebeb37e.gif)


![Mike_SLAM_odo](https://user-images.githubusercontent.com/65348333/117123452-fed3de00-add1-11eb-80a9-124065c94f9c.gif)
