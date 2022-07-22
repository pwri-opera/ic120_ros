# ic120_ros
OPERA対応クローラダンプIC120の土木研究所公開ROSパッケージ群

## 概説
- 国立研究開発法人土木研究所が公開するOPERA（Open Platform for Eathwork with Robotics Autonomy）対応のクローラダンプであるIC120用のROSパッケージ群である
- 本パッケージに含まれる各launchファイルを起動することで、実機やシミュレータを動作させるのに必要なROSノード群が立ち上がる
- 動作確認済のROS Version : ROS Melodic Morenia + Ubuntu 18.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．以下、新規作成するワークスペースの名称を"catkin_ws"と仮定して表記）
  ```bash
  $ cd ~/
  $ mkdir --parents catkin_ws/src
  $ cd catkin_ws
  $ catkin init
  $ catkin build
  ```

- 依存パッケージ群をインストールした上でパッケージのビルドと自分のワークスペースをインストール環境上にOverlayする  
  [vcstoolに関する参考サイト](https://qiita.com/strv/items/dbde72e20a8efe62ef95)
  ```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/pwri-opera/ic120_ros.git
  $ sudo apt update
  $ sudo apt install python-vcstool python-rosdep python-catkin-tools
  $ git clone https://github.com/strv/vcstool-utils.git
  $ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
  $ ./vcstool-utils/import_all.sh -s .rosinstall ~/catkin_ws/src
  $ catkin build
  $ source ../devel/setup.bash
  ```

## 含有するサブパッケージ
### ic120_bringup:
- ic120の実機を動作させる際に必要なノード群を一括起動するためのlaunch用のサブパッケージ

### ic120_description:
- ic120用のロボットモデルファイル(urdf, xacro含む)群

### ic120_gazebo:
- ic120をgazeboシミュレータ上で動作させるのに必要なノード群を一括起動するためのlaunch用のサブパッケージ

### ic120_navigation:
- ic120の自動走行のためのライブラリ
- [Navigation Stack](http://wiki.ros.org/navigation) に準拠

### ic120_unity:
- OPERAのUnityシミュレータと連携するために必要なノード群を一括起動するためのlaunch用のサブパッケージ

## 各ROSノード群の起動方法
- 実機動作に必要なROSノード群の起動方法
  ```bash
  $ roslaunch ic120_bringup ic120_vehicle.launch
  ```
- 実機遠隔操作に必要なROSノード群の起動方法
  ```bash
  $ roslaunch ic120_bringup ic120_remote.launch
  ```
- Unityシミュレータとの連携に必要なROSノード群の起動方法
  ```bash
  $ roslaunch ic120_unity ic120_standby.launch
  ```
 
### ハードウェアシステム
![ic120_hardware_system](https://user-images.githubusercontent.com/24404939/159679362-c82d3720-089a-47f1-9251-a02f9e8a7fd4.jpg)

## ソフトウェアシステム
### roslaunch ic120_unity ic120_standy.launch実行時のノード/トピックパイプライン（rqt_graph）
![unity_launch](https://user-images.githubusercontent.com/24404939/175253675-c9fe28b6-398b-46c4-963f-aad9289c3c9b.png)

### roslaunch ic120_bringup ic120_vehicle.launch実行時のノード/トピックパイプライン（rqt_graph）
![Screenshot from 2022-07-22 17-20-43](https://user-images.githubusercontent.com/24404939/180416808-acab38d4-04b0-48aa-a50f-15a0c7be0808.png)

### roslaunch ic120_launch ic120_remote.launch実行時のノード/トピックパイプライン（rqt_graph）
![Screenshot from 2022-07-22 17-18-45](https://user-images.githubusercontent.com/24404939/180416944-6568d3dc-23ad-4508-9e3a-55f378f093f1.png)
