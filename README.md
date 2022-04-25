# ic120_ros

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
  $ sudo apt install python-vcstool python-rosdep python-catkin-tools
  $ git clone https://github.com/strv/vcstool-utils.git
  $ rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
  $ ./vcstool-utils/import_all.sh -s .rosinstall ~/catkin_ws/src
  $ catkin build
  $ source ../devel/setup.bash
  ```

### ハードウェアシステム
![ic120_hardware_system](https://user-images.githubusercontent.com/24404939/159679362-c82d3720-089a-47f1-9251-a02f9e8a7fd4.jpg)
