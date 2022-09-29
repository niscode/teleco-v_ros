# このパッケージの用途
created by ©︎niscode

ここではTeleco-Vにて、
- gmappingを用いたSLAM
- dwa（障害物回避）と自己位置推定（amcl）を用いたナビゲーションなどを実行するためのlaunchファイルが格納されます。

実行に必要なパッケージ群は以下の通りです。
- `sudo apt install ros-melodic-rosserial`
- `sudo apt install ros-melodic-slam-gmapping`
- `sudo apt install ros-melodic-navigation`
<br>
ROSワークスペースの『src』ディレクトリ直下に以下を追加<br>

- `git clone https://github.com/Slamtec/rplidar_ros.git`

＊複数のLiDARを使用する場合
- `git clone https://github.com/iralabdisco/ira_laser_tools.git`

＊＊このパッケージではデバイスファイル名を以下のように固定して使用します（[こちら](https://scrapbox.io/nishi-pro/%E3%80%90ROS%E3%80%91USB%E6%8E%A5%E7%B6%9A%E3%81%95%E3%82%8C%E3%81%9F%E3%83%87%E3%83%90%E3%82%A4%E3%82%B9%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB%E5%90%8D%E3%81%AE%E5%9B%BA%E5%AE%9A)を参照）
- RPLiDARM<br>
   -> Front_LRF<br>
   -> Rear_LRF<br>

- FT231X（駆動系）<br>
   -> ROVER_BOARD
   

<br><br>

まずは `roscore` を実行した状態で、以下に進む。

<br>

> ### SLAMを実行する
<br>

`roslaunch teleco-v_ros gmapping.launch`

上を起動後、rvizで表示される地図を見ながら、
X-boxコントローラなどでTeleco-Vを操作し地図を作成していく。  

ある程度地図が完成したら、任意のディレクトリ直下に移動し、mapファイルを保存する。

`rosrun map_server map_server -f gamp_3f`

実行したディレクトリ直下にyamlファイルとpgmファイルがそれぞれ"gmap_3f"という名前で保存される。

<br>

> ### ナビゲーションを実行する
<br>

以下を起動することで、rviz上にSLAMで作成した地図とTelecoの自己位置のパーティクルが表示される。  

＊作成した地図のディレクトリは `navigation.launch` ファイルを編集して正しく指定されているか確認。

`roslaunch teleco-v_ros navigation.launch`

rviz上でTelecoの初期位置と目標位置をそれぞれマウス操作で与える。

<br>

> ### ナビゲーション実行時の目的地座標を調べる

`rostopic echo /move_base_simple/goal`
<br><br>
Topicの中身は以下の通りで、目的地を与えるたびに更新される
- シーケンス番号
- タイムスタンプ
- 基準となる座標系
- ロボットの位置と姿勢  
<br>

```
std_msgs/Header header
   uint32 seq
   time stamp
   string frame_id

geometry_msgs/Pose pose
   geometry_msgs/Point position
     float64 x
     float64 y
     float64 z
   geometry_msgs/Quaternion orientation
     float64 x
     float64 y
     float64 z
     float64 w
```
