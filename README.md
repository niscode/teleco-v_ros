# このフォルダの用途
created by ©︎niscode

ここではTeleco-Vにて、
- gmappingを用いたSLAM
- dwa（障害物回避）と自己位置推定（amcl）を用いたナビゲーションなどを実行するためのlaunchファイルが格納されます。

実行に必要なパッケージ群は以下の通りです。
- `sudo apt install ros-melodic-rosserial`
- `sudo apt install ros-melodic-slam-gmapping`
- `sudo apt install ros-melodic-navigation`


まずは `roscore` を実行した状態で、以下に進む。

<br>

> ### SLAMを実行する場合
<br>

`roslaunch teleco-v_ros gmapping.launch`

上を起動後、rvizで表示される地図を見ながら、
X-boxコントローラなどでTeleco-Vを操作し地図を作成していく。  

ある程度地図が完成したら、任意のディレクトリ直下に移動し、mapファイルを保存する。

`rosrun map_server map_server -f gamp_3f`

実行したディレクトリ直下にyamlファイルとpgmファイルがそれぞれ"gmap_3f"という名前で保存される。

<br>

> ### ナビゲーションを実行する場合
<br>

以下を起動することで、rviz上にSLAMで作成した地図とTelecoの自己位置のパーティクルが表示される。  

＊作成した地図のディレクトリは `navigation.launch` ファイルを編集して正しく指定されているか確認。

`roslaunch teleco-v_ros navigation.launch`

rviz上でTelecoの初期位置と目標位置をそれぞれマウス操作で与える。

<br>

> ### ナビゲーション実行時の目的地座標を調べる
<br>

`rosmsg show geometry_msgs/PoseStamped`

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
