## 1. セットアップ

vmware (macならfusion)で imageから起動する

user: robond
pass: robo-nd

```bash
$ sudo apt-get update
$ sudo apt-get upgrade
```

### catkin_wsの作成

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

## 2. simple_arm

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/simple_arm.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ sudo apt-get install ros-kinetic-controller-manager
$ rosdep check simple_arm
$ rosdep install -i simple_arm
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> .bashrc
```

### 実行

gazeboを立ち上げる

```bash
$ roslaunch simple_arm robot_spawn.launch
```

joint1/joint2をコントロールする

```bash
$ rostopic pub -1 /simple_arm/joint_1_position_controller/command std_msgs/Float64 "data: 1.5"
$ rostopic pub -1 /simple_arm/joint_2_position_controller/command std_msgs/Float64 "data: 1.5"
```

カメラからのimage streamをみる

```bash
$ rosrun image_view image_view image:=/rgb_camera/image_raw
```

## 3. RoboND-Kinematics-Project

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/msrks/RoboND-Kinematics-Project.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models" >> .bashrc
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod u+x target_spawn.py
$ sudo chmod u+x IK_server.py
$ sudo chmod u+x safe_spawner.sh
```

### 実行

#### 1) forward kinematics

```bash
$ roslaunch kuka_arm forward_kinematics.launch
```

#### 2) pick and place demo

demo flag is set to "true" in `inverse_kinematics.launch file under /RoboND-Kinematics-Project/kuka_arm/launch`

```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

#### 3) pick and place (non demo)：自分のIK_serverで動かす

demo flag is set to "false" in `inverse_kinematics.launch file under /RoboND-Kinematics-Project/kuka_arm/launch`

```bash
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/
$ sh ik_test.sh
```

### おまけ（プロジェクトを眺める用）

Kuka_arm

```bash
$ roslaunch kuka_arm load_urdf.launch
```

tf: tf_echo

```
$ rosrun tf tf_echo [reference frame] [target frame]
$ rosrun tf tf_echo base_link link_6
```

joint_state_publisher
