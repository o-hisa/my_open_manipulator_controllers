# OpenMANIPULATOR Controls 

## 概要
OpenMANIPULATOR-Xの制御パッケージに力制御のプログラムを実装したものです。
プログラムのベースは[open_manipulator_controllers](https://github.com/ROBOTIS-GIT/open_manipulator_controls/tree/master)です。
各々の動かし方は以下のとおりです。

## 変更点
基本的に変更部はopen_manipulator_controllersです。
以下の2つが実装されています。
- force_control_based_on_joint_angle_controller
  - モーターの軸に対してパラメータを設定して力制御を行う
- force_control_based_on_tip_position_controller
  - 手先位置に対してパラメータを設定して力制御を行う


## How to Run
```bash
(MoveGroup + JointTrajectoryController)
# Gazebo Simulation
$ roslaunch open_manipulator_controllers joint_trajectory_controller.launch

# Real Robot
$ roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false

(ForceControlBasedOnJointAngleController)
# Real Robot
# モーターの軸に対してパラメータを設定して力制御を行う
# 外力検知が実装されていないので、目標軌道の追従とバネ・ダンパ特性に相関があり、目標軌道をより追従させようとすると、バネ・ダンパ特性が強くなります。
$ roslaunch open_manipulator_controllers force_control_based_on_joint_angle_controller.launch sim:=false

(ForceControlBasedOnTipPositionController)
# Real Robot
# 手先位置に対してパラメータを設定して力制御を行う
# 外力検知が実装されていないので、目標軌道の追従とバネ・ダンパ特性に相関があり、目標軌道をより追従させようとすると、バネ・ダンパ特性が強くなります。
$ roslaunch open_manipulator_controllers force_control_based_on_tip_position_controller.launch sim:=false
```

力制御に関しては、USBのレイテンシー変更により、1msecの制御周期だと想定しています。
以下のコマンドから、該当のUSBポートのレイテンシーを変更してください。
```bash
$ sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
$ echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
