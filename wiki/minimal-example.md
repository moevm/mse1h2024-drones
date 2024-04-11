# Инструкции по запуску минимального примера

Ниже заголовки описывают то что нужно сделать в общем виде. Содержимое же описывает инструкции конкретно для Ubuntu.

## Установить ROS 2 Iron

Подробности описаны [по ссылке](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).

## Установить пакет `webots-ros2`

`$ sudo apt install ros-iron-webots-ros2`

## Собрать пакет

Действия ниже показаны от корня проекта.

```
$ cd aruco_follower_ws/
$ source /opt/ros/iron/setup.bash
$ colcon build
```

## Убедиться, что все работает

В папке созданного пакета открыть 2 новых терминала.

### В первом терминале запустить пакет

```
$ source install/setup.bash
$ ros2 launch aurco_follower robot_launch.py
```

### Во втором -- отправить команду на взлет

```
$ source /opt/ros/iron/setup.bash
$ ros2 topic pub /fly std_msgs/Bool "data: true"
```
