# Инструкции по запуску симуляции

Ниже заголовки описывают то что нужно сделать в общем виде. Содержимое же описывает инструкции конкретно для Ubuntu.

## Установить Webots

Подробности описаны [по ссылке](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt).

## Установить пакет `webots-ros2`

`$ sudo apt install ros-iron-webots-ros2`

## Собрать пакет

Действия ниже показаны от корня проекта.

```
$ cd aruco_follower_ws/
$ source /opt/ros/iron/setup.bash
$ colcon build
```

## Запустить симуляцию

Открыть новый терминал.

```
$ source install/setup.bash
$ ros2 launch aruco_follower robot_launch.py
```