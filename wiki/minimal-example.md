# Инструкции по запуску минимального примера

## Необходимые приготовления

Описанное ниже, к сожаление не претендует ни на что более, чем works on my machine, поскольку взаимодействие всех компонентов очень хрустальное.

1. Установить ROS 2.
1. Пользуясь [подсказкой](https://gazebosim.org/docs/harmonic/ros_installation#summary-of-compatible-ros-and-gazebo-combinations) выбрать и установить совместимую версию Gazebo.
1. Установить `ros_gz_bridge`. Его версия должна быть совместимой с установленной версией ROS 2.
1. Создать рабочую среду ROS 2.
1. Скопировать папку `gazebo_test` в `<workspace>/src`, где `<workspace>` -- название созданной рабочей среды.
1. Разрешить зависимости `gazebo_test`.
1. Собрать рабочую среду.

## Непосредственно запуск

1. Запустить симуляцию `visualize_lidar` в Gazebo.
1. Связать при помощи `ros_gz_bridge` сообщения типа `geometry_msgs/msg/Twist`, напрвляемые в канал `/model/vehicle_blue/cmd_vel`, с типом `ignition.msgs.Twist` в Gazebo.
1. Запустить скрипт `gazebo_test.gazebo_commander`.
1. Наслаждаться тем, как синяя тележка в симуляции поехала вперед.
