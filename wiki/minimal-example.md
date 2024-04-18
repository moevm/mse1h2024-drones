# Инструкции по запуску минимального примера

Ниже заголовки описывают то что нужно сделать в общем виде. Содержимое же описывает инструкции конкретно для Ubuntu.

## Установить ROS 2 Iron

Подробности описаны [по ссылке](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html).

## Установить пакет `webots-ros2`

`$ sudo apt install ros-iron-webots-ros2`

## Создать пакет с названием aurco_follower

```
$ source /opt/ros/iron/setup.bash
$ ros2 pkg create --build-type ament_python --node-name simple_mavic_driver aurco_follower --dependencies rclpy std_msgs webots_ros2_driver
```

В создавшийся пакет скопировать содержимое папки `aurco_follower` репозитория, заменяя исходное содержимое пакета, а именно:

- `aurco_follower`
- `launch`
- `resource`
- `test`
- `worlds`
- `LICENSE`
- `package.xml`
- `setup.py`

## Собрать пакет

`$ colcon build`

## Тестирование 

Запуск тестов

`$ colcon test`

## Убедиться, что все работает

В папке созданного пакета открыть 2 новых терминала.

### В первом терминале запустить пакет

```
$ source install/setup.bash
$ ros2 launch aurco_follower robot_launch.py
```

### Во втором -- отправить команду на взлет

```
$ source /opt/ros/iron/setub.bash
$ ros2 topic pub /fly std_msgs/Bool  "data: true"
```
