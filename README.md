# Решение задачи Pick&Place


Выполнили: Кузин, Сеньковски, Печуркин
## Замечание

Справа указано использование html и других странных форматов - скорее всего косяк при заливке не через консоль, а через десктоп гитхаба. 

## Установка зависимостей и пакетов

**Проверка и обноление пакетов**

`sudo apt update`

`sudo apt upgrade`

`rosdep update`

**Установка пакетов panda_robot**

Перейдите по ссылке и выполните инструкции 

[Здесь](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)

**Также в catkin\src**

необходимо склонировать репозиторий 

`git clone https://github.com/frankaemika/franka_ros.git`

## Замена файлов

Взять из репозитория папку `panda_moveit_config` и заменить ее на ту что есть.

Затем в пакете `franka_ros` аналогичным образом заменить `franka_gazebo`

Возможно понадобится сделать 

`catkin_make`

## Запуск

`roslaunch panda_moveit_config demo_gazebo.launch world:=$(rospack find panda_moveit_config)/world/team_world.sdf`

**Запуск скрипта управления**

`cd`

`cd catkin_ws/src/control_robot_script/scripts/`

**Вспомогательный скрипт для отоброжения положения звеньев в [rad]**

`python3 check_joint_position.py`

**Запустить симуляцию pick&place**

`python3 cycle_script.py`





