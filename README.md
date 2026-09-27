# MT_2026_reserve_odometry
## 1. Описание модуля

Описание модуля представлено по ссылке:

https://github.com/VirusFOSTER/MT_2026_reserve_odometry/tree/main/docs

## 2. Интерфейсы

**Входные топики**
| **Имя** | **Тип** | **Описание** |
|---|---|---|
| `/vehicle/front_bogie_velocity` | message_filters::Subscriber<tram_vehicle_msgs::msg::VelocitySensor | Топик с исходными данными (скорость вращения передних колес)  |
| `/vehicle/rear_bogie_velocity` | message_filters::Subscriber<tram_vehicle_msgs::msg::VelocitySensor | Топик с исходными данными (скорость вращения задних колес)  |
| `/vehicle/driver_position_cmd` | message_filters::Subscriber<tram_vehicle_msgs::msg::DriverControllerCommand | Топик с исходными данными (положение ручки контролера водителя)  |

**Входные топики (временные - на первые секунды)**
| **Имя** | **Тип** | **Описание** |
|---|---|---|
| `/sensing/gnss/master/fix` | sensor_msgs::msg::NavSatFix | Топик с исходными данными (позиция по GNSS (master))  |
| `/sensing/gnss/rover/fix` | sensor_msgs::msg::NavSatFix | Топик с исходными данными (позиция по GNSS (rover))  |

**Выходные топики**
| **Имя** | **Тип** | **Описание** |
|---|---|---|
| `/result/velocity` | tram_vehicle_msgs::msg::VelocitySensor | Топик с вычисленной скоростью движения трамвая  |
| `/result/position` | nav_msgs::msg::Odometry | Топик с вычисленным положением трамвая |

## 3. Cборка и запуск проетка

Если пользователь не добавлен в группу docker, то можно предварительно выполнить команду

```bash
sudo usermod -aG docker $USER
```

Либо выполнять все последующие команды по сборке и запуску docker-образа через sudo

### Сборка докера (при необходимости)

Выполнить скрипт:


```bash
bash ./docker_build.sh
```


В результате будет собран докер образ

### Запуск контейнера (докер)

Выполнить скрипт:

```bash
bash ./docker_run.sh
```

В результате будет запущен контейнер с поддержкой графики (можно использовать RVIZ)

### Сборка проекта

```bash
# Сборка всех пакетов
colcon build --symlink-install

# Сборка отдельно взятого пакета reserve_odometry
colcon build --packages-select reserve_odometry --symlink-install

# Настройка переменных окружения текущей оболочки для поиска пакетов ROS 2
source install/setup.bash
```

### Запуск проекта

```bash
# Запуск модуля резервной одометрии
ros2 run reserve_odometry reserve_odometry_node

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Для прослушивания выходных топиков необходимо в отдельном терминале выполнить следующее:

```bash
docker exec -ti ros2_dev bash

source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 bag play <наименование bag-файла>
```

После начала проигрывания bag-файла запущенная нода reserve_odometry будет публиковать сообщения в выходные топики

### Примечание

Если установлен ros2 запуск докера не обязателен
