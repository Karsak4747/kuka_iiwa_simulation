# Dockerfile для ROS 2 и плагина iiwa_ros2 с зависимостями MoveIt
FROM osrf/ros:humble-desktop-full

# Обновляем пакеты и устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    git wget curl x11-apps \
    build-essential \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-move-group

# Копирование и устанавливка плагина iiwa_ros2
# COPY iiwa_ros2 /root/ros2_ws/src/iiwa_ros2

# Устанавливаем зависимости, если они указаны в package.xml
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ICube-Robotics/iiwa_ros2.git
RUN apt-get update && rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y && pip install pyyaml && apt-get install nano


WORKDIR /root/ros2_ws
# Сборка ROS2 пакетов и плагинов
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --parallel-workers 4"

# Настройка среды
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc



RUN /bin/bash -c "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/iiwa_ros2"

# Копирование файлов для конкурса
COPY trajectories /root/trajectories
# Запуск контейнера в интерактивном режиме с шеллом
WORKDIR /root/ros2_ws
ENTRYPOINT ["xeyes"]
