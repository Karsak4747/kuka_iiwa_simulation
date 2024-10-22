# Dockerfile для ROS 2 и плагина iiwa_ros2 с зависимостями MoveIt
FROM osrf/ros:humble-desktop-full

# Обновляем пакеты и устанавливаем необходимые зависимости
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-moveit \
    ros-humble-moveit-common \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-move-group

# Клонируем и устанавливаем плагин iiwa_ros2
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/ICube-Robotics/iiwa_ros2.git

# Устанавливаем зависимости, если они указаны в package.xml
WORKDIR /root/ros2_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && pip install pyyaml


# Сборка ROS2 пакетов и плагинов
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --parallel-workers 4"

# Настройка среды
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/iiwa_ros2"
COPY trajectories /root/trajectories
# Запуск контейнера в интерактивном режиме с шеллом
CMD ["/bin/bash"]
