# Установка репозитория с github
```bash
git clone https://github.com/Karsak4747/kuka_iiwa_simulation.git
```

# Запуск контейнера и вход в него
```bash
sudo xhost +
docker-compose up -d
docker exec -it ros2_iiwa bash
```
## Запуск rviz c kuka iiwa
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_planning:=true
```

# Остановка контейнера
```bash
docker-compose down
```

