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

# Остановка контейнера
```bash
docker-compose down
```

