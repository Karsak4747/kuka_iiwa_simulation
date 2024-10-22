Запускается командой
```shell
docker-compose up --build
```
Для Windows нужно установить эту программу https://sourceforge.net/projects/vcxsrv/
Обязательно нужно поставить галочки:
- Multiple windows
- Display number: -1
- Start no client
- Clipboard
- Disable access control

Нстройка симуляции:
1. Текущее положение звеньев
```shell
ros2 topic echo /joint_states
```
2. Запуск траектории
```shell
cd ../trajectories
python3 send_trajectory.py trajectory.yml
```