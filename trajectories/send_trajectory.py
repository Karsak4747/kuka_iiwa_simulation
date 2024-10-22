import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml
import sys

def send_trajectory_from_yaml(yaml_file):
    # Инициализация ROS2
    rclpy.init()

    # Создание ноды и клиента для отправки действий
    node = rclpy.create_node('send_trajectory')
    client = ActionClient(node, FollowJointTrajectory, '/iiwa_arm_controller/follow_joint_trajectory')

    # Чтение YAML-файла
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    # Ожидание готовности клиента
    client.wait_for_server()

    # Создание сообщения с траекторией
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = data['trajectory']['joint_names']

    for point in data['trajectory']['points']:
        traj_point = JointTrajectoryPoint()
        traj_point.positions = point['positions']
        traj_point.time_from_start.sec = point['time_from_start']['sec']
        traj_point.time_from_start.nanosec = point['time_from_start']['nanosec']
        goal_msg.trajectory.points.append(traj_point)

    # Отправка траектории
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    # Закрытие ноды
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 send_trajectory.py <path_to_yaml_file>")
        sys.exit(1)

    yaml_file = sys.argv[1]
    send_trajectory_from_yaml(yaml_file)
