#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.duration import Duration  # для timeout

import json
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import UInt16, UInt8MultiArray
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
import math  # для вычисления кватернионов
import time  # для логирования времени


class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')
        self.declare_parameter('route_file', 'path/to/your/route.json') # Путь к JSON файлу
        self.route_file_path = self.get_parameter('route_file').get_parameter_value().string_value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Подписчики для выполнения действий
        self.lift_pub = self.create_publisher(UInt16, '/pwb/lift_target_height', 10)
        self.servos_pub = self.create_publisher(UInt8MultiArray, '/pwb/servos_target_angles', 10)

        self.current_waypoint_index = 0
        self.is_executing_route = False

        # Сервис или топик для запуска выполнения маршрута
        self.start_execution_service = self.create_service(
            Trigger, 'start_route_execution', self.start_execution_callback
        )
        self.get_logger().info(f'[{time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}] Node initialized. Waiting for route file: {self.route_file_path}')
        self.get_logger().info(f'[{time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())}] Waiting for NavigateToPose action server...')
        self.nav_client.wait_for_server()

    def start_execution_callback(self, request, response):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        if self.is_executing_route:
            self.get_logger().warn(f'[{timestamp}] Route execution is already in progress.')
            response.success = False
            response.message = 'Execution in progress'
            return response

        try:
            with open(self.route_file_path, 'r') as f:
                self.route_data = json.load(f)
            self.route_points = self.route_data.get('route', [])
        except FileNotFoundError:
            self.get_logger().error(f'[{timestamp}] Route file not found: {self.route_file_path}')
            response.success = False
            response.message = f'File not found: {self.route_file_path}'
            return response
        except json.JSONDecodeError as e:
            self.get_logger().error(f'[{timestamp}] Error parsing JSON file: {e}')
            response.success = False
            response.message = f'Invalid JSON: {e}'
            return response

        if not self.route_points:
            self.get_logger().warn(f'[{timestamp}] Route file is empty or has no valid points.')
            response.success = False
            response.message = 'No points in route'
            return response

        self.current_waypoint_index = 0
        self.is_executing_route = True
        response.success = True
        response.message = f'Starting execution of {len(self.route_points)} waypoints.'

        self.get_logger().info(f'[{timestamp}] Started executing route with {len(self.route_points)} points.')
        self.execute_next_waypoint()
        return response

    def execute_next_waypoint(self):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        if not self.is_executing_route or self.current_waypoint_index >= len(self.route_points):
            self.get_logger().info(f'[{timestamp}] Finished executing route or execution stopped.')
            self.is_executing_route = False
            return

        waypoint = self.route_points[self.current_waypoint_index]
        x = waypoint.get('x', 0.0)
        y = waypoint.get('y', 0.0)
        theta = waypoint.get('theta', 0.0)  # Угол в радианах

        # Создание сообщения цели для Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Убедись, что это соответствует твоей системе координат
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position = Point(x=x, y=y, z=0.0)

        # Преобразование угла theta в кватернион (w, x, y, z)
        # Для вращения вокруг Z: qx=qy=0, qz=sin(theta/2), qw=cos(theta/2)
        siny_cosp = math.sin(theta / 2.0)
        cosy_cosp = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=siny_cosp, w=cosy_cosp)

        self.get_logger().info(f'[{timestamp}] Sending goal to Nav2: ({x}, {y}, theta={theta}) for waypoint {self.current_waypoint_index}')

        # Отправка цели Nav2
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)

        # Установка обратных вызовов для завершения навигации
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'[{timestamp}] Goal was rejected by server')
                self.is_executing_route = False  # Остановить выполнение при ошибке
                return

            self.get_logger().info(f'[{timestamp}] Goal accepted by server, waiting for result')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'[{timestamp}] Exception in goal_response_callback: {e}')
            self.is_executing_route = False

    def get_result_callback(self, future):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        try:
            result = future.result().result
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'[{timestamp}] Successfully reached waypoint {self.current_waypoint_index}!')

                # Выполнение действия (если есть)
                waypoint = self.route_points[self.current_waypoint_index]
                action = waypoint.get('action', None)
                if action:
                    self.execute_action(action, waypoint)
                else:
                    # Если нет действия, сразу перейти к следующей точке
                    self.current_waypoint_index += 1
                    self.execute_next_waypoint()
            else:
                self.get_logger().error(f'[{timestamp}] Failed to reach waypoint {self.current_waypoint_index}. Status code: {status}')
                # Здесь можно добавить логику повтора или остановки
                self.is_executing_route = False
        except Exception as e:
            self.get_logger().error(f'[{timestamp}] Exception in get_result_callback: {e}')
            self.is_executing_route = False


    def execute_action(self, action_type, waypoint_data):
        """Выполняет специфичное действие в точке маршрута."""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        self.get_logger().info(f'[{timestamp}] Executing action: {action_type}')

        if action_type == "pause":
            duration_sec = waypoint_data.get('duration', 0)
            self.get_logger().info(f'[{timestamp}] Pausing for {duration_sec} seconds...')
            # Используем таймер для неблокирующей паузы
            self.timer = self.create_timer(duration_sec, self.continue_after_pause)

        elif action_type == "lift_move":
            target_height = waypoint_data.get('height', 0)
            msg = UInt16()
            msg.data = target_height
            self.lift_pub.publish(msg)
            self.get_logger().info(f'[{timestamp}] Sent lift height command: {target_height}')
            # После выполнения действия перейти к следующей точке
            self.current_waypoint_index += 1
            self.execute_next_waypoint()

        elif action_type == "servos_move":
            angles_list = waypoint_data.get('angles', [0, 0, 0, 0])  # 4 сервопривода
            if len(angles_list) != 4:
                self.get_logger().warn(f'[{timestamp}] Expected 4 servo angles, got {len(angles_list)}. Using default [0,0,0,0]')
                angles_list = [0, 0, 0, 0]
            msg = UInt8MultiArray()
            msg.data = angles_list
            self.servos_pub.publish(msg)
            self.get_logger().info(f'[{timestamp}] Sent servos angles command: {angles_list}')
            # После выполнения действия перейти к следующей точке
            self.current_waypoint_index += 1
            self.execute_next_waypoint()

        # Добавь другие типы действий по мере необходимости

    def continue_after_pause(self):
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        self.get_logger().info(f'[{timestamp}] Pause finished, continuing route...')
        # Уничтожить таймер после использования
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
            self.timer.destroy()
        # Пауза завершена, вызвать выполнение следующей точки
        self.current_waypoint_index += 1
        self.execute_next_waypoint()


def main(args=None):
    rclpy.init(args=args)

    node = RouteExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
