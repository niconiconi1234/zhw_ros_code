from rclpy.node import Node
from zhw_padbot_msgs.srv import NavigateToTargetPoint
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import requests


class PadBotNavigationServer(Node):

    def __init__(self):
        super().__init__('padbot_navigation_server')  # type: ignore
        self.cbg = ReentrantCallbackGroup()
        self.padbot_ip = self.declare_parameter('padbot_ip').get_parameter_value().string_value  # 派宝机器人的IP地址
        self.padbot_port = self.declare_parameter('padbot_port').get_parameter_value().integer_value  # 派宝机器人的导航控制程序的端口号
        self.create_service(NavigateToTargetPoint, 'navigate_to_target_point', self.handle_navigate_to_target_point, callback_group=self.cbg)

    def handle_navigate_to_target_point(self, req, rsp):
        # make http request to navigation app running on padbot, so that the padbot can start to navigate to target point
        try:
            nav_result = requests.post(f'http://{self.padbot_ip}:{self.padbot_port}/navigation', json={'targetPoint': req.target_point})
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation request to padbot: {e}')
            rsp.success = False
            rsp.message = f'Failed to send navigation request to padbot: {e}'
            return rsp
        rsp.success = nav_result.json()['success']
        rsp.message = nav_result.json()['message']
        return rsp


def main(args=None):
    rclpy.init(args=args)
    node = PadBotNavigationServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node=node)
    executor.spin()
    executor.shutdown()


if __name__ == '__main__':
    main()
