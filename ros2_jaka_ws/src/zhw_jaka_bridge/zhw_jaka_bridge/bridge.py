import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robot_msgs.msg import RobotMsg
from robot_msgs.srv import Move
from std_srvs.srv import Empty
import roslibpy
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py import message_to_ordereddict

class JakaBridge(Node):
    def __init__(self):
        super().__init__(node_name='zhw_jaka_bridge')  # type: ignore

        # 初始化roslibpy与ROS1上的rosbridge_server通信
        ros1_host = 'localhost'
        ros1_port = 9090
        self.get_logger().info('Try to connect to ROS1 rosbridge_server at %s:%d' % (ros1_host, ros1_port))
        self.ros1_client = roslibpy.Ros(host=ros1_host, port=ros1_port)
        self.ros1_client.run()
        self.get_logger().info('Connected to ROS1 rosbridge_server at %s:%d' % (ros1_host, ros1_port))

        # robot_driver/tool_point的publisher
        self.tool_point_publisher = self.create_publisher(TwistStamped, 'robot_driver/tool_point', 10)

        # robot_driver/robot_states的publisher
        self.robot_states_publisher = self.create_publisher(RobotMsg, 'robot_driver/robot_states', 10)

        # robot_driver/stop_move的service
        self.stop_move_service = self.create_service(
            Empty, 'robot_driver/stop_move', self.handle_stop_move)  # type: ignore

        # 底层的ROS1 robot_driver/stop_move的Service
        self.base_stop_move_service = roslibpy.Service(self.ros1_client, 'robot_driver/stop_move', 'std_srvs/Empty')

        # robot_driver/move_line的service
        self.move_line_service = self.create_service(
            Move, 'robot_driver/move_line', self.handle_move_line)  # type: ignore

        # 底层的ROS1 robot_driver/move_line的service
        self.base_move_line_service = roslibpy.Service(self.ros1_client, 'robot_driver/move_line', 'robot_msgs/Move')

        # robot_driver/tool_point的listener，从ROS1上的robot_driver/tool_point订阅，并发布到ROS2上的robot_driver/tool_point
        self.tool_point_listener = roslibpy.Topic(self.ros1_client, 'robot_driver/tool_point',
                                                  'geometry_msgs/TwistStamped')
        self.tool_point_listener.subscribe(self.publish_tool_point)

        # robot_driver/robot_states的listener，从ROS1上的robot_driver/robot_states订阅，并发布到ROS2上的robot_driver/robot_states
        self.robot_states_listener = roslibpy.Topic(
            self.ros1_client, 'robot_driver/robot_states', "robot_msgs/RobotMsg")
        self.robot_states_listener.subscribe(self.publish_robot_states)

    def handle_stop_move(self, req: Empty.Request, rsp: Empty.Response):
        ros1Request = roslibpy.ServiceRequest()
        ros1Response = self.base_stop_move_service.call(ros1Request)
        return rsp  # std_srvs/Empty, so no need to use ros1Response

    def handle_move_line(self, req: Move.Request, rsp: Move.Response):
        od = message_to_ordereddict(req)
        ros1Request = roslibpy.ServiceRequest(values=od)
        ros1Response = self.base_move_line_service.call(ros1Request)
        set_message_fields(rsp, ros1Response)  # type: ignore
        return rsp

    def publish_tool_point(self, data):
        # don't remove these three lines!!!
        # because TwistStamped datatype has some difference between ROS1 and ROS2, we need to convert it
        data['header'].pop('seq')
        data['header']['stamp']['sec'] = data['header']['stamp'].pop('secs')
        data['header']['stamp']['nanosec'] = data['header']['stamp'].pop('nsecs')

        msg = TwistStamped()
        set_message_fields(msg, data)  # 将roslibpy的消息转换为ROS2的消息
        self.tool_point_publisher.publish(msg)

    def publish_robot_states(self, data):
        msg = RobotMsg()
        set_message_fields(msg, data)  # 将roslibpy的消息转换为ROS2的消息
        self.robot_states_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = JakaBridge()
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
