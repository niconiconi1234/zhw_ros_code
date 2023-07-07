import rclpy
from rclpy.node import Node
import socket
import time
from zhw_msgs.srv import SetGripperPosition


class Ag95GripperWrapperService(Node):

    def __init__(self):
        super().__init__('ag95_gripper_wrapper')
        # 获取相关参数
        self.ip = self.declare_parameter('ag95_gripper_ip').get_parameter_value().string_value
        self.port = self.declare_parameter('ag95_gripper_port').get_parameter_value().integer_value

        # 连接并初始化夹爪
        self.connect_gripper()
        self.init_gripper()

        # service
        self.srv = self.create_service(SetGripperPosition, 'set_gripper_pose', self.set_gripper_pose_handler)

    def connect_gripper(self):
        self.get_logger().info(f'trying to connect to arm gripper at {self.ip}:{self.port}')
        self.gripper_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 我也不知道为什么要有下面这些参数，但是学长给的Python2代码里有，修改成Python3的代码的时候我就照抄了
        self.gripper_sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
        self.gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
        self.gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)

        self.gripper_sock.connect((self.ip, self.port))

    def init_gripper(self):
        # 初始化夹爪时需要发送的命令，具体什么含义我也不知道
        INIT_CMD = 'FFFEFDFC010802010000000000FB'
        INIT_RECV = 'FFFEFDFC010802010000000000FB'

        self.gripper_sock.sendall(INIT_CMD.encode())
        time.sleep(3)
        recv_data = self.gripper_sock.recv(2048).decode()

        if recv_data == INIT_RECV:
            self.get_logger().info('gripper initialized')
        else:
            self.get_logger().error('failed to initialize gripper')
            raise Exception('failed to initialize gripper')

    def set_gripper_pose_handler(self, req: SetGripperPosition.Request, rsp: SetGripperPosition.Response):
        target = req.target_position  # 夹爪的目标位置
        target = max(min(target, 100), 20)  # 将目标位置限制在20~100之间
        retry = 3  # 最多尝试3次
        for i in range(retry):
            hex_target = hex(int(target))[2:].upper()  # 将目标位置转换成16进制字符串
            GRASP_COMMD = 'FFFEFDFC0106020100{}000000FB--(Set Position {})'.format(hex_target, target)
            GRASP_RECV = 'FFFEFDFC0106020100{}000000FB'.format(hex_target)

            self.gripper_sock.sendall(GRASP_COMMD.encode())
            recv_data = self.gripper_sock.recv(2048).decode()
            time.sleep(1)

            if recv_data == GRASP_RECV:
                info = f'gripper set to {target}'
                self.get_logger().info(info)
                rsp.message = info
                rsp.success = True
                return rsp
            else:
                self.get_logger().error('failed to set gripper position')
                self.get_logger().error(f'recv: {recv_data}')
                self.get_logger().error(f'expected: {GRASP_RECV}')
                self.get_logger().error(f'retrying {retry - i} times')

        # 彻底失败
        rsp.message = 'failed to set gripper position after {} retries'.format(retry)
        rsp.success = False
        return rsp


def main(args=None):
    rclpy.init(args=args)
    service = Ag95GripperWrapperService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
