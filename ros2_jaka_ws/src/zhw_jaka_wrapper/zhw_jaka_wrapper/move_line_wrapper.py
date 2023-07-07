import rclpy
from rclpy.node import Node
from zhw_msgs.srv import WrappedMove
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from robot_msgs.srv import Move
from robot_msgs.msg import RobotMsg
from rclpy.executors import MultiThreadedExecutor
import time
from datetime import datetime

EPS = 1e-4


class MoveLineWrapperService(Node):
    def __init__(self):
        super().__init__('move_line_wrapper')  # type: ignore
        self.cbg = ReentrantCallbackGroup()
        self.exclusive_cb_group = ReentrantCallbackGroup()

        self.service = self.create_service(srv_type=WrappedMove, srv_name='move_line_wrapper',
                                           callback=self.handle_wrapped_move, callback_group=self.exclusive_cb_group)  # type: ignore
        self.stop_move_cli = self.create_client(Empty, '/robot_driver/stop_move', callback_group=self.cbg)
        self.base_move_cli = self.create_client(Move, '/robot_driver/move_line', callback_group=self.cbg)

        self.tool_point_sub = self.create_subscription(TwistStamped, '/robot_driver/tool_point',
                                                       self.on_receive_tool_point, 1, callback_group=self.cbg)  # type: ignore
        self.robot_state_sub = self.create_subscription(RobotMsg, '/robot_driver/robot_states',
                                                        self.on_receive_robot_state, 1, callback_group=self.cbg)  # type: ignore

        self.latest_tool_point = TwistStamped()
        self.latest_robot_state = RobotMsg()

        self.get_logger().info('move_line_wrapper started')

        self.move_vel = self.declare_parameter('move_vel').get_parameter_value().double_value  # m/s，末端爪子的移动速度
        self.move_acc = self.declare_parameter('move_acc').get_parameter_value().double_value  # m/s^2，末端爪子的移动加速度

    async def on_receive_tool_point(self, tool_point: TwistStamped):
        self.latest_tool_point = tool_point
        self.latest_tool_point_time = datetime.now()

    async def on_receive_robot_state(self, robot_state: RobotMsg):
        self.latest_robot_state = robot_state
        self.latest_robot_state_time = datetime.now()

    async def handle_wrapped_move(self, req: WrappedMove.Request, rsp: WrappedMove.Response):

        # 首先停止之前的移动
        self.stop_move_cli.wait_for_service()
        base_stop_move_rsp = await self.stop_move_cli.call_async(Empty.Request())

        # 获得移动前爪子的位置
        orig_ts = self.latest_tool_point
        # 将移动前位置和目标位置比较，判断是否需要移动
        need_move = (abs(req.pose[0] - orig_ts.twist.linear.x) > EPS) or \
            (abs(req.pose[1] - orig_ts.twist.linear.y) > EPS) or \
            (abs(req.pose[2] - orig_ts.twist.linear.z) > EPS) or \
            (abs(req.pose[3] - orig_ts.twist.angular.x) > EPS) or\
            (abs(req.pose[4] - orig_ts.twist.angular.y) > EPS) or \
            (abs(req.pose[5] - orig_ts.twist.angular.z) > EPS)

        # 尝试移动爪子
        # 等待底层的move_line服务开启
        self.base_move_cli.wait_for_service()
        base_move_line_req = Move.Request()
        base_move_line_req.mvacc = self.move_acc
        base_move_line_req.mvvelo = self.move_vel
        base_move_line_req.pose = req.pose
        base_move_line_rsp = await self.base_move_cli.call_async(base_move_line_req)

        # 等待爪子移动完成，如果robot_state的mode不是3，说明运动完成
        STATE_MOVING = 3
        STATE_ERROR = 4
        while True:
            time.sleep(1.0)
            robot_state = self.latest_robot_state
            if robot_state.state != STATE_MOVING:
                break

        # 获得移动后爪子的位置
        curr_ts = self.latest_tool_point
        # 判断爪子是否移动
        ever_moved = (abs(curr_ts.twist.linear.x - orig_ts.twist.linear.x) > EPS) or (abs(curr_ts.twist.linear.y - orig_ts.twist.linear.y) > EPS) or (abs(curr_ts.twist.linear.z - orig_ts.twist.linear.z) >
                                                                                                                                                      EPS) or (abs(curr_ts.twist.angular.x - orig_ts.twist.angular.x) > EPS) or (abs(curr_ts.twist.angular.y - orig_ts.twist.angular.y) > EPS) or (abs(curr_ts.twist.angular.z - orig_ts.twist.angular.z) > EPS)
        # 爪子在移动过程中是否遇到错误
        ever_error = robot_state.state == STATE_ERROR
        # 爪子是否移动成功
        move_success = not need_move or (ever_moved and not ever_error)

        # 输出响应
        rsp = WrappedMove.Response()
        if move_success:
            rsp.success = True
            if not need_move:
                rsp.message = 'success, no need to move, because the target pose is the same as the original pose'
            else:
                rsp.message = 'success'
        else:
            rsp.success = False
            if not ever_moved:
                rsp.message = 'not moved, maybe the target pose is unreachable'
            if ever_error:
                rsp.message = 'move error, maybe has encountered obstacle when moving'
        return rsp


def main(args=None):
    rclpy.init(args=args)
    service = MoveLineWrapperService()
    # rclpy.spin(service)
    # rclpy.shutdown()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(service)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
