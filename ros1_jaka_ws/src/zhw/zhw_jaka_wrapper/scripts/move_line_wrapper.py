import rospy
from robot_msgs.srv import MoveRequest, MoveResponse, Move
from robot_msgs.msg import RobotMsg
from zhw_msgs.srv import WrappedMove, WrappedMoveRequest, WrappedMoveResponse
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty

EPS = 1e-4


def handle_move_line(req: WrappedMoveRequest) -> WrappedMoveResponse:
    move_vel = rospy.get_param('~move_vel', 0.5)  # m/s，末端爪子的移动速度
    move_acc = rospy.get_param('~move_acc', 0.3)  # m/s^2，末端爪子的移动加速度

    # 首先停止之前的移动
    rospy.wait_for_service("/robot_driver/stop_move")
    base_stop_move = rospy.ServiceProxy("/robot_driver/stop_move", Empty)
    base_stop_move_req = EmptyRequest()
    base_stop_move_rsp = base_stop_move(base_stop_move_req)

    # 获得移动前爪子的位置
    orig_ts: TwistStamped = rospy.wait_for_message('/robot_driver/tool_point', TwistStamped)  # type: ignore
    # 将移动前位置和目标位置比较，判断是否需要移动
    need_move = (abs(req.pose[0] - orig_ts.twist.linear.x) > EPS) or \
        (abs(req.pose[1] - orig_ts.twist.linear.y) > EPS) or \
        (abs(req.pose[2] - orig_ts.twist.linear.z) > EPS) or \
        (abs(req.pose[3] - orig_ts.twist.angular.x) > EPS) or\
        (abs(req.pose[4] - orig_ts.twist.angular.y) > EPS) or \
        (abs(req.pose[5] - orig_ts.twist.angular.z) > EPS)

    # 尝试移动爪子
    BASE_MOVE_LINE_SERVICE = '/robot_driver/move_line'
    rospy.wait_for_service(BASE_MOVE_LINE_SERVICE)  # 等待底层的move_line服务开启
    base_move_line = rospy.ServiceProxy(BASE_MOVE_LINE_SERVICE, Move)  # type: ignore
    base_move_line_req = MoveRequest()
    base_move_line_req.pose = req.pose
    base_move_line_req.mvacc = move_acc
    base_move_line_req.mvvelo = move_vel
    base_move_line_rsp = base_move_line(base_move_line_req)

    # 等待爪子移动完成，如果robot_state的mode不是3，说明运动完成
    STATE_MOVING = 3
    STATE_ERROR = 4
    while True:
        rospy.sleep(1.0)
        robot_state: RobotMsg = rospy.wait_for_message('/robot_driver/robot_states', RobotMsg)  # type: ignore
        if robot_state.state != STATE_MOVING:
            break

    # 获得移动后爪子的位置
    curr_ts: TwistStamped = rospy.wait_for_message('/robot_driver/tool_point', TwistStamped)  # type: ignore
    # 判断爪子是否移动
    ever_moved = (abs(curr_ts.twist.linear.x - orig_ts.twist.linear.x) > EPS) or (abs(curr_ts.twist.linear.y - orig_ts.twist.linear.y) > EPS) or (abs(curr_ts.twist.linear.z - orig_ts.twist.linear.z) >
                                                                                                                                                  EPS) or (abs(curr_ts.twist.angular.x - orig_ts.twist.angular.x) > EPS) or (abs(curr_ts.twist.angular.y - orig_ts.twist.angular.y) > EPS) or (abs(curr_ts.twist.angular.z - orig_ts.twist.angular.z) > EPS)
    # 爪子在移动过程中是否遇到错误
    ever_error = robot_state.state == STATE_ERROR
    # 爪子是否移动成功
    move_success = not need_move or (ever_moved and not ever_error)

    # 输出响应
    rsp = WrappedMoveResponse()
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

def main():
    rospy.init_node('move_line_wrapper', anonymous=True)
    rospy.loginfo(f'move_line_wrapper node started')
    s = rospy.Service('move_line_wrapper', WrappedMove, handle_move_line)
    rospy.spin()


if __name__ == '__main__':
    main()
