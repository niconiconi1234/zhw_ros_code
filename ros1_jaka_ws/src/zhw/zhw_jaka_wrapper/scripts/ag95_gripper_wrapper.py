import rospy
import socket
from zhw_msgs.srv import SetGripperPosition, SetGripperPositionRequest, SetGripperPositionResponse
gripper_sock: socket.socket = None  # type: ignore


def set_gripper_pose_handler(req: SetGripperPositionRequest) -> SetGripperPositionResponse:
    target = req.target_position  # 夹爪的目标位置
    target = max(min(target, 100), 20)  # 将目标位置限制在20~100之间
    retry = 3  # 最多尝试3次
    for i in range(retry):
        hex_target = hex(int(target))[2:].upper()  # 将目标位置转换成16进制字符串
        GRASP_COMMD = 'FFFEFDFC0106020100{}000000FB--(Set Position {})'.format(hex_target, target)
        GRASP_RECV = 'FFFEFDFC0106020100{}000000FB'.format(hex_target)

        gripper_sock.sendall(GRASP_COMMD.encode())
        recv_data = gripper_sock.recv(2048).decode()
        rospy.sleep(1)

        if recv_data == GRASP_RECV:
            info = f'gripper set to {target}'
            rospy.loginfo(info)
            rsp = SetGripperPositionResponse()
            rsp.message = info
            rsp.success = True
            return rsp
        else:
            rospy.logerr('failed to set gripper position')
            rospy.logerr(f'recv: {recv_data}')
            rospy.logerr(f'expected: {GRASP_RECV}')
            rospy.logerr(f'retrying {retry - i} times')

    # 彻底失败
    rsp = SetGripperPositionResponse()
    rsp.message = 'failed to set gripper position after {} retries'.format(retry)
    rsp.success = False
    return rsp


def connect_gripper(ip, port):
    rospy.loginfo(f'trying to connect to arm gripper at {ip}:{port}')
    gripper_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 我也不知道为什么要有下面这些参数，但是学长给的Python2代码里有，修改成Python3的代码的时候我就照抄了
    gripper_sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
    gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
    gripper_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)

    gripper_sock.connect((ip, port))
    return gripper_sock


def init_gripper():
    global gripper_sock
    # 初始化夹爪时需要发送的命令，具体什么含义我也不知道
    INIT_CMD = 'FFFEFDFC010802010000000000FB'
    INIT_RECV = 'FFFEFDFC010802010000000000FB'

    gripper_sock.sendall(INIT_CMD.encode())
    rospy.sleep(3)
    recv_data = gripper_sock.recv(2048).decode()

    if recv_data == INIT_RECV:
        rospy.loginfo('gripper initialized')
    else:
        rospy.logerr('failed to initialize gripper')
        raise Exception('failed to initialize gripper')


def main():
    global gripper_sock
    rospy.init_node('ag95_gripper_wrapper')
    rospy.loginfo('ag95_gripper_wrapper node started')

    ag95_gripper_ip = rospy.get_param('~ag95_gripper_ip', 'None')
    ag95_gripper_port = rospy.get_param('~ag95_gripper_port', 0)

    # 尝试连接arm gripper
    try:
        gripper_sock = connect_gripper(ag95_gripper_ip, ag95_gripper_port)
    except Exception as e:
        rospy.logerr(f'failed to connect to arm gripper at {ag95_gripper_ip}:{ag95_gripper_port}')
        rospy.logerr(e)
        return

    # 连接成功
    rospy.loginfo(f'connected to arm gripper at {ag95_gripper_ip}:{ag95_gripper_port}')

    # 初始化夹爪
    try:
        init_gripper()
    except Exception as e:
        rospy.logerr(e)
        return

    # 初始化夹爪成功
    s = rospy.Service('set_gripper_pose', SetGripperPosition, set_gripper_pose_handler)
    rospy.spin()


if __name__ == '__main__':
    main()
