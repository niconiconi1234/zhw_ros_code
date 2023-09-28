import rospy
from zhw_jaka_msgs.srv import JakaPickByMarkerId, JakaPickByMarkerIdRequest, JakaPickByMarkerIdResponse
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs  # do not remove this line!! or the transform will not work!!
from geometry_msgs.msg import PoseStamped
from zhw_jaka_msgs.srv import JakaMoveGripperToPose, JakaMoveGripperToPoseRequest, JakaMoveGripperToPoseResponse, JakaSetGripperTightness, JakaSetGripperTightnessRequest, JakaSetGripperTightnessResponse, JakaMoveJoints, JakaMoveJointsRequest, JakaMoveJointsResponse
import tf2_ros
import redis
import threading
import time


class MoveGripperToPoseOperation():
    def __init__(self, pose, step_name):
        self.pose = pose
        self.step_name = step_name


class MoveJointsOperation():
    def __init__(self, joint_values, step_name):
        self.joint_values = joint_values
        self.step_name = step_name


class SetGripperTightnessOperation():
    TIGHT = 1.0
    LOOSE = 0.0

    def __init__(self, tightness, step_name):
        self.tightness = tightness
        self.step_name = step_name


buffer = None
listener = None
move_gripper_to_pose = None
set_gripper_tightness = None
move_joints = None
redis_client = None

REDIS_JAKA_HEALTH_KEY = 'JAKA:HEALTH'
REDIS_JAKA_STATUS_KEY = 'JAKA:STATUS'
REDIS_JAKA_CUR_STEP_KEY = 'JAKA:CUR_STEP'

# 机械臂初始状态的各个关节角度
initial_joints = [-3.2287884849090203, 0.3889835751924734, 2.329802144868561, 0.443159886524249, -4.669364354537797, 2.3346812392809193]

# 机械臂释放物体时的各个关节角度
release_joints = [-1.9517921877559492, -1.1263589681693977, 1.0584953022395054, 3.2097573942477253, -4.432808216956338, 2.3318052443100754]


def handle_shelf2cart_byMarkerId(req: JakaPickByMarkerIdRequest):
    global buffer
    global listener
    global move_gripper_to_pose
    global set_gripper_tightness
    global move_joints

    markerId = req.markerId  # 要抓取的物品的markerId
    rospy.loginfo("shelf2cart_byMarkerId: recieved request: markerId = %d", markerId)

    all_markers: AlvarMarkers = rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)  # 当前摄像头看到的所有marker
    target_marker = None  # 要抓取物品的marker
    for marker in all_markers.markers:
        if marker.id == markerId:
            target_marker = marker
            break

    if target_marker is None:
        rospy.logwarn("shelf2cart_byMarkerId: cannot find marker with id = %d", markerId)
        return JakaPickByMarkerIdResponse(success=False, message=f'shelf2cart_byMarkerId: cannot find marker with id = {markerId}')

    # 物体上二维码的位姿，抓取物体时爪子的位置就是物体的位置，爪子姿态是写死的
    object_pose = target_marker.pose
    object_pose.header = target_marker.header
    object_pose.header.frame_id = 'camera_frame'
    object_pose.header.stamp = rospy.Time(0)
    object_pose = buffer.transform(object_pose, 'Link_0', rospy.Duration(1.0))

    # 发布机械臂的状态为busy
    redis_client.set(REDIS_JAKA_STATUS_KEY, 'busy')
    redis_client.set(REDIS_JAKA_CUR_STEP_KEY, 'Step 0: ready')

    # 动作列表
    atom_ops_list = [
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.LOOSE, step_name='Step1: Release Gripper'),  # 首先释放夹爪
        MoveJointsOperation(joint_values=initial_joints, step_name='Step2: Move the robot arm joint to the initial state'),  # 移动机械臂关节到初始状态
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x+0.1, object_pose.pose.position.y,
                                         object_pose.pose.position.z+0.04, -0.833, 1.541, 2.266],
                                   step_name='Step3: Move the gripper 0.1 meters in front of the object and prepare to grab it'),  # 移动爪子到物体前方0.1米，准备抓取
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x, object_pose.pose.position.y,
                                         object_pose.pose.position.z+0.04, -0.833, 1.541, 2.266],
                                   step_name='Step4: Move the gripper to the object and prepare to grab it'),  # 移动爪子到物体处，准备抓取
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.TIGHT,
                                     step_name='Step5: Grab the object'),  # 抓取物体
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x, object_pose.pose.position.y,
                                         object_pose.pose.position.z+0.05+0.04, -0.833, 1.541, 2.266],
                                   step_name='Step6: Lift the object 0.05 meters'),  # 将物体抬起0.05米
        MoveJointsOperation(joint_values=initial_joints, step_name='Step7: Return to initial state'),  # 回到初始状态
        MoveJointsOperation(joint_values=release_joints,
                            step_name='Step8: Move the robotic arm to the state of releasing the object'),  # 将机械臂移动到释放物体的状态
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.LOOSE, step_name='Step9: Release the object'),  # 释放物体
        MoveJointsOperation(joint_values=initial_joints, step_name='Step10: Return to initial state'),  # 回到初始状态
    ]

    for i, atom_op in enumerate(atom_ops_list):
        redis_client.set(REDIS_JAKA_CUR_STEP_KEY, atom_op.step_name)
        step_failed = False
        if isinstance(atom_op, MoveGripperToPoseOperation):
            rsp: JakaMoveGripperToPoseResponse = move_gripper_to_pose(JakaMoveGripperToPoseRequest(pose=atom_op.pose))
            if not rsp.success:
                msg = f'Step: {i+1} MoveGripperOperation failed, message: {rsp.message}'
                step_failed = True
        elif isinstance(atom_op, SetGripperTightnessOperation):
            rsp: JakaSetGripperTightnessResponse = set_gripper_tightness(JakaSetGripperTightnessRequest(tightness=atom_op.tightness))
            if not rsp.success:
                msg = f'Step: {i+1} SetGripperTightnessOperation failed, message: {rsp.message}'
                step_failed = True
        elif isinstance(atom_op, MoveJointsOperation):
            rsp = move_joints(JakaMoveJointsRequest(joint_values=atom_op.joint_values))
            if not rsp.success:
                msg = f'Step: {i+1} MoveJointsOperation failed, message: {rsp.message}'
                step_failed = True

        if step_failed:
            rospy.logerr(msg)
            redis_client.set(REDIS_JAKA_STATUS_KEY, 'idle')
            redis_client.set(REDIS_JAKA_CUR_STEP_KEY, 'idle')
            return JakaPickByMarkerIdResponse(success=False, message=msg)

    redis_client.set(REDIS_JAKA_STATUS_KEY, 'idle')
    redis_client.set(REDIS_JAKA_CUR_STEP_KEY, 'idle')
    return JakaPickByMarkerIdResponse(success=True, message=f'Successfully pick object with markerId = {markerId} onto cart')


def report_health():
    while True:
        redis_client.setex(REDIS_JAKA_HEALTH_KEY, 1, '1')
        time.sleep(0.5)


def main():
    global buffer
    global listener
    global move_gripper_to_pose
    global set_gripper_tightness
    global move_joints
    global redis_client

    # 连接本地redis
    redis_client = redis.Redis(host='localhost', port=6379, db=0, password='5iwrnpFC4BRqADn6qCFV')
    if not redis_client.ping():
        rospy.logerr('Cannot connect to redis server')
        return
    rospy.loginfo('Connected to redis server')

    # 启动一个线程，定时向redis汇报机械臂的健康状态
    report_health_thread = threading.Thread(target=report_health)
    report_health_thread.start()

    # 发布机械臂的状态为idle
    redis_client.set(REDIS_JAKA_STATUS_KEY, 'idle')
    redis_client.set(REDIS_JAKA_CUR_STEP_KEY, 'idle')

    rospy.init_node('shelf2cart_byMarkerId', anonymous=True)
    rospy.Service('shelf2cart_byMarkerId', JakaPickByMarkerId,
                  handle_shelf2cart_byMarkerId)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    move_gripper_to_pose = rospy.ServiceProxy('/zhw_jaka_wrapper/move_gripper_to_pose', JakaMoveGripperToPose)
    move_joints = rospy.ServiceProxy('/zhw_jaka_wrapper/move_joints', JakaMoveJoints)
    set_gripper_tightness = rospy.ServiceProxy('/zhw_jaka_wrapper/set_gripper_tightness', JakaSetGripperTightness)
    rospy.loginfo("Ready to shelf2cart_byMarkerId.")
    rospy.spin()


if __name__ == '__main__':
    main()
