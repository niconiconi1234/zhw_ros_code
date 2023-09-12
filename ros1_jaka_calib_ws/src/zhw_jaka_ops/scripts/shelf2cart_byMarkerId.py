import rospy
from zhw_jaka_msgs.srv import JakaPickByMarkerId, JakaPickByMarkerIdRequest, JakaPickByMarkerIdResponse
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from zhw_jaka_msgs.srv import JakaMoveGripperToPose, JakaMoveGripperToPoseRequest, JakaMoveGripperToPoseResponse, JakaSetGripperTightness, JakaSetGripperTightnessRequest, JakaSetGripperTightnessResponse, JakaMoveJoints, JakaMoveJointsRequest, JakaMoveJointsResponse
import tf2_ros


class MoveGripperToPoseOperation():
    def __init__(self, pose):
        self.pose = pose


class MoveJointsOperation():
    def __init__(self, joint_values):
        self.joint_values = joint_values


class SetGripperTightnessOperation():
    TIGHT = 1.0
    LOOSE = 0.0

    def __init__(self, tightness):
        self.tightness = tightness


buffer = None
listener = None
move_gripper_to_pose = None
set_gripper_tightness = None
move_joints = None

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

    # 动作列表
    atom_ops_list = [
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.LOOSE),  # 首先释放夹爪
        MoveJointsOperation(joint_values=initial_joints),  # 移动机械臂关节到初始状态
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x+0.1, object_pose.pose.position.y,
                                         object_pose.pose.position.z, -0.833, 1.541, 2.266]),  # 移动爪子到物体前方0.1米，准备抓取
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x, object_pose.pose.position.y,
                                         object_pose.pose.position.z, -0.833, 1.541, 2.266]),  # 移动爪子到物体处，准备抓取
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.TIGHT),  # 抓取物体
        MoveGripperToPoseOperation(pose=[object_pose.pose.position.x, object_pose.pose.position.y,
                                         object_pose.pose.position.z+0.05, -0.833, 1.541, 2.266]),  # 将物体抬起0.05米
        MoveJointsOperation(joint_values=initial_joints),  # 回到初始状态
        MoveJointsOperation(joint_values=release_joints),  # 将机械臂移动到释放物体的状态
        SetGripperTightnessOperation(tightness=SetGripperTightnessOperation.LOOSE),  # 释放物体
        MoveJointsOperation(joint_values=initial_joints),  # 回到初始状态
    ]

    for i, atom_op in enumerate(atom_ops_list):
        if isinstance(atom_op, MoveGripperToPoseOperation):
            rsp: JakaMoveGripperToPoseResponse = move_gripper_to_pose(JakaMoveGripperToPoseRequest(pose=atom_op.pose))
            if not rsp.success:
                msg = f'Step: {i+1} MoveGripperOperation failed, message: {rsp.message}'
                rospy.logerr(msg)
                return JakaPickByMarkerIdResponse(success=False, message=msg)
        elif isinstance(atom_op, SetGripperTightnessOperation):
            rsp: JakaSetGripperTightnessResponse = set_gripper_tightness(JakaSetGripperTightnessRequest(tightness=atom_op.tightness))
            if not rsp.success:
                msg = f'Step: {i+1} SetGripperTightnessOperation failed, message: {rsp.message}'
                rospy.logerr(msg)
                return JakaPickByMarkerIdResponse(success=False, message=msg)
        elif isinstance(atom_op, MoveJointsOperation):
            rsp = move_joints(JakaMoveJointsRequest(joint_values=atom_op.joint_values))
            if not rsp.success:
                msg = f'Step: {i+1} MoveJointsOperation failed, message: {rsp.message}'
                rospy.logerr(msg)
                return JakaPickByMarkerIdResponse(success=False, message=msg)

    return JakaPickByMarkerIdResponse(success=True, message=f'Successfully pick object with markerId = {markerId} onto cart')


def main():
    global buffer
    global listener
    global move_gripper_to_pose
    global set_gripper_tightness
    global move_joints
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
