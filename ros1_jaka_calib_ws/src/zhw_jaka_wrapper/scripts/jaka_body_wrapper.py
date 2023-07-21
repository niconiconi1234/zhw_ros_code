import rospy
from zhw_jaka_msgs.srv import JakaMoveGripperToPose, JakaMoveGripperToPoseRequest, JakaMoveGripperToPoseResponse, JakaMoveJoints, JakaMoveJointsRequest, JakaMoveJointsResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander
import sys
import tf2_ros
from math import pi
import tf2_geometry_msgs


robot = None
scene = None
group_name = "jaka_zu7"
move_group = None
tf_buffer = None
tf_listener = None


def handle_move_gripper_to_pose(req: JakaMoveGripperToPoseRequest) -> JakaMoveGripperToPoseResponse:
    global tf_buffer
    global tf_listener
    x, y, z, roll, pitch, yaw = req.pose[0], req.pose[1], req.pose[2], req.pose[3], req.pose[4], req.pose[5]
    q = quaternion_from_euler(roll, pitch, yaw)
    target_pose_in_world = PoseStamped()
    target_pose_in_world.header.frame_id = 'Link_0'
    target_pose_in_world.header.stamp = rospy.Time(0)
    target_pose_in_world.pose.position.x = x
    target_pose_in_world.pose.position.y = y
    target_pose_in_world.pose.position.z = z
    target_pose_in_world.pose.orientation.x = q[0]
    target_pose_in_world.pose.orientation.y = q[1]
    target_pose_in_world.pose.orientation.z = q[2]
    target_pose_in_world.pose.orientation.w = q[3]

    # 我们想要让机械臂的爪子(gripper_frame)运动到target_post，但是如果直接调用moveit，会使机械臂的Link_6运动到target_pose
    # 因为爪子(gripper_frame)和Link_6之间存在一定距离，因此直接调用moveit得到的结果不是我们想要的
    # 因此我们需要对target_pose进行修正
    target_post_in_link_6 = tf_buffer.transform(
        target_pose_in_world,
        'Link_6',
        rospy.Duration(1.0)
    )

    # 因为Link_6 -> gripper_frame的变换是0, 0, 0.15, 0, 0, pi/4，因此我们对target_pose进行相反变换
    e = euler_from_quaternion([target_post_in_link_6.pose.orientation.x, target_post_in_link_6.pose.orientation.y,
                               target_post_in_link_6.pose.orientation.z, target_post_in_link_6.pose.orientation.w])
    q = quaternion_from_euler(e[0], e[1], e[2] - 3.1415926 / 4)

    # 修正后的target_pose在Link_6坐标系下的坐标
    target_pose_amend_in_link_6 = PoseStamped()
    target_pose_amend_in_link_6.header.frame_id = 'Link_6'
    target_pose_amend_in_link_6.header.stamp = rospy.Time(0)
    target_pose_amend_in_link_6.pose.position.x = target_post_in_link_6.pose.position.x
    target_pose_amend_in_link_6.pose.position.y = target_post_in_link_6.pose.position.y
    target_pose_amend_in_link_6.pose.position.z = target_post_in_link_6.pose.position.z-0.15
    target_pose_amend_in_link_6.pose.orientation.x = q[0]
    target_pose_amend_in_link_6.pose.orientation.y = q[1]
    target_pose_amend_in_link_6.pose.orientation.z = q[2]
    target_pose_amend_in_link_6.pose.orientation.w = q[3]

    # 修正后的target_pose在Link_0(world)坐标系下的坐标
    target_pose_amend_in_world = tf_buffer.transform(
        target_pose_amend_in_link_6, 'Link_0', rospy.Duration(1.0))
    print(target_pose_amend_in_world)

    # 准备用moveit移动机械臂
    pose_goal = Pose()
    n = 10
    pose_goal.position.x = round(target_pose_amend_in_world.pose.position.x, n)
    pose_goal.position.y = round(target_pose_amend_in_world.pose.position.y, n)
    pose_goal.position.z = round(target_pose_amend_in_world.pose.position.z, n)
    pose_goal.orientation.x = round(target_pose_amend_in_world.pose.orientation.x, n)
    pose_goal.orientation.y = round(target_pose_amend_in_world.pose.orientation.y, n)
    pose_goal.orientation.z = round(target_pose_amend_in_world.pose.orientation.z, n)
    pose_goal.orientation.w = round(target_pose_amend_in_world.pose.orientation.w, n)

    move_group.set_pose_target(pose_goal)

    maxRetry = 5
    success = False
    for _ in range(maxRetry):
        success = success or move_group.go(wait=True)
        if success:
            break

    move_group.stop()
    move_group.clear_pose_targets()
    return JakaMoveGripperToPoseResponse(success=success, message='Success' if success else 'Failed')


def handle_move_joints(req: JakaMoveJointsRequest) -> JakaMoveJointsResponse:
    joint_goal = req.joint_values

    maxRetry = 5
    success = False
    for _ in range(maxRetry):
        success = success or move_group.go(joint_goal, wait=True)
        if success:
            break

    move_group.stop()
    if success:
        return JakaMoveJointsResponse(success=success, message='Success')
    return JakaMoveJointsResponse(success=success, message='Failed')


def main():
    global robot
    global scene
    global move_group
    global group_name
    global tf_buffer
    global tf_listener

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jaka_body_wrapper', anonymous=True)

    # tf initialization
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # moveit initializationa
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    rospy.loginfo(f'jaka_body_wrapper node started')

    s1 = rospy.Service('zhw_jaka_wrapper/move_gripper_to_pose',
                       JakaMoveGripperToPose, handle_move_gripper_to_pose)
    s2 = rospy.Service('zhw_jaka_wrapper/move_joints', JakaMoveJoints, handle_move_joints)
    rospy.spin()


if __name__ == '__main__':
    main()
