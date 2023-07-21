import rospy
from zhw_jaka_msgs.srv import JakaMoveGripperToPose, JakaMoveGripperToPoseRequest, JakaMoveGripperToPoseResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import tf2_ros
import sys
import rospy
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion


def main():
    rospy.init_node('handeye_calib_test')
    rospy.loginfo('Handeye calibration test node started')

    pose_in_camera: PoseStamped = rospy.wait_for_message(
        '/aruco_single/pose', PoseStamped)
    pose_in_camera.header.stamp = rospy.Time(0)

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    pose_in_base = buffer.transform(
        pose_in_camera, 'Link_0', rospy.Duration(1.0))
    print(pose_in_base)

    e = euler_from_quaternion([-0.499, 0.511, 0.517, -0.472])
    t = [pose_in_base.pose.position.x,
         pose_in_base.pose.position.y,
         pose_in_base.pose.position.z]

    move_gripper_to_pose = rospy.ServiceProxy(
        "zhw_jaka_wrapper/handle_move_gripper_to_pose",
        JakaMoveGripperToPose
    )
    req = JakaMoveGripperToPoseRequest()
    req.pose = [t[0], t[1], t[2], e[0], e[1], e[2]]
    rsp = move_gripper_to_pose(req)
    print(rsp)


if __name__ == '__main__':
    main()
