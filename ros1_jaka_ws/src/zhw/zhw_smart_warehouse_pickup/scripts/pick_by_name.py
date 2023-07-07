import rospy
from zhw_msgs.srv import PickByName, PickByNameResponse, PickByNameRequest
from zhw_msgs.srv import WrappedMove, WrappedMoveRequest, WrappedMoveResponse
from zhw_msgs.srv import SetGripperPosition, SetGripperPositionRequest, SetGripperPositionResponse

name2loc = {
    "YiQuan": [-0.5292653452770633, -0.33377252442580135, 0.5346162330068581, -1.7580875915853706, 0.7930845076869383, 1.8085721987854315],
    "YuanQiSenLin": [-0.49608131982421927, -0.12935652197265138, 0.5346162109375003, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958],
    "BaiSuiShan": [-0.47668931982421753, 0.05886747802734894, 0.5346162109375008, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958],
    "HengDa": [-0.4540333198242171, 0.24171547802735058, 0.5346162109375006, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958]
}
release_loc = [-0.10985980987549104, -0.6901132543945238, -0.0675829401321393, -1.5630430318776487, -0.6873484399663788, 0.6007501504361223]
initial_loc = [-0.1649426259969279, -0.20539363533719734, 0.7154192434613571, -1.7651893869295003, 0.3854301695230128, 1.5270821565668262]
before_release_loc = [-0.10985980987548907, -0.562273286621104, -0.014049059867859263, -1.5630430318776487, -0.6873484399663788, 0.6007501504361223]


def handle_pick_by_name(req: PickByNameRequest) -> PickByNameResponse:
    obj_name = req.name
    if obj_name not in name2loc:
        return PickByNameResponse(success=False, message="unknown object name")

    rospy.wait_for_service('/jaka_wrapper/move_line_wrapper')
    rospy.wait_for_service('/jaka_wrapper/set_gripper_pose')

    move_line_wrapper = rospy.ServiceProxy('/jaka_wrapper/move_line_wrapper', WrappedMove)
    set_gripper_pose = rospy.ServiceProxy('/jaka_wrapper/set_gripper_pose', SetGripperPosition)

    loc_pick = name2loc[obj_name]
    loc_before_pock = [loc_pick[0]+0.1, loc_pick[1], loc_pick[2], loc_pick[3], loc_pick[4], loc_pick[5]]  # 抓取前的位置在物体前面0.1m处
    loc_lift = [loc_pick[0], loc_pick[1], loc_pick[2]+0.05, loc_pick[3], loc_pick[4], loc_pick[5]]  # 将物体抬起一点
    # 移动到初始位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=initial_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="1.initlal move to initial_loc failed")
    # 释放夹爪
    gripper_rsp: SetGripperPositionResponse = set_gripper_pose(SetGripperPositionRequest(target_position=100))
    if not gripper_rsp.success:
        return PickByNameResponse(success=False, message="2.initial release gripper failed")
    # 移动到目标位置的前方0.1m处
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=loc_before_pock))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="3.move to loc_before_pick failed")
    # 移动到目标位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=loc_pick))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="4.move to loc_pick failed")
    # 夹紧架爪
    gripper_rsp: SetGripperPositionResponse = set_gripper_pose(SetGripperPositionRequest(target_position=20))
    if not gripper_rsp.success:
        return PickByNameResponse(success=False, message="5.grip object failed")
    # 将物体抬起一点
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=loc_lift))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="6.move to loc_lift failed")
    # 移动到初始位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=initial_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="7.move to initial_loc failed")
    # 移动到释放前位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=before_release_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="8.move to before_release_loc failed")
    # 移动到释放位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=release_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="9.move to release_loc failed")
    # 释放夹爪
    gripper_rsp: SetGripperPositionResponse = set_gripper_pose(SetGripperPositionRequest(target_position=100))
    if not gripper_rsp.success:
        return PickByNameResponse(success=False, message="10.final release gripper failed")
    # 移动到释放前位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=before_release_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="11.move to before_release_loc failed")
    # 移动回初始位置
    move_rsp: WrappedMoveResponse = move_line_wrapper(WrappedMoveRequest(pose=initial_loc))
    if not move_rsp.success:
        return PickByNameResponse(success=False, message="12.final move to initial_loc failed")
    # 成功
    return PickByNameResponse(success=True, message="Successfully picked object %s" % obj_name)


def main():
    rospy.init_node('pick_by_name_server')
    rospy.loginfo('pick_by_name_server started')
    s = rospy.Service('pick_by_name', PickByName, handle_pick_by_name)
    rospy.spin()


if __name__ == '__main__':
    main()
