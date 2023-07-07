import rclpy
from zhw_msgs.srv import PickByName
from zhw_msgs.srv import WrappedMove
from zhw_msgs.srv import SetGripperPosition
from zhw_utils.zhw_utils import wait_for_message
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

name2loc = {
    "YiQuan": [-0.5292653452770633, -0.33377252442580135, 0.5346162330068581, -1.7580875915853706, 0.7930845076869383, 1.8085721987854315],
    "YuanQiSenLin": [-0.49608131982421927, -0.12935652197265138, 0.5346162109375003, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958],
    "BaiSuiShan": [-0.47668931982421753, 0.05886747802734894, 0.5346162109375008, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958],
    "HengDa": [-0.4540333198242171, 0.24171547802735058, 0.5346162109375006, -1.7580876004654573, 0.7930844840327533, 1.8085721676276958]
}
release_loc = [-0.10985980987549104, -0.6901132543945238, -0.0675829401321393, -
               1.5630430318776487, -0.6873484399663788, 0.6007501504361223]
initial_loc = [-0.1649426259969279, -0.20539363533719734, 0.7154192434613571, -
               1.7651893869295003, 0.3854301695230128, 1.5270821565668262]
before_release_loc = [-0.10985980987548907, -0.562273286621104, -
                      0.014049059867859263, -1.5630430318776487, -0.6873484399663788, 0.6007501504361223]


class Shelf2CartByName(Node):
    def __init__(self):
        super().__init__('shelf2cart_by_name')  # type: ignore
        self.cbg = ReentrantCallbackGroup()
        self.srv = self.create_service(PickByName, 'shelf2cart_by_name',
                                       self.handle_shelf2cart_by_name, callback_group=MutuallyExclusiveCallbackGroup())  # type: ignore
        self.move_line_wrapper_cli = self.create_client(
            WrappedMove, '/zhw_jaka_wrapper/move_line_wrapper', callback_group=self.cbg)
        self.set_gripper_pose_cli = self.create_client(
            SetGripperPosition, '/zhw_jaka_wrapper/set_gripper_pose', callback_group=self.cbg)

    async def handle_shelf2cart_by_name(self, request: PickByName.Request, response: PickByName.Response):
        obj_name = request.name
        if obj_name not in name2loc:
            response.success = False
            response.message = "unknown object name"

        self.move_line_wrapper_cli.wait_for_service()
        self.set_gripper_pose_cli.wait_for_service()

        loc_pick = name2loc[obj_name]
        loc_before_pick = [loc_pick[0]+0.1, loc_pick[1], loc_pick[2],
                           loc_pick[3], loc_pick[4], loc_pick[5]]  # 抓取前的位置在物体前面0.1m处
        loc_lift = [loc_pick[0], loc_pick[1], loc_pick[2]+0.05, loc_pick[3], loc_pick[4], loc_pick[5]]  # 将物体抬起一点

        # 移动到初始位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=initial_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '1.initlal move to initial_loc failed'
            return response

        # 释放夹爪
        gripper_rsp: SetGripperPosition.Response = await self.set_gripper_pose_cli.call_async(
            SetGripperPosition.Request(target_position=100))  # type: ignore
        if not gripper_rsp.success:
            response.success = False
            response.message = '2.initial release gripper failed'
            return response

        # 移动到目标位置的前方0.1m处
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=loc_before_pick))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '3.move to loc_before_pick failed'
            return response

        # 移动到目标位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=loc_pick))        # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '4.move to loc_pick failed'
            return response

        # 夹紧架爪
        gripper_rsp = await self.set_gripper_pose_cli.call_async(
            SetGripperPosition.Request(target_position=20))  # type: ignore
        if not gripper_rsp.success:
            response.success = False
            response.message = '5.grip object failed'
            return response

        # 将物体抬起一点
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=loc_lift))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '6.move to loc_lift failed'
            return response

        # 移动到初始位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=initial_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '7.move to initial_loc failed'
            return response

        # 移动到释放前位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=before_release_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '8.move to before_release_loc failed'
            return response

        # 移动到释放位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=release_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '9.move to release_loc failed'
            return response

        # 释放夹爪
        gripper_rsp = await self.set_gripper_pose_cli.call_async(
            SetGripperPosition.Request(target_position=100))  # type: ignore
        if not gripper_rsp.success:
            response.success = False
            response.message = '10.release gripper failed'
            return response

        # 移动到释放前位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=before_release_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '11.move to before_release_loc failed'
            return response

        # 移动回初始位置
        move_rsp: WrappedMove.Response = await self.move_line_wrapper_cli.call_async(
            WrappedMove.Request(pose=initial_loc))  # type: ignore
        if not move_rsp.success:
            response.success = False
            response.message = '12.final move to initial_loc failed'
            return response

        # 成功
        response.success = True
        response.message = "Successfully picked object %s" % obj_name
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Shelf2CartByName()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__name__':
    main()
