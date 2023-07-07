from rclpy.node import Node
from zhw_msgs.srv import PickByName
from zhw_warehouse_msgs.srv import PickObjectByName
from zhw_padbot_msgs.srv import NavigateToTargetPoint
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.executors import MultiThreadedExecutor


class PickObjectByNameServer(Node):
    def __init__(self):
        super().__init__('pick_object_by_name_server')  # type: ignore

        self.cbg = ReentrantCallbackGroup()
        self.mutex_cbg = MutuallyExclusiveCallbackGroup()

        self.pick_object_by_name_server = self.create_service(PickObjectByName, 'pick_object_by_name', self.handle_pick_object_by_name, callback_group=self.mutex_cbg)  # type: ignore
        self.jaka_shelf2cart_cli = self.create_client(PickByName, '/zhw_jaka_ops/shelf2cart_by_name', callback_group=self.cbg)  # type: ignore
        self.padbot_navigate_cli = self.create_client(NavigateToTargetPoint, '/zhw_padbot_ops/navigate_to_target_point', callback_group=self.cbg)  # type: ignore
        
        self.get_logger().info('Pick object by name server started')

    async def handle_pick_object_by_name(self, req: PickObjectByName.Request, rsp: PickObjectByName.Response):
        self.padbot_navigate_cli.wait_for_service()
        self.jaka_shelf2cart_cli.wait_for_service()

        object_name = req.name

        # 导航派宝机器人到货架前
        nav_rsp: NavigateToTargetPoint.Response = await self.padbot_navigate_cli.call_async(NavigateToTargetPoint.Request(target_point='智慧湾货架旁'))  # type: ignore
        if not nav_rsp.success:
            self.get_logger().error('Failed to navigate padbot to shelf')
            rsp.success = False
            rsp.message = f'Failed to navigate padbot to shelf, reason is {nav_rsp.message}'
            return rsp

        # 派宝机器人到货架前后，机械臂抓取物品
        jaka_rsp: PickByName.Response = await self.jaka_shelf2cart_cli.call_async(PickByName.Request(name=object_name))  # type: ignore
        if not jaka_rsp.success:
            self.get_logger().error(f'Failed to pick object by name, reason is {jaka_rsp.message}')
            rsp.success = False
            rsp.message = 'Failed to pick object by name'
            return rsp

        # 导航派宝机器人门口
        nav_rsp: NavigateToTargetPoint.Response = await self.padbot_navigate_cli.call_async(NavigateToTargetPoint.Request(target_point='智慧湾门口'))  # type: ignore
        if not nav_rsp.success:
            self.get_logger().error('Failed to navigate padbot to gate')
            rsp.success = False
            rsp.message = f'Failed to navigate padbot to gate, reason is {nav_rsp.message}'
            return rsp

        rsp.success = True
        rsp.message = f'Pick up and deliver {object_name} Success'
        return rsp


def main(args=None):
    rclpy.init(args=args)
    pick_object_by_name_server = PickObjectByNameServer()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(pick_object_by_name_server)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pick_object_by_name_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
