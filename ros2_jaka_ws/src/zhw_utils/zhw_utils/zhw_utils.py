from rclpy.callback_groups import ReentrantCallbackGroup
import time
import rclpy
from rclpy.client import Client
from functools import partial
from rclpy.node import Node


def wait_for_message(node, topic_type, topic, callback_group=None):
    class _vfm(object):
        def __init__(self) -> None:
            self.msg = None

        def cb(self, msg):
            self.msg = msg

    vfm = _vfm()
    subscription = node.create_subscription(
        topic_type, topic, vfm.cb, 1, callback_group=callback_group)
    while rclpy.ok():
        if vfm.msg != None:
            res = vfm.msg
            subscription.destroy()
            return res
        rclpy.spin_once(node)
        time.sleep(0.001)
    # unsubcription>
    subscription.destroy()


def call_async_and_wait(node: Node, client: Client, req):
    class _vfm(object):
        def __init__(self) -> None:
            self.rsp = None

        def cb(self, name, rspFuture):
            self.rsp = rspFuture.result()
    vfm = _vfm()

    future = client.call_async(req)
    future.add_done_callback(partial(vfm.cb, future))

    while rclpy.ok():
        if vfm.rsp != None:
            return vfm.rsp
        rclpy.spin_once(node=node)
        time.sleep(0.001)

    client.destroy()
