from __future__ import annotations

import itertools
from typing import Any

import action_msgs.msg  # type: ignore[import-untyped]
import navground_msgs.action  # type: ignore[import-untyped]
import rclpy
import rclpy.action
import rclpy.action.client
import rclpy.utilities


class Node(rclpy.node.Node):

    def __init__(self) -> None:
        super().__init__("ros_demo")
        coords: list[float] = self.declare_parameter(
            "path", [1.0, 0.0, -1.0, 0.0]).value
        if len(coords) % 2 != 0:
            self.get_logger().warning(
                f"Number of coordinates {coords} should be even!")
            coords.append(0.0)
        self.path: list[tuple[float, float]] = list(
            zip(coords[::2], coords[1::2], strict=False))
        self.tol = self.declare_parameter("tolerance", 0.2).value
        self.client = rclpy.action.client.ActionClient(
            self, navground_msgs.action.GoToTarget, 'go_to')

    async def run(self) -> None:
        self.get_logger().info("Waiting controller")
        self.client.wait_for_server()
        self.get_logger().info("Controller available controller")
        for x, y in itertools.cycle(self.path):
            if not await self.move(x, y):
                continue

    async def move(self, x: float, y: float, frame_id: str = "world") -> bool:
        goal_msg = navground_msgs.action.GoToTarget.Goal()
        goal_msg.target_point.header.frame_id = frame_id
        goal_msg.target_point.point.x = float(x)
        goal_msg.target_point.point.y = float(y)
        goal_msg.position_tolerance = self.tol
        self.get_logger().info(f'Go to {x} {y}')
        goal_handle = await self.client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Go request rejected')
            return False
        self.get_logger().info('Waiting to arrive')
        response = await goal_handle.get_result_async()
        if response.status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived')
            return True
        else:
            self.get_logger().warning('Failed')
            return False


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Node()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.create_task(node.run)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.utilities.try_shutdown()
    node.destroy_node()
