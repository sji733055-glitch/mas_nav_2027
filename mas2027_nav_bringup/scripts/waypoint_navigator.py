#!/usr/bin/env python3
# Copyright 2026 mas2027
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Drive a fixed waypoint route by sending NavigateToPose goals one at a time.

就是把 RViz 里手点的 "2D Nav Goal" 自动化：一个一个发 `navigate_to_pose`，
等上一个出结果再发下一个，跑完可以从头再来（巡逻）。

为什么不用已经在跑的 `nav2_waypoint_follower`（FollowWaypoints）：
Humble 版的 FollowWaypoints 把整条航线一次交出去，没有单点超时——某个航点因为
ROG-Map 还没看到那片区域而长时间规划不出来时，整条航线就卡在那儿。逐点发
`navigate_to_pose` 可以给每个点单独设超时和重试次数，卡住就跳到下一个。
两者用的是同一套 BT 与规划/控制链路（`navigate_to_pose_w_replanning_and_recovery.xml`
→ MincoPlanner + MincoMpcController），所以行为上没有额外风险。

坐标系：航点写在 `frame_id`（默认 `odom`）下。本仓库全程没有 `map` 帧，
只有 small_point_lio 的 `odom`。**odom 会漂**，所以一条固定航线跑久了会整体偏移；
真要长时间巡逻得先有重定位（PRIORMAP 模式 + 先验图），这里只做到"能自动跑"。

默认不启动：`use_waypoint_navigator` 默认 False。比赛里上电自动开跑很危险，
要用就显式打开。
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# 状态机取值。用字符串而不是枚举，日志里直接可读。
_WAIT_SERVER = "WAIT_SERVER"
_DELAY = "DELAY"
_SEND = "SEND"
_AWAIT_ACCEPT = "AWAIT_ACCEPT"
_AWAIT_RESULT = "AWAIT_RESULT"
_PAUSE = "PAUSE"
_STOPPED = "STOPPED"

_TICK_PERIOD_SEC = 0.2


class WaypointNavigator(Node):
    """按顺序把航点作为 NavigateToPose 目标发出去, 跑完可循环."""

    def __init__(self):
        super().__init__("waypoint_navigator")

        # 航点用一维数组存 [x, y, yaw_deg] 三元组——ROS 2 参数不支持嵌套数组。
        self.declare_parameter("waypoints", [])
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("loop", True)
        # 等 Nav2 全部 active 之后再多等一会儿；ROG-Map 需要几帧点云才有可用地图，
        # 太早发目标会因为四周全是 UNKNOWN 而直接规划失败。
        self.declare_parameter("start_delay_sec", 5.0)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("pause_at_waypoint_sec", 0.2)
        self.declare_parameter("attempts_per_waypoint", 2)
        # False：某个航点失败就跳过，继续跑后面的（与 waypoint_follower 的
        # stop_on_failure: false 保持一致）。True：整条航线停下。
        self.declare_parameter("stop_on_failure", False)

        self._frame_id = self.get_parameter("frame_id").value
        self._loop = bool(self.get_parameter("loop").value)
        self._start_delay = float(self.get_parameter("start_delay_sec").value)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self._pause = float(self.get_parameter("pause_at_waypoint_sec").value)
        self._attempts = max(1, int(self.get_parameter("attempts_per_waypoint").value))
        self._stop_on_failure = bool(self.get_parameter("stop_on_failure").value)
        self._waypoints = self._parse_waypoints(self.get_parameter("waypoints").value)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._state = _WAIT_SERVER
        self._index = 0
        self._attempt = 0
        self._lap = 0
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._mark = self.get_clock().now()
        self._deadline = None

        if not self._waypoints:
            self.get_logger().error(
                "waypoints 参数为空，节点空转。格式是 [x, y, yaw_deg] 三元组展平的一维数组，"
                "例如 [1.0, 0.0, 0.0,  1.0, 1.0, 90.0]。"
            )
            self._state = _STOPPED
        else:
            self.get_logger().info(
                f"航点自动行驶：{len(self._waypoints)} 个航点，frame_id={self._frame_id}，"
                f"loop={self._loop}，单点超时 {self._goal_timeout:.1f}s，"
                f"每点最多 {self._attempts} 次尝试。等 navigate_to_pose 服务端..."
            )

        self._timer = self.create_timer(_TICK_PERIOD_SEC, self._tick)

    def _parse_waypoints(self, raw):
        """Turn the flat [x, y, yaw_deg, ...] array into a list of triples."""
        if raw is None:
            return []
        values = [float(v) for v in raw]
        if len(values) % 3 != 0:
            self.get_logger().error(
                f"waypoints 长度 {len(values)} 不是 3 的倍数，按 [x, y, yaw_deg] 三元组解析失败。"
            )
            return []
        return [tuple(values[i : i + 3]) for i in range(0, len(values), 3)]

    def _elapsed(self):
        return (self.get_clock().now() - self._mark).nanoseconds * 1e-9

    def _goto(self, state):
        self._state = state
        self._mark = self.get_clock().now()

    def _make_goal(self, waypoint):
        x, y, yaw_deg = waypoint
        yaw = math.radians(yaw_deg)
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        goal.pose = pose
        return goal

    def _tick(self):
        """State machine driven by a timer, so nothing here may block."""
        if self._state == _STOPPED:
            return

        if self._state == _WAIT_SERVER:
            if self._client.server_is_ready():
                self.get_logger().info(
                    f"navigate_to_pose 就绪，{self._start_delay:.1f}s 后开始跑航线。"
                )
                self._goto(_DELAY)
            else:
                self.get_logger().info("等 navigate_to_pose 服务端...", throttle_duration_sec=5.0)
            return

        if self._state == _DELAY:
            if self._elapsed() >= self._start_delay:
                self._goto(_SEND)
            return

        if self._state == _SEND:
            # bt_navigator 掉了就退回等待，不要把目标发进空气里。
            if not self._client.server_is_ready():
                self.get_logger().warn("navigate_to_pose 服务端消失，退回等待。")
                self._goto(_WAIT_SERVER)
                return
            waypoint = self._waypoints[self._index]
            self._attempt += 1
            self.get_logger().info(
                f"[第 {self._lap + 1} 圈] 航点 {self._index + 1}/{len(self._waypoints)} "
                f"-> x={waypoint[0]:.2f} y={waypoint[1]:.2f} yaw={waypoint[2]:.1f}° "
                f"(第 {self._attempt}/{self._attempts} 次尝试)"
            )
            self._send_future = self._client.send_goal_async(self._make_goal(waypoint))
            self._goto(_AWAIT_ACCEPT)
            return

        if self._state == _AWAIT_ACCEPT:
            self._tick_await_accept()
            return

        if self._state == _AWAIT_RESULT:
            self._tick_await_result()
            return

        if self._state == _PAUSE and self._elapsed() >= self._pause:
            self._advance()

    def _tick_await_accept(self):
        if not self._send_future.done():
            return
        goal_handle = self._send_future.result()
        self._send_future = None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"航点 {self._index + 1} 的目标被 bt_navigator 拒绝。")
            self._on_failure()
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self._goal_timeout
        )
        self._goto(_AWAIT_RESULT)

    def _tick_await_result(self):
        if self._result_future.done():
            status = self._result_future.result().status
            self._result_future = None
            self._goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"航点 {self._index + 1} 到达。")
                self._attempt = 0
                self._goto(_PAUSE)
            else:
                self.get_logger().warn(f"航点 {self._index + 1} 失败，GoalStatus={status}。")
                self._on_failure()
            return

        if self.get_clock().now() >= self._deadline:
            self.get_logger().warn(
                f"航点 {self._index + 1} 超过 {self._goal_timeout:.1f}s 未完成，撤销该目标。"
            )
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._result_future = None
            self._goal_handle = None
            self._on_failure()

    def _on_failure(self):
        if self._attempt < self._attempts:
            self._goto(_SEND)
            return
        self._attempt = 0
        if self._stop_on_failure:
            self.get_logger().error(
                f"航点 {self._index + 1} 用尽 {self._attempts} 次尝试，"
                "stop_on_failure=true，航线停止。"
            )
            self._goto(_STOPPED)
            return
        self.get_logger().warn(f"跳过航点 {self._index + 1}，继续下一个。")
        self._goto(_PAUSE)

    def _advance(self):
        self._index += 1
        if self._index < len(self._waypoints):
            self._goto(_SEND)
            return
        self._index = 0
        self._lap += 1
        if self._loop:
            self.get_logger().info(f"第 {self._lap} 圈跑完，从头开始。")
            self._goto(_SEND)
        else:
            self.get_logger().info("航线跑完，loop=false，停止。")
            self._goto(_STOPPED)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停之前把在飞的目标撤掉，否则机器人会继续往最后一个航点跑。
        if node._goal_handle is not None:
            node._goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
