#!/usr/bin/env python3

import json
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import (
    ActionServer,
    ActionClient,
    GoalResponse,
    CancelResponse
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from go2_msgs.action import ExecuteMission
from go2_msgs.srv import PauseMission, ResumeMission, SkipTask

from go2_mission.mission_loader import Station, MissionLoader, DEFAULT_WAIT_DURATION

MISSIONS_DIR = os.path.expanduser("~/go2_cnimi_ws/missions")


class MissionExecutorNode(Node):

    def __init__(self):

        super().__init__("mission_executor")

        # declare with default pointing to your workspace missions folder
        self.declare_parameter("missions_dir", MISSIONS_DIR)

        self._cb_group = ReentrantCallbackGroup()

        self.loader     = MissionLoader(self)
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            ExecuteMission,
            "execute_mission",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self._paused = False
        self._skip_requested = False
        self._loop_index = 0
        self._loop_count = 1

        self._task_handlers = {
            "wait":       self._task_wait,
            "take_photo": self._task_take_photo,
        }

        self.create_service(
            PauseMission, "pause_mission", self._pause_cb, callback_group=self._cb_group
        )
        self.create_service(
            ResumeMission, "resume_mission", self._resume_cb, callback_group=self._cb_group
        )
        self.create_service(
            SkipTask, "skip_task", self._skip_task_cb, callback_group=self._cb_group
        )

        self.get_logger().info("Mission executor action server ready")

    # ---------------------------------
    # Pause / resume callbacks
    # ---------------------------------

    def _pause_cb(self, request, response):

        if self._paused:
            response.success = False
            response.message = "Mission already paused"
            return response

        self._paused = True
        self.get_logger().warn("Pause requested")

        response.success = True
        response.message = "paused"
        return response

    def _resume_cb(self, request, response):

        if not self._paused:
            response.success = False
            response.message = "Mission is not paused"
            return response

        self._paused = False
        self.get_logger().info("Resume requested")

        response.success = True
        response.message = "resumed"
        return response

    def _skip_task_cb(self, request, response):

        self._skip_requested = True
        self.get_logger().warn("Skip task requested")

        response.success = True
        response.message = "skip requested"
        return response

    # ---------------------------------
    # Goal callback
    # ---------------------------------

    def _goal_callback(self, goal_request):

        self.get_logger().info(
            f"Goal received: mission_id='{goal_request.mission_id}'"
        )

        return GoalResponse.ACCEPT

    # ---------------------------------
    # Cancel callback
    # ---------------------------------

    def _cancel_callback(self, goal_handle):

        self.get_logger().warn("Cancel requested")

        return CancelResponse.ACCEPT

    # ---------------------------------
    # Execute callback
    # ---------------------------------

    def _execute_callback(self, goal_handle):

        mission_id = goal_handle.request.mission_id
        result     = ExecuteMission.Result()

        mission = self.loader.load(mission_id)

        if mission is None:
            goal_handle.abort()
            result.success = False
            result.message = f"Failed to load mission '{mission_id}'"
            return result

        if not mission.stations:
            goal_handle.abort()
            result.success = False
            result.message = "Mission has no stations"
            return result

        self._paused = False

        total = len(mission.stations)

        self.get_logger().info(
            f"Mission '{mission.name}' started — {total} stations"
        )

        loop_count = max(1, goal_handle.request.loop_count)
        self._loop_count = loop_count

        for loop_iter in range(loop_count):

            self._loop_index = loop_iter

            if loop_count > 1:
                self.get_logger().info(f"Mission loop {loop_iter + 1}/{loop_count}")

            for index, station in enumerate(mission.stations):

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result

                if self._paused:
                    if self._block_until_resumed(goal_handle, index, total, station):
                        goal_handle.canceled()
                        result.success = False
                        result.message = "canceled"
                        return result

                self._publish_feedback(goal_handle, index, total, station, "NAVIGATING")

                if not self.nav_client.wait_for_server(timeout_sec=5.0):
                    goal_handle.abort()
                    result.success = False
                    result.message = "Nav2 not available"
                    return result

                nav_goal      = NavigateToPose.Goal()
                nav_goal.pose = station.pose
                nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

                self.get_logger().info(f"Navigating to station {station.id}")

                send_goal_future = self.nav_client.send_goal_async(nav_goal)
                self._wait_for_future(send_goal_future)
                nav_goal_handle = send_goal_future.result()

                if not nav_goal_handle.accepted:
                    self.get_logger().warn(f"Goal rejected — skipping station {station.id}")
                    continue

                result_future = nav_goal_handle.get_result_async()

                while not result_future.done():
                    if goal_handle.is_cancel_requested:
                        cancel_future = nav_goal_handle.cancel_goal_async()
                        self._wait_for_future(cancel_future)
                        goal_handle.canceled()
                        result.success = False
                        result.message = "canceled"
                        return result

                    if self._paused:
                        cancel_future = nav_goal_handle.cancel_goal_async()
                        self._wait_for_future(cancel_future)

                        if self._block_until_resumed(goal_handle, index, total, station):
                            goal_handle.canceled()
                            result.success = False
                            result.message = "canceled"
                            return result

                        self._publish_feedback(goal_handle, index, total, station, "NAVIGATING")

                        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
                        send_goal_future = self.nav_client.send_goal_async(nav_goal)
                        self._wait_for_future(send_goal_future)
                        nav_goal_handle = send_goal_future.result()

                        if not nav_goal_handle.accepted:
                            self.get_logger().warn(f"Goal rejected — skipping station {station.id}")
                            result_future = None
                            break

                        result_future = nav_goal_handle.get_result_async()
                        continue

                    time.sleep(0.1)

                if result_future is None:
                    continue

                nav_status = result_future.result().status

                if nav_status != GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().error(
                        f"Navigation failed at station {station.id} (status={nav_status})"
                    )
                    self._publish_feedback(goal_handle, index, total, station, "FAILED")
                    goal_handle.abort()
                    result.success = False
                    result.message = f"Navigation failed at station {station.id} (status={nav_status})"
                    return result

                self.get_logger().info(f"Arrived at station {station.id}")
                self._publish_feedback(goal_handle, index, total, station, "ARRIVED")

                self._publish_feedback(goal_handle, index, total, station, "TASK")
                self._skip_requested = False

                handler = self._task_handlers.get(station.task)

                if handler is not None:
                    if handler(goal_handle, index, total, station) == "canceled":
                        goal_handle.canceled()
                        result.success = False
                        result.message = "canceled"
                        return result

        self.get_logger().info("Mission complete")
        self._publish_feedback(goal_handle, total, total, None, "COMPLETED")

        goal_handle.succeed()
        result.success = True
        result.message = "completed"
        return result

    # ---------------------------------
    # Task handlers
    #
    # Called once the robot has arrived at `station` and "TASK" feedback has
    # been published. Each handler must return either "done" or "canceled".
    #
    # Two helper patterns are available:
    #
    #   _sleep_with_cancel_check(duration, ...)
    #       Time-bounded task: runs for `duration` seconds then returns.
    #       Returns True early if canceled; returns False early if skipped.
    #       Pauses suspend the countdown. → use for WAIT-style tasks.
    #
    #   _wait_for_task_succeeded(...)
    #       External-trigger task: blocks indefinitely until the operator
    #       calls /skip_task ("task succeeded") or the mission is canceled.
    #       Use when the robot must wait for a sensor, arm, or human signal.
    #       → use for TAKE_PHOTO-style tasks.
    #
    # TO ADD A NEW TASK TYPE — touch these 4 files:
    #   1. mission_loader.py     — add MyTask = "my_task" to TaskType enum
    #   2. mission_executor_action.py (here) — add _task_my_task() below
    #                              and register it in self._task_handlers
    #   3. mission-ui/src/MissionPlanner.jsx — add "MY_TASK" to TASK_TYPES
    #   4. mission-ui/src/components/StationList.jsx — add MY_TASK: "Label"
    #                              to the TASK_LABELS dict at the top
    # ---------------------------------

    def _task_wait(self, goal_handle, index: int, total: int, station: Station) -> str:
        duration = getattr(station, "wait_duration", DEFAULT_WAIT_DURATION)
        if self._sleep_with_cancel_check(duration, goal_handle, index, total, station):
            return "canceled"
        return "done"

    def _task_take_photo(self, goal_handle, index: int, total: int, station: Station) -> str:
        # Blocks until the operator calls /skip_task to signal the photo was taken,
        # or the mission is canceled. Replace this body with a real camera trigger
        # (e.g. call a ROS service/action) and remove the _wait_for_task_succeeded
        # call once the hardware is available.
        if self._wait_for_task_succeeded(goal_handle, index, total, station):
            return "canceled"
        return "done"

    # ---------------------------------
    # Helpers
    # ---------------------------------

    def _wait_for_future(self, future) -> None:
        while not future.done():
            time.sleep(0.05)

    def _sleep_with_cancel_check(self, duration: float, goal_handle, index: int, total: int, station: Station) -> bool:
        """Sleeps for `duration` seconds, returning True early if a cancel is requested. Returns early (without canceling) if a skip is requested. Pauses suspend the countdown."""
        remaining = duration
        while remaining > 0:
            if goal_handle.is_cancel_requested:
                return True

            if self._skip_requested:
                self._skip_requested = False
                self.get_logger().info(f"Task skipped at station {station.id}")
                return False

            if self._paused:
                if self._block_until_resumed(goal_handle, index, total, station):
                    return True
                self._publish_feedback(goal_handle, index, total, station, "TASK")
                continue

            step = min(0.1, remaining)
            time.sleep(step)
            remaining -= step
        return False

    def _wait_for_task_succeeded(self, goal_handle, index: int, total: int, station: Station) -> bool:
        """Blocks until skip ('task succeeded') or cancel is requested. Pauses suspend the wait."""
        while True:
            if goal_handle.is_cancel_requested:
                return True

            if self._skip_requested:
                self._skip_requested = False
                self.get_logger().info(f"Task completed at station {station.id}")
                return False

            if self._paused:
                if self._block_until_resumed(goal_handle, index, total, station):
                    return True
                self._publish_feedback(goal_handle, index, total, station, "TASK")
                continue

            time.sleep(0.1)

    def _block_until_resumed(self, goal_handle, index: int, total: int, station: Station) -> bool:
        """Publishes a PAUSED feedback and blocks until resumed. Returns True if a cancel request arrives during the pause."""
        self.get_logger().warn("Mission paused")
        self._publish_feedback(goal_handle, index, total, station, "PAUSED")

        while self._paused:
            if goal_handle.is_cancel_requested:
                return True
            time.sleep(0.1)

        self.get_logger().info("Mission resumed")
        return False

    def _publish_feedback(self, goal_handle, index: int, total: int, station: Optional[Station], state: str) -> None:
        fb = ExecuteMission.Feedback()
        fb.station_index  = index
        fb.total_stations = total
        fb.station_id     = str(station.id) if station else ""
        fb.task           = station.task if station else ""
        fb.state          = state
        fb.loop_index     = self._loop_index
        fb.loop_count     = self._loop_count
        goal_handle.publish_feedback(fb)


def main(args=None):

    rclpy.init(args=args)

    node     = MissionExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
