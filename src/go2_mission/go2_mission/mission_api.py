#!/usr/bin/env python3
import json
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from flask import Flask, jsonify, request

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from go2_msgs.srv import (
    AbortMission,
    PauseMission,
    ResumeMission,
    SkipTask,
    StartCmdVelBridge,
    StopCmdVelBridge,
)
from go2_msgs.action import ExecuteMission

MISSIONS_DIR = os.path.expanduser("~/go2_cnimi_ws/missions")


class MissionApiNode(Node):

    def __init__(self):
        super().__init__("mission_api")
        self.abort_client   = self.create_client(AbortMission, "abort_mission")
        self.pause_client   = self.create_client(PauseMission, "pause_mission")
        self.resume_client  = self.create_client(ResumeMission, "resume_mission")
        self.skip_task_client = self.create_client(SkipTask, "skip_task")
        self.start_bridge_client = self.create_client(StartCmdVelBridge, "start_cmd_vel_bridge")
        self.stop_bridge_client  = self.create_client(StopCmdVelBridge,  "stop_cmd_vel_bridge")
        self.execute_client = ActionClient(self, ExecuteMission, "execute_mission")
        self.feedback_pub   = self.create_publisher(String, "mission_feedback", 10)

        self.declare_parameter("missions_dir", MISSIONS_DIR)
        self.missions_dir = self.get_parameter("missions_dir").get_parameter_value().string_value

        # last known progress per mission_id, used to fill in the final feedback message
        self._last_progress = {}

        # currently executing goal, used to support aborting via action cancel
        self._active_goal_handle = None
        self._active_mission_id = None

    def _wait_for_future(self, future, timeout_sec):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout_sec)

    def call_abort_mission(self, timeout_sec=10.0):
        if not self.abort_client.wait_for_service(timeout_sec=5.0):
            return None, "abort_mission service not available"

        req = AbortMission.Request()

        future = self.abort_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec):
            return None, "timeout waiting for abort_mission response"

        return future.result(), None

    def call_pause_mission(self, timeout_sec=10.0):
        if not self.pause_client.wait_for_service(timeout_sec=5.0):
            return None, "pause_mission service not available"

        req = PauseMission.Request()

        future = self.pause_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec):
            return None, "timeout waiting for pause_mission response"

        return future.result(), None

    def call_resume_mission(self, timeout_sec=10.0):
        if not self.resume_client.wait_for_service(timeout_sec=5.0):
            return None, "resume_mission service not available"

        req = ResumeMission.Request()

        future = self.resume_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec):
            return None, "timeout waiting for resume_mission response"

        return future.result(), None

    def call_skip_task(self, timeout_sec=10.0):
        if not self.skip_task_client.wait_for_service(timeout_sec=5.0):
            return None, "skip_task service not available"

        req = SkipTask.Request()

        future = self.skip_task_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec):
            return None, "timeout waiting for skip_task response"

        return future.result(), None

    def call_start_cmd_vel_bridge(self):
        if not self.start_bridge_client.wait_for_service(timeout_sec=5.0):
            return None, "start_cmd_vel_bridge service not available"

        future = self.start_bridge_client.call_async(StartCmdVelBridge.Request())
        future.add_done_callback(lambda f: self.get_logger().warn(
            f"start_cmd_vel_bridge: {f.result().message}"
        ) if not f.result().success else None)
        return {"success": True, "message": "Start requested"}, None

    def call_stop_cmd_vel_bridge(self):
        if not self.stop_bridge_client.wait_for_service(timeout_sec=5.0):
            return None, "stop_cmd_vel_bridge service not available"

        future = self.stop_bridge_client.call_async(StopCmdVelBridge.Request())
        future.add_done_callback(lambda f: self.get_logger().warn(
            f"stop_cmd_vel_bridge: {f.result().message}"
        ) if not f.result().success else None)
        return {"success": True, "message": "Stop requested"}, None

    def call_execute_mission(self, mission_id, loop_count=1, timeout_sec=10.0):
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            return None, "execute_mission action server not available"

        goal = ExecuteMission.Goal()
        goal.mission_id = mission_id
        goal.loop_count = loop_count

        send_goal_future = self.execute_client.send_goal_async(
            goal,
            feedback_callback=lambda msg: self._on_execute_feedback(mission_id, msg),
        )

        if not self._wait_for_future(send_goal_future, timeout_sec):
            return None, "timeout waiting for execute_mission goal acceptance"

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return None, "execute_mission goal rejected"

        self._active_goal_handle = goal_handle
        self._active_mission_id = mission_id

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_execute_result(mission_id, f)
        )

        return goal_handle, None

    def call_cancel_execute_mission(self, timeout_sec=10.0):
        if self._active_goal_handle is None:
            return None, "no active mission to abort"

        future = self._active_goal_handle.cancel_goal_async()
        if not self._wait_for_future(future, timeout_sec):
            return None, "timeout waiting for cancel response"

        return future.result(), None

    # ---------------------------------
    # Action feedback/result -> /mission_feedback relay
    # ---------------------------------

    def _on_execute_feedback(self, mission_id, feedback_msg):
        fb = feedback_msg.feedback

        self._last_progress[mission_id] = (fb.station_index, fb.total_stations, fb.loop_index, fb.loop_count)

        self._publish_feedback(
            mission_id=mission_id,
            state=fb.state,
            station_index=fb.station_index,
            total_stations=fb.total_stations,
            station_id=fb.station_id,
            task=fb.task,
            loop_index=fb.loop_index,
            loop_count=fb.loop_count,
        )

    def _on_execute_result(self, mission_id, future):
        status = future.result().status
        result = future.result().result

        with node_lock:
            if self._active_mission_id == mission_id:
                self._active_goal_handle = None
                self._active_mission_id = None

        station_index, total_stations, loop_index, loop_count = self._last_progress.pop(mission_id, (0, 0, 0, 1))

        if status == GoalStatus.STATUS_CANCELED:
            state = "ABORTED"
        elif status == GoalStatus.STATUS_SUCCEEDED and result.success:
            state = "COMPLETED"
        else:
            state = "FAILED"

        self._publish_feedback(
            mission_id=mission_id,
            state=state,
            station_index=station_index,
            total_stations=total_stations,
            message=result.message,
            loop_index=loop_index,
            loop_count=loop_count,
        )

    def _publish_feedback(self, mission_id, state, station_index=0, total_stations=0, station_id="", task="", message="", loop_index=0, loop_count=1):
        msg = String()
        msg.data = json.dumps({
            "mission_id": mission_id,
            "state": state,
            "station_index": station_index,
            "total_stations": total_stations,
            "station_id": station_id or None,
            "task": task or None,
            "message": message,
            "loop_index": loop_index,
            "loop_count": loop_count,
        })
        self.feedback_pub.publish(msg)


def _is_valid_mission(data):
    if not isinstance(data, dict):
        return False
    if "id" not in data or "stations" not in data:
        return False

    for station in data["stations"]:
        pose = station.get("pose", {})
        if "x" not in pose or "y" not in pose:
            return False

    return True


app = Flask(__name__)
node = None
node_lock = threading.Lock()


@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


@app.route("/execute_mission", methods=["POST", "OPTIONS"])
def execute_mission():
    if request.method == "OPTIONS":
        return ""

    data = request.get_json(silent=True) or {}
    mission_id = data.get("mission_id", "")
    loop_count = data.get("loop_count", 1)

    if not mission_id:
        return jsonify({"success": False, "message": "mission_id is required"}), 400

    with node_lock:
        goal_handle, error = node.call_execute_mission(mission_id, loop_count)

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify({"success": True, "message": "Mission execution started"})


@app.route("/missions", methods=["GET", "OPTIONS"])
def list_missions():
    if request.method == "OPTIONS":
        return ""

    missions = []

    if os.path.isdir(node.missions_dir):
        for filename in sorted(os.listdir(node.missions_dir)):
            if not filename.endswith(".json"):
                continue

            try:
                with open(os.path.join(node.missions_dir, filename)) as f:
                    data = json.load(f)
            except Exception:
                continue

            if _is_valid_mission(data):
                missions.append(data)

    return jsonify(missions)


@app.route("/save_mission", methods=["POST", "OPTIONS"])
def save_mission():
    if request.method == "OPTIONS":
        return ""

    data = request.get_json(silent=True)

    if not _is_valid_mission(data):
        return jsonify({"success": False, "message": "Invalid mission data"}), 400

    os.makedirs(node.missions_dir, exist_ok=True)
    filepath = os.path.join(node.missions_dir, f"{data['id']}.json")

    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)

    return jsonify({"success": True, "message": f"Mission saved to {filepath}"})


@app.route("/delete_mission", methods=["POST", "OPTIONS"])
def delete_mission():
    if request.method == "OPTIONS":
        return ""

    data = request.get_json(silent=True) or {}
    mission_id = data.get("mission_id", "")

    if not mission_id or ".." in mission_id or "/" in mission_id:
        return jsonify({"success": False, "message": "Invalid mission_id"}), 400

    filepath = os.path.join(node.missions_dir, f"{mission_id}.json")

    if not os.path.isfile(filepath):
        return jsonify({"success": False, "message": "Mission not found"}), 404

    os.remove(filepath)

    return jsonify({"success": True, "message": f"Mission {mission_id} deleted"})


@app.route("/abort_mission", methods=["POST", "OPTIONS"])
def abort_mission():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        if node._active_goal_handle is None:
            return jsonify({"success": False, "message": "No active mission to abort"}), 400

        cancel_result, error = node.call_cancel_execute_mission()

    if error:
        return jsonify({"success": False, "message": error}), 504

    if cancel_result.goals_canceling:
        return jsonify({"success": True, "message": "Mission abort requested"})

    return jsonify({"success": False, "message": "Goal could not be canceled"}), 504


@app.route("/pause_mission", methods=["POST", "OPTIONS"])
def pause_mission():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        result, error = node.call_pause_mission()

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify({"success": result.success, "message": result.message})


@app.route("/resume_mission", methods=["POST", "OPTIONS"])
def resume_mission():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        result, error = node.call_resume_mission()

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify({"success": result.success, "message": result.message})


@app.route("/skip_task", methods=["POST", "OPTIONS"])
def skip_task():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        result, error = node.call_skip_task()

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify({"success": result.success, "message": result.message})


@app.route("/start_cmd_vel_bridge", methods=["POST", "OPTIONS"])
def start_cmd_vel_bridge():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        result, error = node.call_start_cmd_vel_bridge()

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify(result)


@app.route("/stop_cmd_vel_bridge", methods=["POST", "OPTIONS"])
def stop_cmd_vel_bridge():
    if request.method == "OPTIONS":
        return ""

    with node_lock:
        result, error = node.call_stop_cmd_vel_bridge()

    if error:
        return jsonify({"success": False, "message": error}), 504

    return jsonify(result)


def main(args=None):
    global node

    rclpy.init(args=args)
    node = MissionApiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        app.run(host="0.0.0.0", port=5001)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
