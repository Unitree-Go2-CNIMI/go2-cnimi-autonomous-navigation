#!/usr/bin/env python3
import json
import os
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Fallback duration (seconds) for WAIT tasks that don't specify one
DEFAULT_WAIT_DURATION = 2.0


# =====================================================
# DATA MODEL
# =====================================================

class TaskType(Enum):
    NAVIGATE   = "navigate"
    WAIT       = "wait"
    TAKE_PHOTO = "take_photo"


@dataclass
class Station:
    id:   int
    pose: PoseStamped
    task: str = TaskType.NAVIGATE.value


@dataclass
class Mission:
    id:       str           = field(default_factory=lambda: str(uuid.uuid4()))
    name:     str           = "unnamed_mission"
    stations: List[Station] = field(default_factory=list)


# =====================================================
# MISSION LOADER
# =====================================================

class MissionLoader:

    def __init__(self, node: Node):
        self.node        = node
        self.missions_dir = node.get_parameter("missions_dir").get_parameter_value().string_value

    def load(self, mission_id: str) -> Optional[Mission]:

        if ".." in mission_id or "/" in mission_id:
            self.node.get_logger().error(f"Invalid mission_id: {mission_id}")
            return None

        path = os.path.join(self.missions_dir, f"{mission_id}.json")

        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            self.node.get_logger().error(f"Failed to load '{mission_id}': {e}")
            return None

        return self._parse(data)

    def _parse(self, data: dict) -> Mission:

        mission = Mission(id=data["id"], name=data.get("name", ""))

        for s in data.get("stations", []):

            pose                    = PoseStamped()
            pose.header.frame_id    = "map"
            pose.pose.position.x    = float(s["pose"]["x"])
            pose.pose.position.y    = float(s["pose"]["y"])
            pose.pose.position.z    = float(s["pose"]["z"])
            pose.pose.orientation.x = float(s["pose"]["qx"])
            pose.pose.orientation.y = float(s["pose"]["qy"])
            pose.pose.orientation.z = float(s["pose"]["qz"])
            pose.pose.orientation.w = float(s["pose"]["qw"])

            station = Station(id=s["id"], pose=pose, task=self._task_from_station(s))
            station.wait_duration = self._wait_duration_from_station(s)
            mission.stations.append(station)

        return mission

    def _task_from_station(self, s: dict) -> str:
        if "task" in s:
            return s["task"]

        tasks = s.get("tasks") or []
        if tasks and "type" in tasks[0]:
            return tasks[0]["type"].lower()

        return TaskType.NAVIGATE.value

    def _wait_duration_from_station(self, s: dict) -> float:
        tasks = s.get("tasks") or []
        if tasks and "duration" in tasks[0]:
            try:
                return float(tasks[0]["duration"])
            except (TypeError, ValueError):
                pass

        return DEFAULT_WAIT_DURATION
