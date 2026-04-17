#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Conservative upper-workspace sampling band.
# The arm sits on a table, so we keep the motion close to the folded safe pose
# and avoid large downward sweeps that could hit the tabletop.
HOME = np.array([0.00, -1.56, 1.58, 0.75, 0.00, 0.00], dtype=np.float64)
SAFE_DELTA = np.array([0.75, 0.18, 0.18, 0.16, 1.20, 0.00], dtype=np.float64)
SAFE_MIN = HOME - SAFE_DELTA
SAFE_MAX = HOME + SAFE_DELTA


@dataclass
class PoseData:
    frame_id: str
    position_xyz: list[float]
    quaternion_xyzw: list[float]


@dataclass
class Sample:
    index: int
    unix_time: float
    marker_in_camera: PoseData
    tool_in_base: PoseData
    target_joint_rad: list[float]
    actual_joint_rad: list[float]


class AutoSampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("auto_motion_aruco_sampler")
        self.args = args

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Float64MultiArray, args.command_topic, 10)

        self.latest_joint_lock = threading.Lock()
        self.latest_joint_pos: dict[str, float] = {}
        self.latest_pose_lock = threading.Lock()
        self.latest_marker_pose: Optional[PoseStamped] = None
        self.latest_marker_recv_time = 0.0
        self.samples: list[Sample] = []

        self.create_subscription(JointState, args.joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 10)

    def on_joint_state(self, msg: JointState) -> None:
        with self.latest_joint_lock:
            for n, p in zip(msg.name, msg.position):
                self.latest_joint_pos[n] = float(p)

    def on_pose(self, msg: PoseStamped) -> None:
        with self.latest_pose_lock:
            self.latest_marker_pose = msg
            self.latest_marker_recv_time = time.time()

    def get_current_joint_vec(self) -> Optional[np.ndarray]:
        with self.latest_joint_lock:
            try:
                arr = np.array([self.latest_joint_pos[n] for n in JOINT_NAMES], dtype=np.float64)
            except KeyError:
                return None
        return arr

    def get_latest_pose(self) -> Optional[PoseStamped]:
        with self.latest_pose_lock:
            return self.latest_marker_pose

    def get_latest_pose_age(self) -> float:
        with self.latest_pose_lock:
            if self.latest_marker_pose is None:
                return float("inf")
            return time.time() - self.latest_marker_recv_time

    def get_tool_in_base(self) -> Optional[PoseData]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
            )
        except TransformException:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        return PoseData(
            frame_id=self.args.base_frame,
            position_xyz=[float(t.x), float(t.y), float(t.z)],
            quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
        )

    def send_joint_cmd(self, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q.tolist()]
        self.cmd_pub.publish(msg)

    def move_to(self, target: np.ndarray) -> bool:
        now = self.get_current_joint_vec()
        if now is None:
            self.get_logger().warn("No joint_states yet.")
            return False

        diff = target - now
        max_delta = float(np.max(np.abs(diff)))
        steps = max(1, int(math.ceil(max_delta / max(1e-4, self.args.max_step_rad))))
        for i in range(1, steps + 1):
            alpha = i / steps
            q = now + alpha * diff
            self.send_joint_cmd(q)
            time.sleep(self.args.step_dt_s)

        t0 = time.time()
        while time.time() - t0 < self.args.reach_timeout_s:
            cur = self.get_current_joint_vec()
            if cur is not None and float(np.max(np.abs(cur - target))) <= self.args.reach_tol_rad:
                return True
            self.send_joint_cmd(target)
            time.sleep(0.06)
        return False

    def capture_sample(self, target: np.ndarray) -> tuple[bool, str]:
        pose = self.get_latest_pose()
        if pose is None:
            return False, "no marker pose"
        pose_age = self.get_latest_pose_age()
        if pose_age > self.args.max_pose_age_s:
            return False, f"pose stale age={pose_age:.2f}s"

        tool = self.get_tool_in_base()
        if tool is None:
            return False, "no tool tf"
        if tool.position_xyz[2] < self.args.min_tool_z:
            return False, f"tool z too low ({tool.position_xyz[2]:.3f}m < {self.args.min_tool_z:.3f}m)"

        actual = self.get_current_joint_vec()
        if actual is None:
            return False, "no joint states"

        p = pose.pose.position
        q = pose.pose.orientation
        marker = PoseData(
            frame_id=pose.header.frame_id,
            position_xyz=[float(p.x), float(p.y), float(p.z)],
            quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
        )

        s = Sample(
            index=len(self.samples) + 1,
            unix_time=time.time(),
            marker_in_camera=marker,
            tool_in_base=tool,
            target_joint_rad=[float(v) for v in target.tolist()],
            actual_joint_rad=[float(v) for v in actual.tolist()],
        )
        self.samples.append(s)
        return (
            True,
            (
                f"sample#{s.index} marker=({marker.position_xyz[0]:+.3f},{marker.position_xyz[1]:+.3f},{marker.position_xyz[2]:+.3f}) "
                f"tool=({tool.position_xyz[0]:+.3f},{tool.position_xyz[1]:+.3f},{tool.position_xyz[2]:+.3f})"
            ),
        )

    def save_json(self) -> Path:
        out = Path(self.args.out).expanduser().resolve()
        payload = {
            "meta": {
                "tool": "auto_motion_aruco_sampler.py",
                "count": len(self.samples),
                "pose_topic": self.args.pose_topic,
                "joint_state_topic": self.args.joint_state_topic,
                "command_topic": self.args.command_topic,
                "base_frame": self.args.base_frame,
                "tool_frame": self.args.tool_frame,
            },
            "samples": [asdict(s) for s in self.samples],
        }
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return out


def generate_waypoints(count: int, seed: int, range_scale: float) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    pts: list[np.ndarray] = [HOME.copy()]
    delta = SAFE_DELTA * float(range_scale)
    lo = HOME - delta
    hi = HOME + delta

    while len(pts) < count:
        q = HOME + rng.uniform(-delta, delta)
        q = np.clip(q, lo, hi)
        # Prefer moderate geometric separation while keeping the arm in the safe band.
        if all(float(np.linalg.norm(q[:5] - p[:5])) >= 0.18 for p in pts[-8:]):
            pts.append(q)
    return pts


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Auto move robot through many joint poses and collect ArUco hand-eye samples.")
    p.add_argument("--command-topic", default="/follower/forward_controller/commands")
    p.add_argument("--joint-state-topic", default="/follower/joint_states")
    p.add_argument("--pose-topic", default="/vision/aruco/pose_camera")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--tool-frame", default="moving_jaw_so101_v1_link")
    p.add_argument("--samples-target", type=int, default=30, help="有效样本目标数")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--out", default="~/ros2_ws/aruco_handeye_samples.json")
    p.add_argument("--max-step-rad", type=float, default=0.12)
    p.add_argument("--step-dt-s", type=float, default=0.12)
    p.add_argument("--reach-timeout-s", type=float, default=8.0)
    p.add_argument("--reach-tol-rad", type=float, default=0.06)
    p.add_argument("--settle-s", type=float, default=0.35)
    p.add_argument("--max-pose-age-s", type=float, default=0.30)
    p.add_argument("--pose-wait-s", type=float, default=3.0)
    p.add_argument("--min-tool-z", type=float, default=0.11)
    p.add_argument("--range-scale", type=float, default=1.0)
    p.add_argument("--waypoint-multiplier", type=int, default=5)
    p.add_argument("--go-home-first", action="store_true", default=True)
    p.add_argument("--no-go-home-first", dest="go_home_first", action="store_false")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = AutoSampler(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("== 自动多位置采样 ==")
    print(f"目标样本数: {args.samples_target}")
    print(f"输出文件: {Path(args.out).expanduser().resolve()}")
    print("")

    try:
        # Wait for basic streams.
        t0 = time.time()
        while time.time() - t0 < 12.0:
            if node.get_current_joint_vec() is not None:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"joint_state_topic='{args.joint_state_topic}' 未就绪。")

        waypoint_count = max(args.samples_target * max(1, args.waypoint_multiplier), args.samples_target + 5)
        waypoints = generate_waypoints(waypoint_count, args.seed, args.range_scale)
        if args.go_home_first:
            print("[INFO] moving to HOME ...")
            node.move_to(HOME)
            time.sleep(0.6)

        for idx, q in enumerate(waypoints, start=1):
            if len(node.samples) >= args.samples_target:
                break
            print(f"[INFO] waypoint {idx}/{len(waypoints)}")
            ok_move = node.move_to(q)
            if not ok_move:
                print("[WARN] reach timeout, continue to next waypoint.")
            time.sleep(args.settle_s)

            pose_wait_t0 = time.time()
            while node.get_latest_pose() is None and time.time() - pose_wait_t0 < args.pose_wait_s:
                time.sleep(0.05)

            ok, msg = node.capture_sample(q)
            print(("[OK] " if ok else "[WARN] ") + msg)

        out = node.save_json()
        print("")
        print(f"已保存 {len(node.samples)} 组样本到: {out}")
    except (KeyboardInterrupt, ExternalShutdownException):
        out = node.save_json()
        print(f"\n中断，已保存 {len(node.samples)} 组样本到: {out}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
