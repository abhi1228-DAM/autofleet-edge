#!/usr/bin/env python3
"""AutoFleet learning exercise: drive the BLUE Gazebo vehicle to an odometry goal.

SIMULATION ONLY. This proportional controller has no map, obstacle avoidance,
path planner, fleet coordination, or natural-language interface. It is not Nav2.
Use it in the open area of ros_gz_sim_demos/diff_drive.launch.py. Stop any manual
cmd_vel publisher first. Coordinates are in the incoming odometry frame.

Run inside your ROS 2 Humble container, with Gazebo running and unpaused:
    source /opt/ros/humble/setup.bash
    python3 autofleet_go_to.py --x 0.0 --y 1.0

Run the offline tests with any Python 3.8+ installation (ROS is not required):
    python3 autofleet_go_to.py --self-test

Control+C requests cancellation and repeated zero-velocity messages before exit.
Stopping is best effort: terminating the container/process forcibly cannot run
cleanup. This script is intended only for the named simulated blue vehicle.
"""

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass

ODOM_TOPIC = "/model/vehicle_blue/odometry"
COMMAND_TOPIC = "/model/vehicle_blue/cmd_vel"
TOLERANCE = 0.10       # metres
MAX_FORWARD = 0.15    # metres/second
MAX_TURN = 0.60       # radians/second
STALE_AFTER = 2.0     # wall-clock seconds without an advancing odometry timestamp
STARTUP_TIMEOUT = 20.0


def wrap_angle(angle):
    """Return the shortest signed angle, between -pi and pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    """Convert the odometry quaternion to heading (radians)."""
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError("Odometry orientation contains a non-finite value.")
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError("Odometry orientation is invalid.")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


@dataclass(frozen=True)
class Command:
    status: str
    forward: float = 0.0
    turn: float = 0.0
    distance: float = 0.0


def decide(pose, goal, *, feedback_age, elapsed, timeout, bridge_ready=True):
    """Choose one bounded command. All terminal states command zero velocity."""
    if not all(math.isfinite(v) for v in (*pose, *goal, feedback_age, elapsed, timeout)):
        return Command("INVALID_DATA")
    if feedback_age < 0 or elapsed < 0 or timeout <= 0:
        return Command("INVALID_DATA")
    if not bridge_ready:
        return Command("BRIDGE_LOST")
    if feedback_age > STALE_AFTER:
        return Command("STALE_ODOMETRY")
    if elapsed >= timeout:
        return Command("TIMEOUT")

    x, y, heading = pose
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)
    if distance <= TOLERANCE:
        return Command("ARRIVED", distance=distance)

    # Face the destination before moving forward. Slow down near the goal.
    error = wrap_angle(math.atan2(dy, dx) - heading)
    turn = max(-MAX_TURN, min(MAX_TURN, 1.5 * error))
    forward = 0.0
    if abs(error) < 0.35:
        forward = min(MAX_FORWARD, 0.6 * distance) * math.cos(error)
    return Command("MOVING", forward, turn, distance)


def self_test():
    """Check motion, failure stops, and convergence with a simple kinematic model."""
    import unittest

    class ControllerTests(unittest.TestCase):
        def command(self, pose=(0.0, 0.0, 0.0), goal=(1.0, 0.0), **kw):
            args = dict(feedback_age=0.0, elapsed=0.0, timeout=90.0)
            args.update(kw)
            return decide(pose, goal, **args)

        def test_goal_ahead_drives_forward(self):
            c = self.command()
            self.assertGreater(c.forward, 0.0)
            self.assertEqual(c.turn, 0.0)

        def test_goal_behind_requires_turning_first(self):
            c = self.command(goal=(-1.0, 0.0))
            self.assertEqual(c.forward, 0.0)
            self.assertGreater(abs(c.turn), 0.0)

        def test_shortest_turn_across_angle_boundary(self):
            c = self.command(pose=(0.0, 0.0, math.pi - 0.01), goal=(-1.0, -0.01))
            self.assertGreater(c.turn, 0.0)
            self.assertLess(c.turn, 0.1)

        def test_arrival_commands_zero(self):
            c = self.command(pose=(0.95, 0.0, 1.0))
            self.assertEqual(c.status, "ARRIVED")
            self.assertEqual((c.forward, c.turn), (0.0, 0.0))

        def test_terminal_failures_command_zero(self):
            cases = [
                ({"feedback_age": 2.1}, "STALE_ODOMETRY"),
                ({"elapsed": 90.0}, "TIMEOUT"),
                ({"bridge_ready": False}, "BRIDGE_LOST"),
                ({"pose": (float("nan"), 0.0, 0.0)}, "INVALID_DATA"),
            ]
            for kwargs, status in cases:
                with self.subTest(status=status):
                    c = self.command(**kwargs)
                    self.assertEqual(c.status, status)
                    self.assertEqual((c.forward, c.turn), (0.0, 0.0))

        def test_stale_feedback_cannot_report_arrival(self):
            c = self.command(pose=(1.0, 0.0, 0.0), feedback_age=3.0)
            self.assertEqual(c.status, "STALE_ODOMETRY")

        def test_velocity_limits_and_slow_approach(self):
            for goal in [(100.0, 0.0), (0.0, 100.0), (-100.0, -100.0)]:
                c = self.command(goal=goal)
                self.assertLessEqual(c.forward, MAX_FORWARD)
                self.assertLessEqual(abs(c.turn), MAX_TURN)
            self.assertLess(self.command(goal=(0.12, 0.0)).forward, MAX_FORWARD)

        def test_quaternion_normalization_and_bad_input(self):
            self.assertAlmostEqual(quaternion_yaw(0.0, 0.0, 2.0, 2.0), math.pi/2)
            for q in [(0, 0, 0, 0), (0, 0, float("nan"), 1)]:
                with self.assertRaises(ValueError):
                    quaternion_yaw(*q)

        def test_reaches_goal_from_several_start_poses(self):
            # This is a math-model test, not a ROS/Gazebo integration test.
            for heading in [-math.pi, -1.5, 0.0, 1.5, math.pi]:
                with self.subTest(heading=heading):
                    pose = [-0.3516217489663933, 1.939790962870654, heading]
                    for tick in range(900):
                        c = self.command(pose=pose, goal=(0.0, 1.0), elapsed=tick*0.1)
                        if c.status != "MOVING":
                            break
                        pose[0] += c.forward * math.cos(pose[2]) * 0.1
                        pose[1] += c.forward * math.sin(pose[2]) * 0.1
                        pose[2] = wrap_angle(pose[2] + c.turn * 0.1)
                    self.assertEqual(c.status, "ARRIVED")
                    self.assertLessEqual(c.distance, TOLERANCE)

    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ControllerTests)
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


def run_ros(goal, timeout):
    # Late imports let --help and --self-test work without a ROS installation.
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
    except ImportError:
        print("Run this inside the ROS container after: source /opt/ros/humble/setup.bash", file=sys.stderr)
        return 2

    class Navigator(Node):
        def __init__(self):
            super().__init__("autofleet_go_to")
            self.publisher = self.create_publisher(Twist, COMMAND_TOPIC, 10)
            self.subscription = self.create_subscription(
                Odometry, ODOM_TOPIC, self.receive_odometry, qos_profile_sensor_data)
            self.pose = None
            self.frame = None
            self.last_stamp = None
            self.received_at = None
            self.fault = None

        def receive_odometry(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            try:
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise ValueError("Odometry position is invalid.")
                pose = (p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w))
                frame = msg.header.frame_id
                if not frame or (self.frame is not None and frame != self.frame):
                    raise ValueError("Odometry frame is empty or changed; restart the exercise.")
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                if self.last_stamp is not None:
                    if stamp < self.last_stamp:
                        raise ValueError("Simulation time reset; rerun the controller.")
                    if stamp == self.last_stamp:
                        return  # A frozen timestamp does not count as fresh feedback.
                self.pose, self.frame, self.last_stamp = pose, frame, stamp
                self.received_at = time.monotonic()
            except ValueError as exc:
                self.fault = str(exc)

        def send(self, forward=0.0, turn=0.0):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(forward), float(turn)
            self.publisher.publish(msg)

    cancelled = False

    def cancel(_signum, _frame):
        nonlocal cancelled
        cancelled = True

    # Keep the ROS context alive during Control+C cleanup so we can send a stop.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    previous = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    outcome = 2
    try:
        node = Navigator()
        print("SIMULATION ONLY: stop all other blue-vehicle command publishers.", flush=True)
        print("WAITING for fresh odometry and the velocity bridge; keep Gazebo unpaused.", flush=True)
        waiting_since = time.monotonic()
        started_at = None
        next_tick = next_report = 0.0

        while rclpy.ok() and not cancelled:
            rclpy.spin_once(node, timeout_sec=0.05)
            if cancelled:
                break
            now = time.monotonic()
            if now < next_tick:
                continue
            next_tick = now + 0.1
            if node.fault:
                print("STOPPED: " + node.fault, flush=True)
                break

            ready = node.publisher.get_subscription_count() > 0
            if started_at is None:
                node.send()  # Hold still while discovery completes.
                fresh = node.received_at is not None and now - node.received_at <= STALE_AFTER
                if fresh and ready:
                    started_at = now
                    print("TARGET x={:.2f}, y={:.2f} m in frame '{}'".format(*goal, node.frame), flush=True)
                elif now - waiting_since > STARTUP_TIMEOUT:
                    print("STOPPED: no fresh odometry or velocity subscriber. Check Gazebo and its bridge.", flush=True)
                    break
                else:
                    continue

            c = decide(node.pose, goal, feedback_age=now-node.received_at,
                       elapsed=now-started_at, timeout=timeout, bridge_ready=ready)
            node.send(c.forward, c.turn)
            if c.status == "ARRIVED":
                print("ARRIVED: x={:.3f}, y={:.3f}; target error={:.3f} m. Stopping.".format(
                    node.pose[0], node.pose[1], c.distance), flush=True)
                outcome = 0
                break
            if c.status != "MOVING":
                print("STOPPED: {}. Check the simulation before rerunning.".format(c.status), flush=True)
                break
            if now >= next_report:
                print("MOVING: remaining={:.2f} m, forward={:.2f} m/s, turn={:.2f} rad/s".format(
                    c.distance, c.forward, c.turn), flush=True)
                next_report = now + 1.0

        if cancelled:
            print("CANCELLED: sending stop commands.", flush=True)
            outcome = 130
    except Exception as exc:
        print("STOPPED: {}: {}".format(type(exc).__name__, exc), file=sys.stderr, flush=True)
        outcome = 2
    finally:
        if node is not None:
            # Repeat the stop to give discovery/transport time to deliver it.
            for _ in range(10):
                if not rclpy.ok():
                    break
                try:
                    node.send()
                    time.sleep(0.05)
                except Exception as exc:
                    print("Could not send stop: {}".format(exc), file=sys.stderr)
                    break
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return outcome


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--x", type=float, help="Target x in the incoming odometry frame, metres")
    parser.add_argument("--y", type=float, help="Target y in the incoming odometry frame, metres")
    parser.add_argument("--timeout", type=float, default=90.0, help="Maximum motion time in wall seconds (default: 90)")
    parser.add_argument("--self-test", action="store_true", help="Run offline controller tests; no ROS or movement")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.x is None or args.y is None:
        parser.error("Supply both --x and --y, or use --self-test.")
    if not all(math.isfinite(v) for v in (args.x, args.y, args.timeout)) or args.timeout <= 0:
        parser.error("Coordinates must be finite numbers and timeout must be positive.")
    return run_ros((args.x, args.y), args.timeout)


if __name__ == "__main__":
    sys.exit(main())
