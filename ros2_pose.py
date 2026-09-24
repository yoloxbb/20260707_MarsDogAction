#! /usr/bin/env python3

import argparse
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

GOAL_TOPIC = '/goal_pose'
INITIAL_POSE_TOPIC = '/initialpose'
STATUS_TOPIC = '/navigate_to_pose/_action/status'

TERMINAL_STATUS = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}
STATUS_TEXT = {
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class TopicNavigator(Node):
    def __init__(self):
        super().__init__('topic_navigator')
        self.goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 10)

        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, STATUS_TOPIC, self.status_callback, status_qos)

        self.status_by_id = {}
        self.status_seen = False

    def status_callback(self, msg):
        self.status_seen = True
        for status in msg.status_list:
            self.status_by_id[bytes(status.goal_info.goal_id.uuid)] = status.status

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_nav(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.goal_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def set_initial_pose(self, x, y, oz, ow):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = oz
        msg.pose.pose.orientation.w = ow
        self.initial_pose_pub.publish(msg)

    def navigate_to_goal(self, x, y, oz, ow, accept_timeout=10.0, run_timeout=300.0):
        known = set(self.status_by_id)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = oz
        msg.pose.orientation.w = ow
        self.goal_pub.publish(msg)

        goal_id = None
        deadline = time.monotonic() + accept_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = [gid for gid in self.status_by_id if gid not in known]
            if fresh:
                goal_id = fresh[-1]
                break
        if goal_id is None:
            if not self.status_seen:
                print(f'No message on {STATUS_TOPIC}. '
                      'Check that navigation is running and the action name matches.')
            else:
                print('Goal was published but no new goal appeared in the status list.')
            return False

        last_reported = None
        deadline = time.monotonic() + run_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            status = self.status_by_id.get(goal_id)
            if status != last_reported:
                last_reported = status
                print(f'  status: {STATUS_TEXT.get(status, status)}')
            if status in TERMINAL_STATUS:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    print('Goal succeeded!')
                    return True
                if status == GoalStatus.STATUS_CANCELED:
                    print('Goal was canceled!')
                else:
                    print('Goal failed!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f'Goal did not finish within {run_timeout:.0f} s.')
        return False


def parse_arguments():
    parser = argparse.ArgumentParser(description='Send navigation test goals.')
    parser.add_argument(
        'targets',
        nargs='*',
        help='Waypoint letters to execute once, for example AB. Omit to loop ABCDE.')
    return parser.parse_args()


if __name__ == '__main__':
    cli_args = parse_arguments()
    rclpy.init()
    navigator = TopicNavigator()

    if not navigator.wait_for_nav():
        print(f'No subscriber on {GOAL_TOPIC}. Start navigation first.')
        rclpy.shutdown()
        sys.exit(1)
    navigator.spin(1.0)

    waypoints = {}
    try:
        with open('waypoints.yaml', 'r') as f:
            waypoints = (yaml.safe_load(f) or {}).get('waypoints', {})
    except FileNotFoundError:
        with open('waypoints.yaml', 'w') as f:
            yaml.dump({'waypoints': {}}, f, default_flow_style=False)

    goals = {
        'A': waypoints.get('A', [-3.715, -1.065, 0.136, 0.991]),
        'B': waypoints.get('B', [-1.855, -0.665, 0.790, 0.613]),
        'C': waypoints.get('C', [-2.961, 1.478, 0.259, 0.966]),
        'D': waypoints.get('D', [-1.905, 3.360, -0.708, 0.706]),
        'E': waypoints.get('E', [-4.039, 2.627, -0.421, 0.907]),
    }
    args = cli_args.targets
    loop_targets = False
    if args:
        targets = []
        for arg in args:
            for c in arg.upper():
                if c in goals:
                    targets.append(c)
    else:
        targets = ['A', 'B', 'C', 'D', 'E']
        loop_targets = True
        print('No target arguments provided; running A-B-C-D-E repeatedly. Press Ctrl+C to stop.')

    if not targets:
        print('No valid target names provided. Use names such as A, B, C, D, E or AB.')
        rclpy.shutdown()
        sys.exit(1)

    try:
        cycle_index = 1
        while rclpy.ok():
            if loop_targets:
                print(f'============= cycle {cycle_index}: ABCDE =============')

            for name in targets:
                if not rclpy.ok():
                    break

                x_goal, y_goal, orientation_z, orientation_w = goals[name]
                input(f'============={name}==================\n')
                success = navigator.navigate_to_goal(
                    x_goal, y_goal, orientation_z, orientation_w)
                print("Navigation result:", goals[name], success)

            if not loop_targets:
                break
            cycle_index += 1
    except KeyboardInterrupt:
        print('Navigation loop interrupted by user.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
