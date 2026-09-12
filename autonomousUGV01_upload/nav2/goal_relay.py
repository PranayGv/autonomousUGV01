#!/usr/bin/env python3
"""
goal_relay.py -- bridge RViz's "2D Goal Pose" (topic) to Nav2's NavigateToPose action.

Point-LIO's RViz config publishes goals on /goal_pose (a PoseStamped topic), but
Nav2's bt_navigator takes a NavigateToPose *action*. This node forwards each
/goal_pose (and /clicked_point) to the action server, so you can click goals in
the existing RViz without the Nav2 panel.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose


class GoalRelay(Node):
    def __init__(self):
        super().__init__("goal_relay")
        self.cli = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PoseStamped, "/goal_pose", self.on_pose, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.on_point, 10)
        self.get_logger().info("goal_relay: /goal_pose,/clicked_point -> navigate_to_pose action")

    def send(self, ps):
        if not self.cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("navigate_to_pose action server not up yet")
            return
        goal = NavigateToPose.Goal(); goal.pose = ps
        self.cli.send_goal_async(goal)
        self.get_logger().info(f"goal sent: ({ps.pose.position.x:+.2f}, {ps.pose.position.y:+.2f})")

    def on_pose(self, m):
        self.send(m)

    def on_point(self, m):
        ps = PoseStamped(); ps.header = m.header
        ps.pose.position.x = m.point.x; ps.pose.position.y = m.point.y
        ps.pose.orientation.w = 1.0
        self.send(ps)


def main():
    rclpy.init(); n = GoalRelay()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
