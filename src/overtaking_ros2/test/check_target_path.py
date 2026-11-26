"""Exercise car_control's local-path mode with ROS TF and a fake drive consumer."""
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from car_control.pure_pursuit_tf_ackermann import PurePursuit
from geometry_msgs.msg import Point, TransformStamped
from overtaking_msgs.msg import TargetPath
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init(args=['--ros-args', '-p', 'path_source:=target_path',
                     '-p', 'target_max_age:=0.3', '-p', 'pose_max_age:=0.3',
                     '-p', 'geometry_id:=test-geometry'])
    controller = PurePursuit()
    fixture = Node('target_path_fixture', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(fixture)
    publisher = fixture.create_publisher(TargetPath, '/overtaking/target_path', 1)
    broadcaster = TransformBroadcaster(fixture)
    commands = []
    fixture.create_subscription(
        AckermannDriveStamped, '/overtaking/drive', commands.append, 10)

    def spin_for(seconds, send_target=False, send_tf=True, frame='map'):
        deadline = time.monotonic() + seconds
        next_send = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                next_send = time.monotonic() + 0.05
                now = fixture.get_clock().now()
                if send_tf:
                    transform = TransformStamped()
                    transform.header.stamp = now.to_msg()
                    transform.header.frame_id = 'map'
                    transform.child_frame_id = 'base_link'
                    transform.transform.rotation.w = 1.0
                    broadcaster.sendTransform(transform)
                if send_target:
                    target = TargetPath()
                    target.header.stamp = now.to_msg()
                    target.header.frame_id = frame
                    target.plan_id = str(now.nanoseconds)
                    target.state_id = target.plan_id
                    target.geometry_id = 'test-geometry'
                    target.valid_until = (now + Duration(seconds=0.3)).to_msg()
                    target.mode = TargetPath.FOLLOW
                    target.target_speed_mps = 0.4
                    target.points = [Point(x=x) for x in (0.0, 0.5, 1.0, 1.5)]
                    publisher.publish(target)
            executor.spin_once(timeout_sec=0.02)

    try:
        spin_for(0.6)
        assert commands and commands[-1].drive.speed == 0.0
        commands.clear()
        spin_for(6.0, send_target=True)
        assert any(abs(msg.drive.speed - 0.4) < 1e-6 for msg in commands), 'no tracking command'
        spin_for(0.6, send_target=True, frame='invalid')
        assert commands[-1].drive.speed == 0.0, 'wrong frame produced motion'
        spin_for(0.6, send_target=True)
        assert abs(commands[-1].drive.speed - 0.4) < 1e-6
        spin_for(0.6, send_target=True, send_tf=False)
        assert commands[-1].drive.speed == 0.0, 'stale TF produced motion'
        spin_for(0.6, send_target=True)
        assert abs(commands[-1].drive.speed - 0.4) < 1e-6
        spin_for(0.6)
        assert commands[-1].drive.speed == 0.0, 'expired target produced motion'
        print('Existing Pure Pursuit tracks TargetPath and stops on invalid input, stale TF or expiry.')
    finally:
        controller.destroy_node()
        fixture.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
