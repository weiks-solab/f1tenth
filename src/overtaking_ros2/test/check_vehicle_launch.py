"""Exercise the installed live launch in an isolated ROS domain, without hardware."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from collections import Counter

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from f1tenth_overtaking_core.geometry import load_geometry
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from overtaking_msgs.msg import CandidatePaths, RelativeState, SteeringState, TargetPath
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Run the integration test in an isolated ROS domain on localhost')
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-id', choices=['my_map', 'new'], default='my_map')
    map_id = parser.parse_args().map_id
    b = Path(get_package_share_directory('overtaking_ros2')) / 'maps' / map_id / 'v1'
    geo = load_geometry(b/'geometry', b/(map_id + '.yaml'), inspect=True)
    xy, _, yaw, _ = geo.curve.sample([0., 2.])
    rclpy.init()
    n = Node('vehicle_integration_probe')
    tf = TransformBroadcaster(n)
    frames = ('base_link', 'opponent/base_link')
    pubs = [n.create_publisher(Odometry, '/overtaking/input/' + t, 1)
            for t in ('ego_odom', 'opponent_odom')]
    steering = n.create_publisher(SteeringState, '/overtaking/input/steering', 1)
    plans, commands, candidates, states = [], [], [], []
    for typ, topic, received in ((TargetPath, 'target_path', plans), (AckermannDriveStamped, 'drive', commands),
            (CandidatePaths, 'candidate_paths', candidates), (RelativeState, 'relative_state', states)):
        n.create_subscription(typ, '/overtaking/' + topic, received.append, 10)

    def spin(seconds, inputs=False, feedback=True):
        end, next_input = time.monotonic() + seconds, 0.
        while time.monotonic() < end:
            if inputs and time.monotonic() >= next_input:
                stamp = n.get_clock().now().to_msg()
                next_input = time.monotonic() + .04
                for i, pub in enumerate(pubs):
                    msg = Odometry()
                    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'map', frames[i]
                    pos = msg.pose.pose.position
                    pos.x, pos.y = map(float, xy[i])
                    q = msg.pose.pose.orientation
                    q.z, q.w = math.sin(yaw[i] / 2), math.cos(yaw[i] / 2)
                    msg.twist.twist.linear.x = .4
                    pub.publish(msg)
                    if i == 0:
                        t = TransformStamped(header=msg.header, child_frame_id=frames[0])
                        t.transform.translation.x, t.transform.translation.y = pos.x, pos.y
                        t.transform.rotation = q
                        tf.sendTransform(t)
                        f = SteeringState()
                        f.header.stamp, f.header.frame_id = stamp, frames[0]
                        if feedback:
                            steering.publish(f)
            rclpy.spin_once(n, timeout_sec=.01)

    process = subprocess.Popen(['ros2', 'launch', 'overtaking_ros2', 'vehicle.launch.py', 'map_id:=' + map_id],
                               start_new_session=True)
    try:
        spin(6)
        assert process.poll() is None and plans and commands
        assert all(p.mode == p.STOP for p in plans) and all(c.drive.speed == 0. for c in commands)
        assert {(x, '/overtaking') for x in ('map_assets', 'relative_state', 'path_generation',
            'overtaking_decision', 'pure_pursuit')} <= set(n.get_node_names_and_namespaces())
        assert n.count_publishers('/drive') == 0
        spin(3, True, False)
        assert candidates and not any(s.rollout_valid for s in states)
        assert any(p.reason == 'missing_or_invalid_rollout_feedback' for p in plans)
        assert all(c.drive.speed == 0. for c in commands)
        candidates.clear()
        plans.clear()
        start = time.monotonic()
        spin(12, True)
        assert any(s.rollout_valid and abs(s.delta_s_local_m - 2.) < .01 for s in states)
        assert candidates and all([p.candidate_id for p in c.paths] == [1, 2, 3, 4, 5] for c in candidates)
        valid = [p for p in plans if p.mode != p.STOP]
        print('Targets by reason', Counter(p.reason for p in plans), 'candidate batches', len(candidates), flush=True)
        assert valid and all(p.points and p.plan_id == p.state_id for p in valid)
        assert any(c.drive.speed > 0 for c in commands)
        spin(.6)
        assert plans[-1].mode == TargetPath.STOP and commands[-1].drive.speed == 0.
        print('Verified five nodes, timed plans, tracking and input-loss stop', round(time.monotonic()-start, 2))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=12)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
