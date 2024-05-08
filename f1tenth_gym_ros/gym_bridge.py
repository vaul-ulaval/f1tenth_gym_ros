# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler


class GymBridge(Node):
    def __init__(self):
        super().__init__("gym_bridge")  # type: ignore

        self.init_parameters();

        # env backend
        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.map_path,
            map_ext=self.map_img_ext,
            num_agents=self.num_agents,
            timestep=1/self.physics_rate
        )

        # Init robot poses
        self.init_robots()

        # Publishers
        self.init_publishers()

        # Subscribers
        self.init_subscribers()

        # Timers
        if self.scan_rate is None:
            raise ValueError("update_rate is not initialized")
        self.drive_timer = self.create_timer(
            1.0 / self.scan_rate, self.sim_update_callback
        )

    
    def init_parameters(self):

        self.num_agents: int = self.declare_parameter("number_of_agents", 1).value
        self.agent_names: list[str] = self.declare_parameter("agent_names", [""]).value

        if self.num_agents == 1:
            self.agent_names = [""]
        else: 
            self.agent_names = [self.agent_names[i] + "/" for i in range(0, self.num_agents)]
            
        self.start_xs: list[float] = self.declare_parameter("start_xs", [0.0]).value
        self.start_ys: list[float] = self.declare_parameter("start_ys", [0.0]).value
        self.start_thetas: list[float] = self.declare_parameter("start_thetas", [0.0]).value

        self.scan_fov: float = self.declare_parameter("scan_fov", 4.71238898038469).value
        self.scan_beams: int = self.declare_parameter("scan_beams", 1081).value

        self.map_path: str = self.declare_parameter("map_path").value
        self.map_img_ext: str = self.declare_parameter("map_img_ext").value

        self.scan_rate: float = self.declare_parameter("scan_rate", 40.0).value
        self.physics_rate: float = self.declare_parameter("physics_rate", 100.0).value
        self.publish_tf: bool = self.declare_parameter("publish_tf", True).value
        self.tf_frame: str = self.declare_parameter("tf_frame", "map").value

        self.odom_topic: str = self.declare_parameter("odom_topic", "odom").value
        self.scan_topic: str = self.declare_parameter("scan_topic", "scan").value

    def init_robots(self):
        # Init robot poses
        poses = np.array([self.start_xs, self.start_ys, self.start_thetas]).T
        self.obs, _, self.done, _ = self.env.reset(poses[0:self.num_agents])

        # Scan params
        self.angle_min = -self.scan_fov / 2.0
        self.angle_max = self.scan_fov / 2.0
        self.angle_inc = self.scan_fov / self.scan_beams

    def init_publishers(self):
        self.scan_pubs = []
        self.odom_pubs = []
        self.steer_commands = np.zeros(self.num_agents)
        self.speed_commands = np.zeros(self.num_agents)
        self.drive_published = np.zeros(self.num_agents, dtype=bool)
        for agent_name in self.agent_names:
            scan_pub = self.create_publisher(LaserScan, agent_name + self.scan_topic, 10)
            odom_pub = self.create_publisher(Odometry, agent_name + self.odom_topic, 10)
            self.scan_pubs.append(scan_pub)
            self.odom_pubs.append(odom_pub)
        self.tf_broadcaster = TransformBroadcaster(self)

    def init_subscribers(self):
        for i, agent_name in enumerate(self.agent_names):
            self.create_subscription(
                AckermannDriveStamped,
                agent_name + "drive",
                lambda x, idx=i: self.drive_callback(x, idx),
                10,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                agent_name + "initialpose",
                lambda x, idx=i: self.reset_callback(x, idx),
                10,
            )
            self.create_subscription(
                Twist,
                agent_name + "cmd_vel",
                lambda x, idx=i: self.teleop_callback(x, idx),
                10,
            )

    def drive_callback(self, drive_msg, agent_idx):
        self.speed_commands[agent_idx] = drive_msg.drive.speed
        self.steer_commands[agent_idx] = drive_msg.drive.steering_angle

    def reset_callback(self, pose_msg, agent_idx):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes="sxyz")

        poses = []
        for i in range(self.num_agents):
            if i == agent_idx:
                pose = [rx, ry, rtheta]
            else:
                pose = [
                    self.obs["poses_x"][i],
                    self.obs["poses_y"][i],
                    self.obs["poses_theta"][i],
                ]
            poses.append(pose)
        self.obs, _, self.done, _ = self.env.reset(np.array(poses))

    def teleop_callback(self, twist_msg, agent_idx):
        self.speed_commands[agent_idx] = twist_msg.linear.x
        self.steer_commands[agent_idx] = twist_msg.angular.z

    def sim_update_callback(self):
        # Update sim
        self._update_sim_state()

        # Publish scan and odom
        timestamp = self.get_clock().now().to_msg()
        for i in range(self.num_agents):
            self._publish_scan(timestamp, i)
            self._publish_odom(timestamp, i)
            self._publish_wheel_transforms(timestamp, i)
            if self.publish_tf:
                self._publish_transforms(timestamp, i)

    def _update_sim_state(self):
        self.obs, _, self.done, _ = self.env.step(
            np.array([self.steer_commands, self.speed_commands]).T
        )

    def _publish_scan(self, timestamp, agent_idx):
        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = self.agent_names[agent_idx] + "base_link"

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = list(self.obs["scans"][agent_idx])

        self.scan_pubs[agent_idx].publish(scan)

    def _publish_odom(self, timestamp, agent_idx):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.agent_names[agent_idx] + "base_link"
        odom_msg.pose.pose.position.x = self.obs["poses_x"][agent_idx]
        odom_msg.pose.pose.position.y = self.obs["poses_y"][agent_idx]
        odom_msg.pose.pose.position.z = 0.0

        quat = euler.euler2quat(
            0.0, 0.0, self.obs["poses_theta"][agent_idx], axes="sxyz"
        )
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]

        odom_msg.twist.twist.linear.x = self.obs["linear_vels_x"][agent_idx]
        odom_msg.twist.twist.linear.y = self.obs["linear_vels_y"][agent_idx]
        odom_msg.twist.twist.angular.z = self.obs["ang_vels_z"][agent_idx]

        self.odom_pubs[agent_idx].publish(odom_msg)

    def _publish_transforms(self, timestamp, agent_idx):
        tf = Transform()
        tf.translation.x = self.obs["poses_x"][agent_idx]
        tf.translation.y = self.obs["poses_y"][agent_idx]
        tf.translation.z = 0.0

        quat = euler.euler2quat(
            0.0, 0.0, self.obs["poses_theta"][agent_idx], axes="sxyz"
        )
        tf.rotation.x = quat[1]
        tf.rotation.y = quat[2]
        tf.rotation.z = quat[3]
        tf.rotation.w = quat[0]

        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = timestamp
        tf_stamped.header.frame_id = self.tf_frame
        tf_stamped.child_frame_id = self.agent_names[agent_idx] + "base_link"
        tf_stamped.transform = tf

        self.tf_broadcaster.sendTransform(tf_stamped)

    def _publish_wheel_transforms(self, timestamp, agent_idx):
        wheel_tf = TransformStamped()
        wheel_tf.header.stamp = timestamp
        ego_wheel_quat = euler.euler2quat(
            0.0, 0.0, self.steer_commands[agent_idx], axes="sxyz"
        )
        wheel_tf.transform.rotation.x = ego_wheel_quat[1]
        wheel_tf.transform.rotation.y = ego_wheel_quat[2]
        wheel_tf.transform.rotation.z = ego_wheel_quat[3]
        wheel_tf.transform.rotation.w = ego_wheel_quat[0]

        wheel_tf.header.frame_id = self.agent_names[agent_idx] + "front_left_hinge"
        wheel_tf.child_frame_id = self.agent_names[agent_idx] + "front_left_wheel"
        self.tf_broadcaster.sendTransform(wheel_tf)

        wheel_tf.header.frame_id = self.agent_names[agent_idx] + "front_right_hinge"
        wheel_tf.child_frame_id = self.agent_names[agent_idx] + "front_right_wheel"
        self.tf_broadcaster.sendTransform(wheel_tf)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)


if __name__ == "__main__":
    main()
