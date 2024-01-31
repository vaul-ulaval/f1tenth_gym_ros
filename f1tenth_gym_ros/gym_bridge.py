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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.num_agents = self.declare_parameter('number_of_agents').value
        self.agent_names = self.declare_parameter('agent_names').value
        self.scan_distance_to_base_link = self.declare_parameter('scan_distance_to_base_link').value
        self.scan_fov = self.declare_parameter('scan_fov').value
        self.scan_beams = self.declare_parameter('scan_beams').value
        map_path = self.declare_parameter('map_path').value
        map_img_ext = self.declare_parameter('map_img_ext').value
        update_rate = self.declare_parameter('update_rate').value

        # check num_agents
        if self.num_agents < 1:
            raise ValueError('num_agents should be higher than one.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=map_path,
                            map_ext=map_img_ext,
                            num_agents=self.num_agents)

        # Init robot poses
        self.init_robots()       

        # Publishers
        self.init_publishers()

        # Subscribers
        self.init_subscribers()

        # Timers
        self.drive_timer = self.create_timer(1.0 / update_rate, self.sim_update_callback)

        # TF broadcaster
        self.br = TransformBroadcaster(self)
            

    def init_robots(self):
        # Init robot poses
        poses = []
        for name in self.agent_names:
            pose_x = self.declare_parameter(f"agents.{name}.pose_x").value
            pose_y = self.declare_parameter(f"agents.{name}.pose_y").value
            pose_theta = self.declare_parameter(f"agents.{name}.pose_theta").value
            poses.append([pose_x, pose_y, pose_theta])
        self.obs, _ , self.done, _ = self.env.reset(np.array(poses))

        # Scan params
        self.angle_min = -self.scan_fov / 2.
        self.angle_max = self.scan_fov / 2.
        self.angle_inc = self.scan_fov / self.scan_beams 


    def init_publishers(self):
        self.scan_pubs = []
        self.odom_pubs = []
        self.steer_commands = np.zeros(self.num_agents)
        self.speed_commands = np.zeros(self.num_agents)
        self.drive_published = np.zeros(self.num_agents, dtype=bool)
        for agent_name in self.agent_names:
            scan_pub = self.create_publisher(
                LaserScan,
                agent_name + '/scan',
                10)
            odom_pub = self.create_publisher(
                Odometry,
                agent_name + '/odom',
                10)
            self.scan_pubs.append(scan_pub)
            self.odom_pubs.append(odom_pub) 


    def init_subscribers(self):   
        for i, agent_name in enumerate(self.agent_names):
            self.create_subscription(
                AckermannDriveStamped,
                agent_name + '/drive',
                lambda x, idx=i: self.drive_callback(x, idx),
                10)
            self.create_subscription(
                PoseWithCovarianceStamped,
                agent_name + '/set_pose',
                lambda x, idx=i: self.reset_callback(x, idx),
                10)
            self.create_subscription(
                Twist,
                agent_name + '/cmd_vel',
                lambda x, idx=i: self.teleop_callback(x, idx),
                10)
            

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
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        poses = []
        for i in range(self.num_agents):
            if i == agent_idx:
                pose = [rx, ry, rtheta]
            else:
                pose = [self.obs['poses_x'][i], self.obs['poses_y'][i], self.obs['poses_theta'][i]]
            poses.append(pose)
        self.obs, _ , self.done, _ = self.env.reset(np.array(poses))


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
            self._publish_transforms(timestamp, i)
            self._publish_wheel_transforms(timestamp, i)
            self._publish_laser_transforms(timestamp, i)       # TODO: check if this is necessary


    def _update_sim_state(self):
        self.obs, _, self.done, _ = self.env.step(np.array([self.steer_commands, self.speed_commands]).T)


    def _publish_scan(self, timestamp, agent_idx):
        scan = LaserScan()
        scan.header.stamp = timestamp
        scan.header.frame_id = self.agent_names[agent_idx] + '/laser'

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = list(self.obs['scans'][agent_idx])

        self.scan_pubs[agent_idx].publish(scan)


    def _publish_odom(self, timestamp, agent_idx):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = self.agent_names[agent_idx] + '/base_link'
        odom_msg.pose.pose.position.x = self.obs['poses_x'][agent_idx]
        odom_msg.pose.pose.position.y = self.obs['poses_y'][agent_idx]
        odom_msg.pose.pose.position.z = 0.0

        quat = euler.euler2quat(0., 0., self.obs['poses_theta'][agent_idx], axes='sxyz')
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]

        odom_msg.twist.twist.linear.x = self.obs['linear_vels_x'][agent_idx]
        odom_msg.twist.twist.linear.y = self.obs['linear_vels_y'][agent_idx]
        odom_msg.twist.twist.angular.z = self.obs['ang_vels_z'][agent_idx]

        self.odom_pubs[agent_idx].publish(odom_msg)


    def _publish_transforms(self, timestamp, agent_idx):
        tf = Transform()
        tf.translation.x = self.obs['poses_x'][agent_idx]
        tf.translation.y = self.obs['poses_y'][agent_idx]
        tf.translation.z = 0.0

        quat = euler.euler2quat(0.0, 0.0, self.obs['poses_theta'][agent_idx], axes='sxyz')
        tf.rotation.x = quat[1]
        tf.rotation.y = quat[2]
        tf.rotation.z = quat[3]
        tf.rotation.w = quat[0]

        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = timestamp
        tf_stamped.header.frame_id = 'map'
        tf_stamped.child_frame_id = self.agent_names[agent_idx] + '/base_link'
        tf_stamped.transform = tf

        self.br.sendTransform(tf_stamped)
        

    def _publish_wheel_transforms(self, timestamp, agent_idx):
        wheel_tf = TransformStamped()
        wheel_tf.header.stamp = timestamp
        ego_wheel_quat = euler.euler2quat(0., 0., self.steer_commands[agent_idx], axes='sxyz')
        wheel_tf.transform.rotation.x = ego_wheel_quat[1]
        wheel_tf.transform.rotation.y = ego_wheel_quat[2]
        wheel_tf.transform.rotation.z = ego_wheel_quat[3]
        wheel_tf.transform.rotation.w = ego_wheel_quat[0]

        wheel_tf.header.frame_id = self.agent_names[agent_idx] + '/front_left_hinge'
        wheel_tf.child_frame_id = self.agent_names[agent_idx] + '/front_left_wheel'
        self.br.sendTransform(wheel_tf)

        wheel_tf.header.frame_id = self.agent_names[agent_idx] + '/front_right_hinge'
        wheel_tf.child_frame_id = self.agent_names[agent_idx] + '/front_right_wheel'
        self.br.sendTransform(wheel_tf)


    def _publish_laser_transforms(self, timestamp, agent_idx):
        laser_tf = TransformStamped()
        laser_tf.header.stamp = timestamp
        laser_tf.transform.translation.x = self.scan_distance_to_base_link
        laser_tf.transform.rotation.w = 1.
        laser_tf.header.frame_id = self.agent_names[agent_idx] + '/base_link'
        laser_tf.child_frame_id = self.agent_names[agent_idx] + '/laser'
        self.br.sendTransform(laser_tf)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
