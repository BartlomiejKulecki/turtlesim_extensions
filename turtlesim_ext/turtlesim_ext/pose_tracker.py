# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from turtlesim.srv import TeleportAbsolute
from turtlesim_ext_interfaces.msg import WallDist
from turtlesim_ext_interfaces.srv import DistToPoint


class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        self.prev_pose = None
        self.total_distance = 0.0
        
        self.go_through_walls = True
        self.declare_parameter('go_through_walls', self.go_through_walls)
        self.bounds = [0.01, 11.0] # turtlesim window bounds [min, max]
        
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.teleport_req = TeleportAbsolute.Request()
        
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Float32, 'travelled_distance', 10)
        self.pub_wall_dist = self.create_publisher(WallDist, 'distance_to_wall', 10)
        self.dist_srv = self.create_service(DistToPoint, 'distance_to_point', self.handle_dist_to_point)
        self.timer = self.create_timer(timer_period_sec=0.5, callback=self.timer_callback)

    def teleport(self, x, y, theta):
        self.teleport_req.x = float(x)
        self.teleport_req.y = float(y)
        self.teleport_req.theta = float(theta)
        future = self.teleport_cli.call_async(self.teleport_req)
        future.add_done_callback(lambda f: self.get_logger().info(f"Teleport done"))

    def listener_callback(self, pose_msg):
        # self.get_logger().info('I got new pose: x=%f, y=%f, theta=%f' % (pose_msg.x, pose_msg.y, pose_msg.theta))
        if self.prev_pose is not None:
            delta_dist = ((pose_msg.x - self.prev_pose.x) ** 2 + (pose_msg.y - self.prev_pose.y) ** 2) ** 0.5
            if delta_dist < 10.0:  # ignore big jumps (teleportation)
                self.total_distance += delta_dist
            # self.get_logger().info('Total distance moved: %f' % self.total_distance)
        self.prev_pose = pose_msg

        if self.go_through_walls:
            if pose_msg.x < self.bounds[0]:
                self.get_logger().info('Teleporting to the right wall')
                self.teleport(self.bounds[1], pose_msg.y, pose_msg.theta)
            elif pose_msg.x > self.bounds[1]:
                self.get_logger().info('Teleporting to the left wall')
                self.teleport(self.bounds[0], pose_msg.y, pose_msg.theta)
            elif pose_msg.y < self.bounds[0]:
                self.get_logger().info('Teleporting to the top wall')
                self.teleport(pose_msg.x, self.bounds[1], pose_msg.theta)
            elif pose_msg.y > self.bounds[1]:
                self.get_logger().info('Teleporting to the bottom wall')
                self.teleport(pose_msg.x, self.bounds[0], pose_msg.theta)
                    
    def handle_dist_to_point(self, request, response):
        if self.prev_pose is None:
            response.current_dist = -1.0
            return response
        dx = request.target_pt.x - self.prev_pose.x
        dy = request.target_pt.y - self.prev_pose.y
        dist = (dx**2 + dy**2)**0.5
        response.current_dist = float(dist)
        # self.get_logger().info('Distance to point (%f, %f) is %f' % (request.target_pt.x, request.target_pt.y, dist))
        return response
        
    def timer_callback(self):
        msg = Float32()
        msg.data = float(self.total_distance)
        self.publisher.publish(msg)
        # self.get_logger().info('Published total distance: %f' % msg.data)
        
        self.go_through_walls = self.get_parameter('go_through_walls').get_parameter_value().bool_value
        # self.get_logger().info('go_through_walls parameter is: %s' % str(self.go_through_walls))
        if self.prev_pose is not None:
            dist_left = self.prev_pose.x - self.bounds[0]
            dist_right = self.bounds[1] - self.prev_pose.x
            dist_bottom = self.prev_pose.y - self.bounds[0]
            dist_top = self.bounds[1] - self.prev_pose.y
            distances = [dist_left, dist_right, dist_top, dist_bottom]
            wall_names = ['left', 'right', 'top', 'bottom']
            min_dist = min(distances)
            min_index = distances.index(min_dist)
            wall_msg = WallDist()
            wall_msg.distance = float(min_dist)
            wall_msg.wall_name = wall_names[min_index]
            self.pub_wall_dist.publish(wall_msg)
            # self.get_logger().info('Published distance to wall: %f to the %s wall' % (wall_msg.distance, wall_msg.wall_name))
        


def main(args=None):
    rclpy.init(args=args)
    pose_tracker = PoseTracker()
    rclpy.spin(pose_tracker)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()