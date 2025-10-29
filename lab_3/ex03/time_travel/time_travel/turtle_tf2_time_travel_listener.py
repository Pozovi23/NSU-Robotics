import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn
from visualization_msgs.msg import Marker, MarkerArray


class TimeTravelListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_time_travel_listener')

        self.target_frame = self.declare_parameter(
            'target_frame', 'turtle1'
        ).get_parameter_value().string_value

        self.delay = self.declare_parameter(
            'delay', 5.0
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawner = self.create_client(Spawn, 'spawn')
        self.turtle_spawning_service_ready = False
        self.turtle_spawned = False

        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        self.marker_publisher = self.create_publisher(MarkerArray, '/current_target', 10)

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if not self.turtle_spawning_service_ready:
            if self.spawner.service_is_ready():
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
                self.get_logger().info('Spawning turtle2...')
            else:
                self.get_logger().info('Service is not ready', throttle_duration_sec=5)
            return

        if not self.turtle_spawned:
            if self.result.done():
                try:
                    response = self.result.result()
                    self.turtle_spawned = True
                    self.get_logger().info(f'Successfully spawned {response.name}')
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
            else:
                self.get_logger().info('Waiting for spawn to complete...', throttle_duration_sec=2)
            return

        try:
            now = self.get_clock().now()

            past_time = now - rclpy.time.Duration(seconds=self.delay)

            try:
                t = self.tf_buffer.lookup_transform_full(
                    target_frame=to_frame_rel,
                    target_time=rclpy.time.Time(),
                    source_frame=from_frame_rel,
                    source_time=past_time,
                    fixed_frame='world',
                    timeout=rclpy.time.Duration(seconds=0.1)
                )

                try:
                    target_in_world = self.tf_buffer.lookup_transform(
                        'world',
                        from_frame_rel,
                        past_time,
                        timeout=rclpy.time.Duration(seconds=0.1)
                    )
                    self.publish_target_marker(target_in_world.transform.translation)
                except TransformException:
                    pass

                msg = Twist()
                scale_rotation_rate = 4.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x
                )

                scale_forward_speed = 0.5
                distance = math.sqrt(
                    t.transform.translation.x ** 2 + t.transform.translation.y ** 2
                )
                msg.linear.x = scale_forward_speed * distance

                self.publisher.publish(msg)

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform: {ex}',
                    throttle_duration_sec=1
                )
                return

        except TransformException as ex:
            self.get_logger().info(
                f'Transform not ready: {ex}',
                throttle_duration_sec=1
            )
            return

    def publish_target_marker(self, translation):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = translation.x
        marker.pose.position.y = translation.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rclpy.time.Duration(seconds=0.2).to_msg()

        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)


def main():
    rclpy.init()
    node = TimeTravelListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()