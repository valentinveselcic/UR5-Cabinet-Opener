#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityFactory

class CabinetSpawner(Node):
    def __init__(self, urdf_path):
        super().__init__('cabinet_spawner')

        # Publisher to Gazebo Harmonic
        self.pub = self.create_publisher(
            EntityFactory,
            '/world/default/create',
            10
        )

        self.urdf_path = urdf_path

        # Delay spawning slightly so node has time to connect
        self.timer = self.create_timer(1.0, self.spawn_once)

        self.has_spawned = False

    def spawn_once(self):
        if self.has_spawned:
            return

        self.has_spawned = True

        # Load URDF text
        with open(self.urdf_path, 'r') as f:
            urdf_xml = f.read()

        msg = EntityFactory()
        msg.name = "cabinet"
        msg.allow_renaming = True
        msg.sdf = urdf_xml

        # Initial pose
        msg.pose.position.x = 5.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info(f"Spawn request sent for URDF: {self.urdf_path}")


def main(args=None):
    rclpy.init(args=args)

    urdf_path = "/home/user/nrs_ws/cabinet.urdf"
    node = CabinetSpawner(urdf_path)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()