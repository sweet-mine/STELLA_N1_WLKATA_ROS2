import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
import struct
import mmap
import os
import threading

class ShmWriter:
    def __init__(self, path, size):
        self.fd = os.open(path, os.O_CREAT | os.O_TRUNC | os.O_RDWR)
        os.ftruncate(self.fd, size)
        self.mmap = mmap.mmap(self.fd, size, access=mmap.ACCESS_WRITE)
        self.size = size
        self.version = 0
        self.lock = threading.Lock()

    def write(self, data: bytes):
        with self.lock:
            self.version = (self.version + 1) % (2**32)
            self.mmap.seek(0)
            self.mmap.write(struct.pack('I', self.version))  # write version (4 bytes)
            self.mmap.write(data.ljust(self.size - 4, b'\x00'))  # padding

    def close(self):
        self.mmap.close()
        os.close(self.fd)


class ROS2SharedMemoryBridge(Node):
    def __init__(self):
        super().__init__('stella_wlkata_node')

        self.pose_shm = ShmWriter('/dev/shm/ros_bridge_pose', 64)
        self.js_shm = ShmWriter('/dev/shm/ros_bridge_jointstate', 256)
        self.direct_shm = ShmWriter('/dev/shm/ros_bridge_direct', 256)
        self.gripper_shm = ShmWriter('/dev/shm/ros_bridge_gripper', 10)
        self.homing_shm = ShmWriter('/dev/shm/ros_bridge_homing', 10)

        self.create_subscription(Pose, '/pose', self.pose_cb, 10)
        self.create_subscription(JointState, '/jointstate', self.js_cb, 10)
        self.create_subscription(String, '/direct', self.direct_cb, 10)
        self.create_subscription(Bool, '/gripper', self.gripper_cb, 10)
        self.create_subscription(Bool, '/homing', self.homing_cb, 10)

    def pose_cb(self, msg):
        data = struct.pack('7f',
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.pose_shm.write(data)


    def js_cb(self, msg):
        # For simplicity, store only first 6 joint positions
        joints = msg.position[:6] if msg.position else [0.0] * 6
        data = struct.pack('6f', *joints)
        self.js_shm.write(data)

    def direct_cb(self, msg):
        self.direct_shm.write(msg.data.encode('utf-8'))

    def gripper_cb(self, msg):
        self.gripper_shm.write(b'\x01' if msg.data else b'\x00')

    def homing_cb(self, msg):
        self.homing_shm.write(b'\x01' if msg.data else b'\x00')

    def destroy(self):
        self.pose_shm.close()
        self.js_shm.close()
        self.direct_shm.close()
        self.gripper_shm.close()
        self.homing_shm.close()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2SharedMemoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()