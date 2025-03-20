# joint_state_listener.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard joint state:')
        for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.get_logger().info(f'  - {name}: position={position}, velocity={velocity}, effort={effort}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 要加这个if语句，方便调试节点
if __name__ == '__main__':
    main()