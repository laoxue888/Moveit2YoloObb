# keyboard_input_node.py

import rclpy
from rclpy.node import Node
import signal
import sys
import termios
import tty

def handle_sigint(sig, frame):
    global is_running
    print("Ctrl+C detected, exiting...")
    is_running = False

def get_keyboard_input():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KeyboardInputNode(Node):

    def __init__(self):
        super().__init__('keyboard_input_node')
        self.get_logger().info('Keyboard input node initialized')
        global is_running
        is_running = True

        signal.signal(signal.SIGINT, handle_sigint)

        while is_running:
            char = get_keyboard_input()
            self.get_logger().info(f'Received character: {char}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
# 要加这个if语句，方便调试节点
if __name__ == '__main__':
    main()