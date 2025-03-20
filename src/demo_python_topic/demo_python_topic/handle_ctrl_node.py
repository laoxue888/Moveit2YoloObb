import rclpy
from rclpy.node import Node
import time
import pygame

class LoopNode(Node):
    def __init__(self):
        super().__init__('loop_node')
        self.timer_period = 1  # 设置定时器周期为1秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 初始化pygame
        pygame.init()
        # 设置手柄
        # joystick = pygame.joystick.Joystick(0)
        # joystick.init()
        # print("使用的手柄名称:", joystick.get_name())

    def timer_callback(self):
        self.execute_function()

    def execute_function(self):
        """"""
        # 在这里放置您想要循环执行的函数代码
        # self.get_logger().info('Function executed at time: %f' % time.time())
        # try:
        #     while True:
        #         for event in pygame.event.get():
        #             if event.type == pygame.JOYBUTTONDOWN:
        #                 print(f"按钮按下: {event.button}")
        #             elif event.type == pygame.JOYBUTTONUP:
        #                 print(f"按钮松开: {event.button}")
        #             elif event.type == pygame.JOYAXISMOTION:
        #                 print(f"摇杆移动: 轴{event.axis} 值{event.value}")
        # except KeyboardInterrupt:
        #     print("停止读取手柄输入.")
        # finally:
        #     pygame.quit()

        ## 获取键盘输入
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    else:
                        print(f'Key {pygame.key.name(event.key)} pressed.')
def main(args=None):
    rclpy.init(args=args)
    node = LoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 要加这个if语句，方便调试节点
if __name__ == '__main__':
    main()