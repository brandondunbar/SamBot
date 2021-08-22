"""
Provides keyboard interfacing.
"""
import keyboard
import sys
import time
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


KEY_FWD = 'w'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_BKWD = 's'


class Keyboard(Node):
    """Translates keyboard input to movement instructions, which are sent to
    the movement service."""

    def __init__(self):
        super().__init__('keyboard')
        self.cli = self.create_client(AddTwoInts, 'move')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Unable to reach movement...')
        self.req = AddTwoInts.Request()
        self.lastMoveCommand = (-1, -1)

    def listen(self):
        """Listens for keystrokes"""
        try:
            direction = -1
            if keyboard.is_pressed(KEY_FWD):
                direction = 0
            elif keyboard.is_pressed(KEY_LEFT):
                direction = 90
            elif keyboard.is_pressed(KEY_RIGHT):
                direction = 270
            elif keyboard.is_pressed(KEY_BKWD):
                direction = 180

            speed = 0
            if direction >= 0:
                speed = 1
            
            if self.lastMoveCommand != (direction, speed):
                self.send_request(direction, 1)
                self.lastMoveCommand = (direction, speed)

        except KeyboardInterrupt:
            print("Interrupted. Ending...")
            raise(KeyboardInterrupt)

    def send_request(self, direction, speed):
        """Sends the movement request to the movement service."""
        self.req.a = direction
        self.req.b = speed
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    keyboard = Keyboard()
    while True:
        try:
            keyboard.listen()
        except KeyboardInterrupt:
            break

    while rclpy.ok():
        rclpy.spin_once(keyboard)
        if keyboard.future.done():
            
            try:
                response = keyboard.future.result()
            
            except Exception as e:
                keyboard.get_logger().info(
                        'Service call failed %r' % (e, ))
            else:
                keyboard.get_logger().info(
                        f"\nExited with response from movement service: {response.sum}"
                )
            break

    keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

