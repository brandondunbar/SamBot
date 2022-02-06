"""
Provides keyboard interfacing.
"""
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import termios
import tty


startup_msg = """

This message provides keyboard interfacing for remote operation.
---------------
Movement:
    w
a   s   d
---------------
Ctrl-C to quit
"""

CONN_TIMEOUT_S = 1.0
INTERRUPT_CODE = b'\x03'

key_bindings = {
    'w'.encode(): 0,
    'a'.encode(): 90,
    's'.encode(): 180,
    'd'.encode(): 270
}


class Keyboard(Node):
    """Translates keyboard input to movement instructions, which are sent to
    the movement service."""

    def __init__(self):
        super().__init__('keyboard')
        # Create client
        self.cli = self.create_client(AddTwoInts, 'move')
        # Connect
        self.connect()
        # Set initial state
        self.req = AddTwoInts.Request()
        self.lastMoveCommand = (-1, -1)
        # Save current termios settings, to be replaced after execution
        self.save_terminal_settings()

    def connect(self):
        """Attempts to connect to the service."""
        while not self.cli.wait_for_service(timeout_sec=CONN_TIMEOUT_S):
            self.get_logger().info('Unable to reach movement...')
        self.get_logger().info('Connected.')

    def save_terminal_settings(self):
        """Saves the current termios settings to class attribute, allowing them
        to be restored after execution."""
        self.termios_settings = {}
        self.termios_settings['fd'] = sys.stdin.fileno()
        self.termios_settings['saved_settings'] = termios.tcgetattr(self.termios_settings['fd'])

    def restore_terminal_settings(self):
        """Sets the termios settings to the settings saved in the class
        attribute."""
        termios.tcsetattr(
            self.termios_settings['fd'],
            termios.TCSADRAIN,
            self.termios_settings['saved_settings'])

    def listen(self):
        """Listens for keystrokes"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.buffer.raw.read(1)  # Get key
            if key == INTERRUPT_CODE: raise KeyboardInterrupt
            direction = -1
            if key in key_bindings:
                direction = key_bindings[key]
            else:
                self.get_logger().info(f"Unrecognized key {key}. Must be one of {key_bindings.keys()}\r")
            speed = 0
            if direction >= 0:
                speed = 1
            self.get_logger().info(f"Moving in {direction} direction, at {speed} speed.\r")
            if self.lastMoveCommand != (direction, speed):
                self.send_request(direction, 1)
                self.lastMoveCommand = (direction, speed)
        finally:
            self.restore_terminal_settings()

    def send_request(self, direction, speed):
        """Sends the movement request to the movement service.
        Args:
            int direction - The direction (in degrees) to move in, from 0 to 359.
            float speed - The speed at which to move. Currently only 0 and 1 are supported."""
        self.req.a = direction
        self.req.b = speed
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    keyboard = Keyboard()
    keyboard.get_logger().info(startup_msg)
    
    while True:
        try:
            keyboard.listen()
        except KeyboardInterrupt:
            print("Interrupted. Ending...")
            break

    keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

