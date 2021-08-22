from example_interfaces.srv import AddTwoInts
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node


class MoveService(Node):
    """Handles robot movement.
    """

    def __init__(self, **kwargs):
        super().__init__('minimal_service')
        self._assign_pins(config=kwargs)
        self._gpio_setup()
        self.srv = self.create_service(AddTwoInts, 'move', self.move)

    def _assign_pins(self, **kwargs):
        """Assigns provided pin numbers to corresponding attribute.
        :param: pinLeftFwd - The pin that causes the left motor to move forward
        :param: pinLeftBkwd - The pin that causes the left motor to move backward
        :param: pinRightFwd - The pin that causes the right motor to move forward
        :param: pinRightBkwd - The pin that causes the right motor to move backward
        or
        :param: config - Holds the above parameters in dictionary
        """
        pinConfig = kwargs
        if kwargs["config"]:
            pinConfig = kwargs["config"]
        
        self.pinLeftFwd = pinConfig["pinLeftFwd"]
        self.pinLeftBkwd = pinConfig["pinLeftBkwd"]
        self.pinRightFwd = pinConfig["pinRightFwd"]
        self.pinRightBkwd = pinConfig["pinRightBkwd"]

    def _gpio_setup(self):
        """Prepares the relevant GPIO pins.
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinLeftFwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinLeftBkwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinRightFwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinRightBkwd, GPIO.OUT, initial=GPIO.LOW)


    def _set_pin(self, pin, setToHigh):
        """Sets the given pin to high or low.
        :param: pin - The pin to set.
        :param: setToHigh - Boolean, whether to set it to high or not.
        """
        value = GPIO.HIGH if setToHigh else GPIO.LOW
        GPIO.output(pin, value)

    def move(self, request, response):
        """Moves the bot in the provided direction at the given speed.
        :param: direction - The degree to head toward, 0-359, with 0 at the front of the bot.
        :param: speed - The speed to move at, 0-1.
        """
        direction = request.a
        speed = request.b
        if speed == 0:
            self._set_pin(self.pinLeftFwd, False)
            self._set_pin(self.pinLeftBkwd, False)
            self._set_pin(self.pinRightFwd, False)
            self._set_pin(self.pinRightBkwd, False)
        else:
            if direction == 0:
                self._set_pin(self.pinLeftFwd, True)
                self._set_pin(self.pinRightFwd, True)
            elif direction == 180:
                self._set_pin(self.pinLeftBkwd, True)
                self._set_pin(self.pinRightBkwd, True)
            elif direction == 90:
                self._set_pin(self.pinLeftBkwd, True)
                self._set_pin(self.pinRightFwd, True)
            elif direction == 270:
                self._set_pin(self.pinLeftFwd, True)
                self._set_pin(self.pinRightBkwd, True)
        
        self.get_logger().info(f"Moved in direction {direction} at {speed} speed.")
        response.sum = 0
        return response


def main(args=None):
    rclpy.init(args=args)
    move_service = MoveService(
            pinLeftFwd=22,
            pinLeftBkwd=27,
            pinRightFwd=9,
            pinRightBkwd=11
    )
    rclpy.spin(move_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

