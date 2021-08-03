"""
movement.py
Handles the movement of the bot via GPIO pins.
Brandon Dunbar  Brandon.Dunbar97@gmail.com
"""

import Jetson.GPIO as GPIO


class Movement:
    """Movement; Handles basic movement instructions."""

    def __init__(self, pinLeftFwd, pinLeftBkwd, pinRightFwd, pinRightBkwd):
        self.pinLeftFwd = pinLeftFwd
        self.pinLeftBkwd = pinLeftBkwd
        self.pinRightFwd = pinRightFwd
        self.pinRightBkwd = pinRightBkwd
        self._gpioSetup()

    def _gpioSetup(self):
        """_gpioSetup()
        Prepares the relevant GPIO pins.
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinLeftFwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinLeftBkwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinRightFwd, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.pinRightBkwd, GPIO.OUT, initial=GPIO.LOW)

    def _setPin(self, pin, setToHigh):
        """_setPin()
        Sets the given pin to high or low.
        :param: pin - The pin to set.
        :param: setToHigh - Boolean, whether to set it to high or not.
        """
        value = GPIO.HIGH if setToHigh else GPIO.LOW
        GPIO.output(pin, value)

    def move(self, direction, speed):
        """move()
        Moves the bot in the provided direction at the given speed.
        :param: direction - The degree to head toward, 0-359, with 0 at the front of the bot.
        :param: speed - The speed to move at, 0-1.
        """
        if speed == 0:
            self._setPin(self.pinLeftFwd, False)
            self._setPin(self.pinLeftBkwd, False)
            self._setPin(self.pinRightFwd, False)
            self._setPin(self.pinRightBkwd, False)
        else:
            if direction == 0:
                self._setPin(self.pinLeftFwd, True)
                self._setPin(self.pinRightFwd, True)
            elif direction == 180:
                self._setPin(self.pinLeftBkwd, True)
                self._setPin(self.pinRightBkwd, True)
            elif direction == 90:
                self._setPin(self.pinLeftBkwd, True)
                self._setPin(self.pinRightFwd, True)
            elif direction == 270:
                self._setPin(self.pinLeftFwd, True)
                self._setPin(self.pinRightBkwd, True)

