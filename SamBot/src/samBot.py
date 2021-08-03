"""
samBot.py
Contains the SamBot class, a high-level representation of the bot.
Brandon Dunbar brandon.dunbar97@gmail.com
"""

import time
from movement import Movement

config = {
    "PIN_LEFT_FWD": 22,
    "PIN_LEFT_BKWD": 27,
    "PIN_RIGHT_FWD": 9,
    "PIN_RIGHT_BKWD": 11,
    "SLEEP_TIME": .5
}


class SamBot:
    """SamBot; allows high-level interfacing with the bot."""

    def __init__(self):
        self.movement = Movement(
                config["PIN_LEFT_FWD"],
                config["PIN_LEFT_BKWD"],
                config["PIN_RIGHT_FWD"],
                config["PIN_RIGHT_BKWD"]
            )

    def move(self, direction, speed):
        self.movement.move(direction, speed)
    
    def creep(self, direction):
        self.move(direction, 1)
        time.sleep(config["SLEEP_TIME"])
        self.move(0, 0)


