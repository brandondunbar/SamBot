"""
main.py
SamBot's main Python file, executed on start.
Brandon Dunbar  Brandon.Dunbar97@gmail.com
"""

from server import *
from samBot import SamBot
import time


def listen():
    print(">> Ready...")
    app.run(debug=True)


def moveTest(samBot):
    print(">> Testing movement...")
    # Forward
    samBot.move(0, 1)
    time.sleep(.5)
    samBot.move(0, 0)

    # Backward
    samBot.move(180, 1)
    time.sleep(.5)
    samBot.move(0, 0)

    # Left
    samBot.move(90, 1)
    time.sleep(.5)
    samBot.move(0, 0)

    # Right
    samBot.move(270, 1)
    time.sleep(.5)
    samBot.move(0, 0)
    print(">> Test complete.")


def main():
    print(">> SamBot starting.")
    samBot = SamBot()
    moveTest(samBot)
    listen()
    print(">> SamBot ending.")


if __name__ == "__main__":
    main()

