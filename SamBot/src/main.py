"""
main.py
SamBot's main Python file, executed on start.
Brandon Dunbar  Brandon.Dunbar97@gmail.com
"""

from server import *
from samBot import SamBot
import time


def listen():
    print("Ready...")
    app.run(debug=True)


def main():
    print("SamBot starting.")
    samBot = SamBot()
    samBot.move(0, 1)
    time.sleep(1)
    samBot.move(0, 0)
    listen()
    print("SamBot ending.")


if __name__ == "__main__":
    main()

