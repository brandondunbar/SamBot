from samBot import SamBot
import keyboard
import time

KEY_FWD = "w"
KEY_LEFT = "a"
KEY_BKWD = "s"
KEY_RIGHT = "d"


def inputLoop():
    sambot = SamBot()

    try:
        print("Ready.")
        while True:
            direction = -1
            if keyboard.is_pressed(KEY_FWD):
                direction = 0
            elif keyboard.is_pressed(KEY_LEFT):
                direction = 90
            elif keyboard.is_pressed(KEY_RIGHT):
                direction = 270
            elif keyboard.is_pressed(KEY_BKWD):
                direction = 180

            if direction >= 0:
                sambot.creep(direction)

    except KeyboardInterrupt:
        print("Interrupted. Ending...")


def main():
    print("Preparing...")    
    inputLoop()    
    print("Goodbye.")


if __name__ == '__main__':
    main()

