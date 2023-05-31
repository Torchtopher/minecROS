from pynput.keyboard import Key, Controller
from pynput.mouse import Button, Controller as MController
import time 
keyboard = Controller()
mouse = MController()

time.sleep(3)
keyboard.press(Key.esc)
keyboard.release(Key.esc)
#mouse.move(100, 100)
mouse.position = (520, 527)
print("Click")
mouse.click(Button.left)
mouse.release(Button.left)