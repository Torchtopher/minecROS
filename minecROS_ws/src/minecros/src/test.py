from pynput.mouse import Button, Controller

mouse = Controller()

# Read pointer position
print('The current pointer position is {0}'.format(
    mouse.position))
mouse.position = (1051, 299)
print('The current pointer position is {0}'.format(
    mouse.position))
mouse.press(Button.left)
mouse.release(Button.left)
import pyautogui

myScreenshot = pyautogui.screenshot()
myScreenshot.save(r'test.png')
print(pyautogui.size())
