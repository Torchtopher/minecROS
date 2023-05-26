import rospy
import threading
import queue
import time

def read_kbd_input(inputQueue):
    print('Ready for keyboard input:')
    while (True):
        input_str = input()
        inputQueue.put(input_str)

def main():
    inputQueue = queue.Queue()

    inputThread = threading.Thread(target=read_kbd_input, args=(inputQueue,), daemon=True)
    inputThread.start()

    while (True):
        if (inputQueue.qsize() > 0):
            input_str = inputQueue.get()
            rospy.loginfo("input_str = {}".format(input_str))


            
            # Insert your code here to do whatever you want with the input_str.

        # The rest of your program goes here.

        time.sleep(0.01) 

if __name__ == '__main__':
    main()