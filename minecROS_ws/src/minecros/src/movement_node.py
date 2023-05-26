#! /usr/bin/env python3

# imports for actionlib server and messages
import rospy
import actionlib
import minecros_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from pynput.keyboard import Key, Controller
from pynput.mouse import Button, Controller as MController
import time
import enum

# make enum for types of movement
class MovementType(enum.Enum):
    LOGOFF = 0
    LEFT_RIGHT_FARM = 1
    FORWARD_BACK_FARM = 2
    WARP_GARDEN = 3

POINTS_TO_REMEMBER = 5

def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res


class MovementController:
    # create messages that are used to publish feedback/result
    _feedback = minecros_msgs.msg.ControlMovementFeedback()
    _result = minecros_msgs.msg.ControlMovementResult()

    def __init__(self, name):
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, minecros_msgs.msg.ControlMovementAction,
                                                execute_cb=self.movement_action_CB, auto_start=False)
        self._as.start()
        self.keyboard = Controller()
        self.mouse = MController()
        self.command = None # will be a MovementType later

        self.point_history = []    
        self.sub_xyz = rospy.Subscriber("/minecros/coords", geometry_msgs.msg.Point, self.xyz_callback)
        self.LR_inital = handle_param_load("left_right_farm/inital_direction")
        print(self.LR_inital)
        print("------------------")
        self.leftClickPressed = False
        self.keyWPressed = False
        self.keyAPressed = False
        self.keySPressed = False
        self.keyDPressed = False


        rospy.loginfo("Finished initalizing movement server")


    def xyz_callback(self, msg):
        self.point_history.append(msg)
        if len(self.point_history) > POINTS_TO_REMEMBER:
            self.point_history.pop(0)

    def preempt(self):
        rospy.logwarn(f"Preempt called for movement server")
        self.keyboard.release("w")
        self.keyboard.release("a")
        self.keyboard.release("s")
        self.keyboard.release("d")
        self.mouse.release(Button.left)
        self.leftClickPressed = False
        self.keyWPressed = False
        self.keyAPressed = False
        self.keySPressed = False
        self.keyDPressed = False
        self.command = None
        self._as.set_preempted()

    # region movement functions
    # idea with these functions is we want to be able to check what the current state of the keyboard is
    # seems way harder to actually check at a system level than just keep track of it here
    def pressW(self, release=False):
        if release:
            self.keyboard.release("w")
            self.keyWPressed = False
        else:
            self.keyboard.press("w")
            self.keyWPressed = True

    def pressA(self, release=False):
        if release:
            self.keyboard.release("a")
            self.keyAPressed = False
        else:
            self.keyboard.press("a")
            self.keyAPressed = True

    def pressS(self, release=False):
        if release:
            self.keyboard.release("s")
            self.keySPressed = False
        else:
            self.keyboard.press("s")
            self.keySPressed = True

    def pressD(self, release=False):
        if release:
            self.keyboard.release("d")
            self.keyDPressed = False
        else:
            self.keyboard.press("d")
            self.keyDPressed = True

    def pressLeftClick(self, release=False):
        if release:
            self.mouse.release(Button.left)
            self.leftClickPressed = False
        else:
            self.mouse.press(Button.left)
            self.leftClickPressed = True
    # endregion
    
    def logoff():
        # TODO
        pass

    # assumes that positive 
    def movement_action_CB(self, goal):
        rospy.loginfo(f"Movement server called with goal {goal}")

        r = rospy.Rate(100)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt()
                self._as.set_preempted()
                return
            
            # handles start of a movement
            if self.command == None:
                if goal.command == minecros_msgs.msg.ControlMovement.LOGOFF:
                    rospy.loginfo_once("Starting logoff")
                    self.logoff()
                    self.preempt()
                    self._result.success = True
                    self._result.timed_out = False
                    self._as.set_succeeded(self._result)
                    return

                elif goal.command == minecros_msgs.msg.ControlMovement.LEFT_RIGHT_FARM:
                    rospy.loginfo_once("Starting left right farm")
                    self.command = MovementType.LEFT_RIGHT_FARM

                elif goal.command == minecros_msgs.msg.ControlMovement.FORWARD_BACK_FARM:
                    rospy.loginfo_once("Starting forward back farm")
                    self.command = MovementType.FORWARD_BACK_FARM

                elif goal.command == minecros_msgs.msg.ControlMovement.WARP_GARDEN:
                    rospy.loginfo_once("Starting warp garden")
                    self.command = MovementType.WARP_GARDEN
                else:
                    rospy.logerr(f"Unknown command {goal.command} -- Ignoring")
                    self.preempt()
                    return
                
            if self.command == MovementType.LEFT_RIGHT_FARM:
                pass
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
            

if __name__ == '__main__':
    rospy.logwarn("Initalizing movement server")
    rospy.init_node('movement_server')
    name = rospy.get_name()
    movement_server = MovementController(name)
    rospy.spin()