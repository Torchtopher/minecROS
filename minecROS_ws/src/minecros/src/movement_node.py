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
import random
from collections import namedtuple

# xz point
Point = namedtuple("Point", ["x", "z"])

# make enum for types of movement
class MovementType(enum.Enum):
    LOGOFF = 0
    LEFT_RIGHT_FARM = 1
    FORWARD_BACK_FARM = 2
    WARP_GARDEN = 3

POINTS_TO_REMEMBER = 10
OCR_HZ = 10 # the rate we get points from the OCR node
STRICT_AXIS_THRESH = 1 # if the axis is off by more than this, don't fly up
REGULAR_AXIS_THRESH = 2

def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res

def inverse_axis(axis: str):
    if axis == "X":
        return "Z"
    elif axis == "Z":
        return "X"
    else:
        assert False, "Axis must be X or Z"

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
        self.last_point_time = rospy.Time.now()

        self.sub_xyz = rospy.Subscriber("/minecros/coords", geometry_msgs.msg.Point, self.xyz_callback)
        
        # start left right params
        self.LR_inital = handle_param_load("left_right_farm/inital_direction")
        self.LR_XZ_farm = handle_param_load("left_right_farm/X_or_Z_farm")
        self.LR_Y_cutoff = handle_param_load("left_right_farm/Y_cutoff")
        self.LR_ON_LOWER = True
        assert self.LR_XZ_farm == "X" or self.LR_XZ_farm == "Z", "LR_XZ_farm must be either X or Z"
        self.LR_XZ_to_fly_up = handle_param_load("left_right_farm/XZ_to_fly_up")
        self.LR_XZ_to_fly_up = Point(self.LR_XZ_to_fly_up[0], self.LR_XZ_to_fly_up[1])
        print(self.LR_XZ_to_fly_up)

        # end left right params

        # start forward back params
        self.FB_inital = handle_param_load("forward_back_farm/inital_direction")
        self.FB_XZ_farm = handle_param_load("forward_back_farm/X_or_Z_farm")
        assert self.FB_XZ_farm == "X" or self.FB_XZ_farm == "Z", "FB_XZ_farm must be either X or Z"

        print(self.LR_inital)
        print("------------------")
        self.leftClickPressed = False
        self.keyWPressed = False
        self.keyAPressed = False
        self.keySPressed = False
        self.keyDPressed = False
        self.keySpacePressed = False


        rospy.loginfo("Finished initalizing movement server")


    def xyz_callback(self, msg):
        self.point_history.append({"X": msg.x, "Y": msg.y, "Z": msg.z}) # dynamic lookup later
        if len(self.point_history) > POINTS_TO_REMEMBER:
            self.point_history.pop(0)
        self.last_point_time = rospy.Time.now()

    def release_all_keys(self):
        self.keyboard.release("w")
        self.keyboard.release("a")
        self.keyboard.release("s")
        self.keyboard.release("d")
        self.keyboard.release(Key.space)
    
    def preempt(self):
        rospy.logwarn(f"Preempt called for movement server")
        self.release_all_keys()
        self.mouse.release(Button.left)
        self.leftClickPressed = False
        self.keyWPressed = False
        self.keyAPressed = False
        self.keySPressed = False
        self.keyDPressed = False
        self.keySpacePressed = False
        self.command = None
        self._as.set_preempted()

    def checkNotMoving(self, points_to_check, axis="X"):
        # check if the last points_to_check points are the same on the given axis
        for i in range(1, points_to_check):
            try:
                if self.point_history[-i][axis] != self.point_history[-1][axis]:
                    return False
            except IndexError:
                rospy.logerr(f"KeyError in checkNotMoving for axis={axis}")
                return False
        return True


    def rngDelay(self, min, max):
        time.sleep(random.uniform(min, max))
                   
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

    def pressSpace(self, release=False):
        if release:
            self.keyboard.release(Key.space)
            self.keySpacePressed = False
        else:
            self.keyboard.press(Key.space)
            self.keySpacePressed = True

    def pressLeftClick(self, release=False):
        if release:
            self.mouse.release(Button.left)
            self.leftClickPressed = False
        else:
            self.mouse.press(Button.left)
            self.leftClickPressed = True
    # endregion

    def shouldFlyUp(self, point: Point, strict_axis: str):
        if strict_axis == "X":
            strict_pt = point.x
            normal_pt = point.z
        elif strict_axis == "Z":
            strict_pt = point.z
            normal_pt = point.x
        else:
            assert False, "strict_axis must be X or Z"

        if abs(self.point_history[-1][strict_axis] - strict_pt) > STRICT_AXIS_THRESH:
            return False
        if abs(self.point_history[-1][inverse_axis(strict_axis)] - normal_pt) > REGULAR_AXIS_THRESH:
            return False
        return True
    
    def clearPoints(self):
        self.point_history = []

    # assumes that positive 
    def movement_action_CB(self, goal):
        rospy.loginfo(f"Movement server called with goal {goal}! Waiting 3 seconds to start")
        time.sleep(3)
        self.clearPoints()
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
                if goal.command == minecros_msgs.msg.ControlMovementGoal.LOGOFF:
                    rospy.loginfo_once("Starting logoff")
                    self.logoff()
                    self.preempt()
                    self._result.success = True
                    self._result.timed_out = False
                    self._as.set_succeeded(self._result)
                    return

                elif goal.command == minecros_msgs.msg.ControlMovementGoal.LEFT_RIGHT_FARM:
                    rospy.loginfo_once("Starting left right farm")
                    self.command = MovementType.LEFT_RIGHT_FARM
                    self.pressW()
                    self.pressD()
                    self.rngDelay(0.1, 0.2)
                    self.pressLeftClick()
                    self.rngDelay(0.3, 0.5)
                    self.pressW(release=True)

                elif goal.command == minecros_msgs.msg.ControlMovementGoal.FORWARD_BACK_FARM:
                    rospy.loginfo_once("Starting forward back farm")
                    self.command = MovementType.FORWARD_BACK_FARM
                    self.pressD()
                    self.rngDelay(0.1, 0.2)
                    self.pressLeftClick()
                    self.rngDelay(0.3, 0.5)

                elif goal.command == minecros_msgs.msg.ControlMovementGoal.WARP_GARDEN:
                    rospy.loginfo_once("Starting warp garden")
                    self.command = MovementType.WARP_GARDEN
                else:
                    rospy.logerr(f"Unknown command {goal.command} -- Ignoring")
                    self.preempt()
                    return
                
            if self.command == MovementType.FORWARD_BACK_FARM:
                # check if last 3 points are the same
                if len(self.point_history) != POINTS_TO_REMEMBER:
                    rospy.logwarn_once("Waiting for points")
                    continue

                rospy.loginfo_throttle(1, f"Current point: {self.point_history}")
                # print last 3 points on FB_XZ_farm

                # checks if we are not moving for 1 second
                if self.checkNotMoving(2, self.FB_XZ_farm):
                    rospy.loginfo("Detected wall! Moving to next row")
                    if self.keyDPressed:
                        self.clearPoints()
                        self.pressD(release=True)
                        self.rngDelay(0.1, 0.2)
                        self.pressS()

                    elif self.keySPressed:
                        self.clearPoints()
                        self.pressS(release=True)
                        self.rngDelay(0.1, 0.2)
                        self.pressD()
            
            elif self.command == MovementType.LEFT_RIGHT_FARM:
                if len(self.point_history) != POINTS_TO_REMEMBER:
                    rospy.logwarn_once("Waiting for points")
                    continue

                rospy.loginfo_throttle(1, f"Current point: {self.point_history}")

                # check if Y has dropped from cutoff
                if self.point_history[-1]["Y"] < self.LR_Y_cutoff and not self.LR_ON_LOWER:
                    rospy.loginfo("Detected drop! Executing backup")
                    self.LR_ON_LOWER = True
                    self.release_all_keys()
                    self.rngDelay(0.05, 0.1)
                    self.pressS()
                    r = rospy.Rate(OCR_HZ)
                    self.clearPoints()
                    # poll until X or Z stops changing
                    rospy.loginfo("Waiting for X or Z to stop changing")
                    while not rospy.is_shutdown():
                        if self.checkNotMoving(OCR_HZ, inverse_axis(self.LR_XZ_farm)):
                            break
                        r.sleep()
                    rospy.loginfo("Continuing movement")
                    self.clearPoints()
                    self.pressS(release=True)
                    self.pressD()
                    self.rngDelay(0.05, 0.1)
                    self.pressW()
                    self.rngDelay(0.3, 0.5)
                    self.pressW(release=True)
                
                if self.LR_ON_LOWER and self.shouldFlyUp(self.LR_XZ_to_fly_up, self.LR_XZ_farm): 
                    rospy.loginfo("Detected fly up! Executing fly up")
                    self.LR_ON_LOWER = False
                    self.release_all_keys()
                    # double tap space
                    # press space, wait 50ms and release space wait 50ms and press space again
                    self.pressSpace()
                    time.sleep(0.05)
                    self.pressSpace(release=True)
                    time.sleep(0.05)
                    self.pressSpace()
                    time.sleep(0.05)
                    self.pressSpace(release=True)
                    time.sleep(0.05)
                    self.pressSpace()
                    self.rngDelay(0.05, 0.1)
                    self.pressSpace(release=True)
                    # start holding right
                    self.pressD()
                    self.rngDelay(0.05, 0.1)
                    self.pressW()
                    self.rngDelay(0.1, 0.1)
                    # press and release shift
                    self.keyboard.press(Key.shift)
                    self.rngDelay(0.05, 0.1)
                    self.keyboard.release(Key.shift)
                    self.rngDelay(0.05, 0.1)
                    self.pressW(release=True)
                    self.clearPoints()

                if len(self.point_history) != POINTS_TO_REMEMBER:
                    rospy.logwarn_once("Waiting for points")
                    continue
                # checks if we are not moving for 1 second
                if self.checkNotMoving(OCR_HZ//2, self.LR_XZ_farm):
                    rospy.loginfo("Detected wall! Moving to next row")
                    if self.keyDPressed and self.LR_ON_LOWER:
                        self.pressD(release=True)
                        self.rngDelay(0.05, 0.05)
                        self.pressS()
                        self.rngDelay(0.7, 0.9)
                        self.pressA()
                        self.rngDelay(0.03, 0.05)
                        self.pressS(release=True)
                        self.pressW()
                        self.rngDelay(0.2, 0.3)
                        self.pressW(release=True)
                        self.clearPoints()

                    elif self.keyAPressed and self.LR_ON_LOWER:
                        self.pressA(release=True)
                        self.rngDelay(0.05, 0.05)
                        self.pressS()
                        self.rngDelay(0.7, 0.9)
                        self.pressD()
                        self.rngDelay(0.03, 0.05)
                        self.pressS(release=True)
                        self.pressW()
                        self.rngDelay(0.2, 0.3)
                        self.pressW(release=True)
                        self.clearPoints()

                    elif self.keyDPressed:
                        self.pressD(release=True)
                        self.rngDelay(0.05, 0.1)
                        self.pressW()
                        self.rngDelay(0.6, 0.7)
                        self.pressA()
                        self.pressW(release=True)
                        self.clearPoints()

                    elif self.keyAPressed:
                        self.pressA(release=True)
                        self.rngDelay(0.05, 0.1)
                        self.pressW()
                        self.rngDelay(0.7, 0.9)
                        self.pressD()
                        self.pressW(release=True)
                        self.clearPoints()


            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
            

if __name__ == '__main__':
    rospy.logwarn("Initalizing movement server")
    rospy.init_node('movement_server')
    name = rospy.get_name()
    movement_server = MovementController(name)
    rospy.spin()
