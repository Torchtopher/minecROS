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

POINTS_TO_REMEMBER = 100

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

        self._as = actionlib.SimpleActionServer(self._action_name, minecros_msgs.msg.ControlMovement,
                                                execute_cb=self.movement_action_CB, auto_start=False)
        self._as.start()
        self.keyboard = Controller()
        self.mouse = MController()
        # res = handle_param_load("imu_sub_topic")

        self.point_history = []
        
        self.sub_xyz = rospy.Subscriber("/TOPIC_NOT_ADDED_YET", geometry_msgs.msg.Point, self.xyz_callback)
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
        self._as.set_preempted()

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

            elif goal.command == minecros_msgs.msg.ControlMovement.FORWARD_BACK_FARM:
                rospy.loginfo_once("Starting forward back farm")

            elif goal.command == minecros_msgs.msg.ControlMovement.WARP_GARDEN:
                rospy.loginfo_once("Starting warp garden")
            else:
                rospy.logerr(f"Unknown command {goal.command} -- Ignoring")
                self.preempt()
                return

            # publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()
            

if __name__ == '__main__':
    rospy.logwarn("Initalizing movement server")
    rospy.init_node('movement_server')
    name = rospy.get_name()
    movement_server = MovementController(name)
    rospy.spin()