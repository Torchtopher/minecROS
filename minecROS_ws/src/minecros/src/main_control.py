#!/usr/bin/env python3

import rospy
import actionlib
import minecros_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from minecros_msgs.srv import discord_message, discord_messageRequest, discord_messageResponse, MainControl, MainControlRequest, MainControlResponse
import enum
import os
import subprocess
import time
# communicates with the discord and movement node to control what happens
# reponsible for all the checks to not get banned
# read ocr coords and verify we have not been teleported and are still moving
# assert angle is within tolerance
# decide when to take breaks
# make sure crops are being broken

MAX_COORDS_TO_KEEP = 100
MAX_ANGLES_TO_KEEP = 100
ALLOWED_MOVEMENT_THRESHOLD = 3 # if we move more than this on X or Z, big problems and time to disconnect
REQUIRED_ANGLE = (0, 0)
ANGLE_THRESHOLD = 4 # degrees

DISCORD_DC_MSG = "Requested from discord"

class MainState(enum.Enum):
    DISCONNECTED = 0
    CONNECTED = 1


class PrimaryController:

    def __init__(self, name):
        rospy.logwarn("Initalizing primary controller")
        self.coord_sub = rospy.Subscriber("/minecros/coords", geometry_msgs.msg.Point, self.xyz_callback)
        self.angle_sub = rospy.Subscriber("/minecros/angle", geometry_msgs.msg.Point, self.angle_callback)
        self.coord_history = []
        self.angle_history = []
        self.movement_node_client = actionlib.SimpleActionClient('movement_server', minecros_msgs.msg.ControlMovementAction)
        self.movement_node_client.wait_for_server()
        self._action_name = name

        self._as = actionlib.SimpleActionServer(self._action_name, minecros_msgs.msg.ControlMovementAction,
                                                execute_cb=self.primary_control_CB, auto_start=False)
        self._as.start()

        rospy.wait_for_service('/minecros/send_discord_msg')
        self.discord_srv = rospy.ServiceProxy('/minecros/send_discord_msg', discord_message)
        self.discord_srv_server = rospy.Service('/minecros/main_ctrl', MainControl, self.handle_commands)
        self.state = MainState.CONNECTED

    def my_done_cb(self, goal_status, goal_result): 
        rospy.logerr("Done callback called in main controller!")
        print(goal_status)
        print(goal_result)
        if not goal_result.success:
            self.disconnect("Preempt from within movement_node")

    def primary_control_CB(self, req_goal):
        rospy.loginfo(f"Primary control server called with goal {req_goal}!")
        self.state = MainState.CONNECTED
        r = rospy.Rate(100)

        start_time = rospy.Time.now()
        goal = minecros_msgs.msg.ControlMovementAction()

        # pass through inital arguments
        goal.action_goal.goal.command = req_goal.command
        goal.action_goal.goal.starting_direction = req_goal.starting_direction
        goal.action_goal.goal.starting_level = req_goal.starting_level
        self.movement_node_client.send_goal(goal.action_goal.goal, done_cb=self.my_done_cb)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.preempt()
                return
            

    def preempt(self):
        self.movement_node_client.cancel_all_goals()
        self.disconnect("Main node preempted - this is very bad")

    def handle_shell(self, command):
        cmd_list = command.split(" ")
        # call command
        out = subprocess.check_output(cmd_list)
        rospy.loginfo(f"Subprocess returned {out}")
        msg = discord_messageRequest()
        msg.message_type = discord_messageRequest.GENERICMESSAGE
        msg.str_to_send = out
        res = self.discord_srv(msg)
        return

    def resume_farming(self, timer_info):
        if self.state != MainState.CONNECTED:
            rospy.logerr("Was going to continue farming but state was no longer connected")
            msg = discord_messageRequest()
            msg.message_type = discord_messageRequest.GENERICMESSAGE
            msg.str_to_send = "Wanted to continue farming from break, but state was disconnected in main control node"
            res = self.discord_srv(msg)
        else:
            rospy.logwarn("Resuming farming after break!")
            goal = minecros_msgs.msg.ControlMovementAction()
            goal.action_goal.goal.starting_direction = minecros_msgs.msg.ControlMovementGoal.RESUME
            self.movement_node_client.send_goal(goal.action_goal.goal, done_cb=self.my_done_cb)
            msg = discord_messageRequest()
            msg.message_type = discord_messageRequest.GENERICMESSAGE
            msg.str_to_send = "The bot thanks you for its generous break and will now continue farming!"
            res = self.discord_srv(msg)

    def handle_commands(self, req):
        if req.command == MainControlRequest.DISCONNECT:
            self.disconnect(DISCORD_DC_MSG)
        elif req.command == MainControlRequest.START_OR_RESUME_FARMING:
            goal = minecros_msgs.msg.ControlMovementAction()
            goal.action_goal.goal.starting_direction = minecros_msgs.msg.ControlMovementGoal.RESUME
            self.movement_node_client.send_goal(goal.action_goal.goal, done_cb=self.my_done_cb)
        elif req.command == MainControlRequest.BREAK:
            rospy.logwarn(f"Taking a {req.minutes_to_break} minute break!")
            rospy.Timer(rospy.Duration(secs=60*req.minutes_to_break), self.resume_farming, oneshot=True)
            self.movement_node_client.cancel_all_goals()

        elif req.command == MainControlRequest.SHELL:
            self.handle_shell(req.shell_command)
        else:
            rospy.logerr(f"In main control node, {req} contains a command that is unknown")

        return MainControlResponse()

    def disconnect(self, error_msg):
        rospy.logerr(f"Disconnecting, given reason {error_msg}")
        # call disconnect in movment node
        goal = minecros_msgs.msg.ControlMovementAction()
        print(goal)
        print(goal.action_goal.goal.command)
        goal.action_goal.goal.command = minecros_msgs.msg.ControlMovementGoal.LOGOFF
        self.movement_node_client.cancel_all_goals()
        # Sends the goal to the action server.
        self.movement_node_client.send_goal(goal.action_goal.goal)
        # Waits for the server to finish performing the action.
        #self.movement_node_client.wait_for_result()
        rospy.loginfo("Disconnect should be successful")
        # call discord bot here
        msg = discord_messageRequest()
        msg.message_type = 0
        msg.str_to_send = error_msg
        res = self.discord_srv(msg)
        if res:
            rospy.loginfo("Discord service call successful")
        else:
            rospy.logerr("Discord service call failed")
        self.state = MainState.DISCONNECTED

    # check X and Z which should not move t
    def xyz_callback(self, msg):
        self.coord_history.append(msg)
        if len(self.coord_history) > 2:
            most_recent = self.coord_history[-2]
        else:
            return
        if abs(msg.x - most_recent.x) > ALLOWED_MOVEMENT_THRESHOLD or abs(msg.z - most_recent.z) > ALLOWED_MOVEMENT_THRESHOLD:
            rospy.logwarn(f"Current X {msg.x} Z {msg.z} Past X {most_recent.x} Y {most_recent.z}")
            if self.state == MainState.CONNECTED:
                self.disconnect("Coord jump detected")

        if len(self.coord_history) > MAX_COORDS_TO_KEEP:
            self.coord_history.pop(0)

    def angle_callback(self, msg):
        if abs(msg.x) > 0.1 or abs(msg.y) > ANGLE_THRESHOLD:
            if self.state == MainState.CONNECTED:
                self.disconnect("Angle not within tolerance")


if __name__ == '__main__':
    rospy.logwarn("Initalizing main control node")
    rospy.init_node('main_control_node')
    name = rospy.get_name()
    controller = PrimaryController(name)
    rospy.spin()