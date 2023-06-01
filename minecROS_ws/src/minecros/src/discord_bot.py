#!/usr/bin/env python3

import os
from typing import Any
import discord
from discord.flags import Intents
from discord.ext import tasks, commands
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import io
from cv_bridge import CvBridge
import cv2
import pyautogui
from moviepy.editor import VideoFileClip
from minecros_msgs.srv import discord_message
import asyncio
# queue
import queue

VIDEO_SECONDS = 5
FPS = 30
IMAGES_TO_KEEP = VIDEO_SECONDS * FPS 


def handle_param_load(name):
    try:
        res = rospy.get_param(name)
    except rospy.ROSException as e:
        rospy.logerr(f"Unable to load param with name={name}")
        return False
    return res


rospy.init_node('discord_bot', anonymous=True)
TOKEN = handle_param_load('DISCORD_TOKEN')
GUILD = handle_param_load('DISCORD_GUILD')
print(TOKEN, GUILD)

intents=discord.Intents.default()
intents.members = True
intents.message_content = True
intents.guild_messages = True
intents.guilds = True

class ROSDiscordBot(discord.Client):
    def __init__(self, *, intents: Intents, **options: Any) -> None:
        super().__init__(intents=intents, **options)
        self.subscribed_users = []
        self.img_sub = rospy.Subscriber("/autofarm/screen_img", Image, self.image_CB, queue_size=1)
        self.coord_pub = rospy.Subscriber("/minecros/coords", Point, self.coord_CB, queue_size=1)
        self.last_imgs = []
        self.last_coords = []
        self.cv_bridge = CvBridge()
        self.init_time = rospy.Time.now()
        self.channel_to_send = None
        self.message_handler = rospy.Service('/minecros/send_discord_msg', discord_message, self.handle_incoming_message)
        self.message_queue = queue.Queue()

    def handle_incoming_message(self, req):
        print(req)
        self.message_queue.put(f"{[member.mention for member in self.subscribed_users]} {req.message_type}, I ran this from a ros message")
        return True
    
    @tasks.loop(seconds=3.0)
    async def send_message(self):
        rospy.loginfo_throttle(10, "Checking for messages to send")
        if not self.message_queue.empty():
            msg = self.message_queue.get(block=False)
            rospy.loginfo(f"Sending message: {msg}")
            await self.channel_to_send.send(msg)

    async def on_ready(self):
        self.send_message.start()
        for guild in client.guilds:
            if guild.name == GUILD:
                break

        print(
            f'{client.user} is connected to the following guild:\n'
            f'{guild.name}(id: {guild.id})\n'
        )

        members = '\n - '.join([member.name for member in guild.members])
        print(guild.members)
        print(f'Guild Members:\n - {members}')

    async def on_message(self, message):
        print(message)
        print(message.author)
        print(message.content)
        if message.author == client.user:
            return

        if message.content == '!watch':
            self.subscribed_users.append(message.author)
            self.channel_to_send = message.channel
            response = f"{message.author.mention} - Successfully subscribed to bot updates, {[member.name for member in self.subscribed_users]} have subscribed"
            
            await message.channel.send(response, reference=message)

        elif message.content == '!response':
            response = f"<@606169539197009951> of course :)"
            await message.channel.send(response)

        elif message.content == "!info" and message.author in self.subscribed_users:
            rospy
            print("sending screenshot")
            response = f'''{message.author.mention} - 
Position X: {self.last_coords[-1].x}, Y: {self.last_coords[-1].y}, {self.last_coords[-1].z}
Uptime {(rospy.Time.now() - self.init_time)} hours
Users subscribed: {[member.name for member in self.subscribed_users]}'''
            await message.channel.send(response, reference=message)

            # save image to file and then send it
            cv2.imwrite("coolest_image.png", self.last_imgs[-1])
            await message.channel.send(file=discord.File("coolest_image.png"))

        elif message.content == "!video" and message.author in self.subscribed_users:
            # write video to file and then send it
            # use all of the images in last_imgs
            print("sending video")
            response = f"{message.author.mention} - Sending video"
            await message.channel.send(response, reference=message)
            my_files = []
            # send 5 evenly spaced images
            for i in range(0, len(self.last_imgs), len(self.last_imgs) // 6):
                cv2.imwrite(f"coolest_image{i}.png", self.last_imgs[i])
                my_files.append(discord.File(f"coolest_image{i}.png"))
            await message.channel.send(files=my_files)

        elif message.content == "!help":
            response = f'''{message.author.mention} -
!watch - subscribe to bot updates (required to use !info and !video)
!info - get info about the bot
!video - get 6 images from the last 5 seconds
!response - get a response from the bot :)'''
            await message.channel.send(response, reference=message)


    def image_CB(self, msg):
        rospy.loginfo_throttle(10, "image received")
        self.last_imgs.append(cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB))

        if len(self.last_imgs) > IMAGES_TO_KEEP:
            self.last_imgs.pop(0)

    def coord_CB(self, msg):
        rospy.loginfo_coords(10, "coord received")
        self.last_coords.append(msg)
        if len(self.last_coords) > IMAGES_TO_KEEP:
            self.last_coords.pop(0)

client = ROSDiscordBot(intents=intents)
client.run(TOKEN)