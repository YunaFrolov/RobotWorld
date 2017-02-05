# coding: utf-8

import RobotWorld
import time

DELAY = 0.1  # in seconds

myWorld = RobotWorld.World(host='127.0.0.1', portNumber=19999)
myRobotBrain = RobotWorld.RobotBrain()
while True:
    myRobotBrain.think(myWorld)
    time.sleep(DELAY)
