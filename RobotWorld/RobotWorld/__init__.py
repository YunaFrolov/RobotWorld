# coding: utf-8

# Import Libraries:
import vrep                # V-rep library
import sys
import time                # used to keep track of time
import numpy as np         # array library
import math

# Pre-Allocation

PI = math.pi  # constant pi

# orientation of all the sensors - to know which way they face
sensor_loc = np.array(
    [-PI / 2, -50 / 180.0 * PI, -30 / 180.0 * PI, -10 / 180.0 * PI, 10 / 180.0 * PI, 30 / 180.0 * PI,
     50 / 180.0 * PI, PI / 2, PI / 2, 130 / 180.0 * PI, 150 / 180.0 * PI, 170 / 180.0 * PI, -170 / 180.0 * PI,
     -150 / 180.0 * PI, -130 / 180.0 * PI, -PI / 2])


class World(object):
    '''
    robot simulator class to communicate with the simulation environment
    '''
    clientID = 0
    sensor_h = []  # empty list for handles
    sensor_val = np.array([])  # empty array for sensor measurements

    def __init__(self, host='127.0.0.1', portNumber=19999):
        self._host = host
        self._port = portNumber
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)
        if self.clientID != -1:  # check if client connection successful
            print('Connected to remote API server')
        else:
            print('Connection not successful')
            sys.exit('Could not connect')

    def sense(self):
        '''
        implements sensor reading from the simulator
        '''

        # for loop to retrieve sensor arrays and initiate sensors
        for x in range(1, 16 + 1):
            errorCode, sensor_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_proximitySensor' + str(x),
                                                                vrep.simx_opmode_oneshot_wait)
            self.sensor_h.append(sensor_handle)  # keep list of handles
            errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                self.clientID, sensor_handle, vrep.simx_opmode_streaming)
            self.sensor_val = np.append(self.sensor_val, np.linalg.norm(detectedPoint))  # get list of values

    def act(self):
        '''
        implements action command in the robot world
        ;return: True if action was actuated
        '''
        # retrieve motor  handles
        errorCode, left_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor',
                                                                vrep.simx_opmode_oneshot_wait)
        errorCode, right_motor_handle = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor',
                                                                 vrep.simx_opmode_oneshot_wait)
        t = time.time()

        while (time.time() - t) < 180:  # run for 180 seconds
            # Loop Execution
            self.sensor_val = np.array([])
            for x in range(1, 16 + 1):
                errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
                    vrep.simxReadProximitySensor(self.clientID, self.sensor_h[x - 1], vrep.simx_opmode_buffer)
                self.sensor_val = np.append(self.sensor_val, np.linalg.norm(detectedPoint))  # get list of values

            # controller specific
            sensor_sq = self.sensor_val[1:8] * self.sensor_val[1:8]  # square the values of front-facing sensors 1-8

            # get index of the sensor that has the closest obstacle
            min_ind = np.where(sensor_sq == np.min(sensor_sq))
            min_ind = min_ind[0][0]

            # if the obstacle is as close as 0.2 meter - set the steering accordingly
            if sensor_sq[min_ind] < 0.2:
                print("obstacle detected")
                steer = -1 / sensor_loc[min_ind]
            else:
                steer = 0

            v = 1  # forward velocity
            kp = 0.5  # steering gain
            vl = v + kp * steer
            vr = v - kp * steer
            print("Velocity of left motor =", vl)
            print("Velocity of right motor =", vr)

            # steer straight or away from obstacles
            errorCode = vrep.simxSetJointTargetVelocity(self.clientID, left_motor_handle, vl, vrep.simx_opmode_streaming)
            errorCode = vrep.simxSetJointTargetVelocity(self.clientID, right_motor_handle, vr, vrep.simx_opmode_streaming)

            time.sleep(0.2)  # loop executes once every 0.2 seconds (= 5 Hz)

        # Post ALlocation
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, left_motor_handle, 0, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointTargetVelocity(self.clientID, right_motor_handle, 0, vrep.simx_opmode_streaming)
        return True

    def close(self):
        '''
        close connection
        '''
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(self.clientID)
        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)


class RobotBrain(object):

    '''
    The main intelligent controller of the simulated robot
    '''
    def __init__(self):
        pass

    def perception(self, myWorld):
        '''
        Read state and build a representation
        param: myWorld , which represents the world of the robot
        '''
        myWorld.sense()
        self.decision(myWorld)

    def decision(self, myWorld):
        '''
        The state contains the world representation
        param: myWorld , which represents the world of the robot
        '''
        myWorld.act()


    def think(self, myWorld):
        '''
        It decides which action to take given the sensor reading
        param: myWorld , which represents the world of the robot
        '''
        self.perception(myWorld)
