from itertools import product
from math import atan2, sqrt, sin, cos
from time import sleep
import cv2
import numpy as np
import matplotlib.pyplot as plt
import sim
import simConst
from dijkstra import Dijkstra

print('Program started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, -500000, 5)
K = [15, 1, 1]
STEP = 16


class Bot:
    def __init__(self, cID):
        self.clientID = cID
        _, self.robot = sim.simxGetObjectHandle(
            clientID, 'Bob', simConst.simx_opmode_blocking)
        _, self.left = sim.simxGetObjectHandle(
            clientID, 'Bob_leftMotor', simConst.simx_opmode_blocking)
        _, self.right = sim.simxGetObjectHandle(
            clientID, 'Bob_rightMotor', simConst.simx_opmode_blocking)

        _, map = sim.simxGetObjectHandle(
            clientID, 'Bob_mapSensor', simConst.simx_opmode_blocking)

        err = 1

        while err != 0:
            err, size, img = sim.simxGetVisionSensorImage(
                self.clientID, map, 1, simConst.simx_opmode_streaming)
        img = ~(np.array(img).reshape(size) + 1).astype(np.bool)

        compressed = np.zeros(
            [img.shape[0]//STEP, img.shape[1]//STEP], dtype=np.bool)

        for x, y in product(range(0, img.shape[0], STEP),
                            range(0, img.shape[1], STEP)):
            if np.any(img[x:x+STEP, y:y+STEP]):
                compressed[x//STEP, y//STEP] = True

        self.oy, self.ox = np.argwhere(compressed).T.tolist()
        self.planner = Dijkstra(self.ox, self.oy, 1, 2)

        self.r = 0.1
        self.l = 0.25

    def advance_straight(self, x_d, y_d, speed):
        x, y, theta = self.get_position()

        while abs(x_d - x) >= 0.01 or abs(y_d - y) >= 0.01:
            x, y, theta = self.get_position()
            x_hat = x_d - x
            y_hat = y_d - y
            theta_d = atan2(y_hat, x_hat)

            theta_hat = theta_d - theta

            w = - K[0] * theta_hat
            v = max(K[1] * sqrt(x_hat**2 + y_hat**2), speed)

            vR = (2 * v + w * self.l) / (2 * self.r)
            vL = (2 * v - w * self.l) / (2 * self.r)

            sim.simxSetJointTargetVelocity(
                clientID, self.left, vR, simConst.simx_opmode_streaming)

            sim.simxSetJointTargetVelocity(
                clientID, self.right, vL, simConst.simx_opmode_streaming)

            sleep(0.01)

    def get_position(self):
        _, pos = sim.simxGetObjectPosition(
            self.clientID, self.robot, -1, simConst.simx_opmode_oneshot)
        _, orientation = sim.simxGetObjectOrientation(
            self.clientID, self.robot, -1, simConst.simx_opmode_oneshot)
        return [*pos[:2], orientation[2]]

    def advance_path(self, gx, gy):
        pos = self.get_position()
        sx, sy = pos[:2]
        sy = int((sy + 2.5) * 30 / 5)
        sx = int((sx + 2.5) * 30 / 5)


        plt.plot(self.ox, self.oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

        rx, ry = self.planner.planning(sx, sy, gx, gy)

        plt.plot(rx, ry, "-r")
        plt.show()

        for coord in zip(rx[::-1], ry[::-1]):
            y = coord[0] * 5 / 30 - 2.5
            x = coord[1] * 5 / 30 - 2.5

            print(x, y)

            self.advance_straight(y, x, 0.5)


if clientID != -1:
    print('Connected to remote API server')

    bot = Bot(clientID)
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    while bot.get_position()[1] == 0:
        sleep(0.1)

    # here you can adjuct the target point
    # if an error occurs after changing the point, set the values back to 5 and 23
    # and run the program to see the available points on the plot
    # min and max locations are also displayed when the program is run
    target_x = 5
    target_y = 23
    bot.advance_path(target_x, target_y)

    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
