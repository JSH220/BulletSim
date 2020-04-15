# -*- coding: utf-8 -*-
import os
import inspect
from time import sleep
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import random
import numpy as np
import matplotlib.pyplot as plt

import pybullet as pb
import pybullet_data as pbd
from pybullet_utils import bullet_client
from racecar_controller import RacecarController
additional_path = pbd.getDataPath()

timeStep = 1./60.
space = 2
offsetY = space
matrix = [3,2]

def setObstacles(number):
    for i in range(number):
        position = [random.randint(-5, 5), random.randint(-5, 5), 0]
        obstacle_id  = pb.createCollisionShape(pb.GEOM_CYLINDER,radius=0.2,height=0.5) \
            if random.random() > 0.5 else \
            pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[0.4, 0.4, 0.5])
        pb.createMultiBody(baseMass=9999,baseCollisionShapeIndex=obstacle_id, basePosition=position)
    
def draw_collision(hit_pos):
    hit_pos = np.array(hit_pos)
    x = [val for val in hit_pos[:,0]]
    y = [val for val in hit_pos[:,1]]
    plt.scatter(x, y, s=1, alpha=0.6)
    plt.show()

if __name__ == "__main__":
    client = bullet_client.BulletClient(pb.GUI)
    client.setTimeStep(timeStep)
    client.setPhysicsEngineParameter(numSolverIterations=8)
    client.setPhysicsEngineParameter(minimumSolverIslandSize=100)
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,0)
    client.setAdditionalSearchPath(pbd.getDataPath())
    client.loadURDF('plane.urdf')
    setObstacles(10)
    client.setGravity(0,0,-9.8)
    robots = []
    x = []
    y = []
    for j in range (matrix[1]):
        offsetX = 0
        for i in range(matrix[0]):
            offset=[offsetX,offsetY,0.5]
            #robot = client.loadURDF('racecar/racecar.urdf', offset)
            robot = RacecarController(client, additional_path, timeStep, offset, [0, 0, 0])
            robots.append(robot)
            offsetX += space 
        offsetY += space
    
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,1)
    
    #position = [robots[0].pos[0], robots[0].pos[1]+2, robots[0].pos[2]]
    #obstacle_id = pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[0.4, 0.4, 0.5])
    #pb.createMultiBody(baseMass=9999,baseCollisionShapeIndex=obstacle_id, basePosition=position)
    for i in range (2000):
        for robot in robots:
            if type(robot) == type(0):
                break
            robot.apply_action([1.5, 0.5])
            hit_pos = robot.stepSim(False, 200)

            if i > 50:
                #print(len(hit_pos))
                yaw = robot.orient[2]

                hit_pos = np.array(hit_pos)
                _x = np.array([val - robot.pos[0] for val in hit_pos[:,0]])
                _y = np.array([val - robot.pos[1] for val in hit_pos[:,1]])

                x_ = _x*np.cos(yaw) - _y*np.sin(yaw)
                y_ = _x*np.sin(yaw) + _y*np.cos(yaw)
                x.extend(list(x_))
                y.extend(list(y_))

        client.stepSimulation()
        #sleep(0.01)
    print('Finish !')
    #x.extend(list(x_))
    #y.extend(list(y_))
    plt.figure(num=1, figsize=(5, 5))
    plt.xlim((-3, 3))
    plt.ylim((-3, 3))
    plt.scatter(x, y, s=1, alpha=0.6)
    plt.show()