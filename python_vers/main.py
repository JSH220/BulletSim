# -*- coding: utf-8 -*-
import os
import inspect
from time import sleep
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import random

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
    for j in range (matrix[1]):
        offsetX = 0
        for i in range(matrix[0]):
            offset=[offsetX,offsetY,0.5]
            #sim = client.loadURDF('racecar/racecar.urdf', offset)
            sim = RacecarController(client, additional_path, offset, [0, 0, 0])
            robots.append(sim)
            offsetX += space 
        offsetY += space 
    
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,1)

    for i in range (200):
        for robot in robots:
            if type(robot) == type(0):
                break
            robot.stepSim(False, 50)
        client.stepSimulation()
        sleep(0.01)