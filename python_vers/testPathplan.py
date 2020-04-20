# -*- coding: utf-8 -*-
import os
import inspect
from time import sleep, time
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
from dynamic_window_approach import dwa_control, Config
additional_path = pbd.getDataPath()

timeStep = 1./30.
space = 2
offsetY = space
matrix = [1,1]
obj_pos = []
config = Config()
goal = None

def setObstacles(number):
    global goal
    position = [random.randint(-500, 500)/100.0, random.randint(-500, 500)/100.0, 0]
    goal = position
    obstacle_id  = pb.createCollisionShape(pb.GEOM_CYLINDER,radius=0.2,height=0.3)
    pb.createMultiBody(baseMass=9999,baseCollisionShapeIndex=obstacle_id, basePosition=position)
    for _ in range(number):
        position = [random.randint(-500, 500)/100.0, random.randint(-500, 500)/100.0, 0]
        obj_pos.append(position[0:2])
        #obj_pos.append([9999, 9999])
        obstacle_id  = pb.createCollisionShape(pb.GEOM_CYLINDER,radius=0.05,height=0.1) #\
            # if random.random() > 0.5 else \
            # pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[0.4, 0.4, 0.5])
        pb.createMultiBody(baseMass=9999,baseCollisionShapeIndex=obstacle_id, basePosition=position)
    
    
if __name__ == "__main__":
    client = bullet_client.BulletClient(pb.GUI)
    client.setTimeStep(timeStep)
    client.setPhysicsEngineParameter(numSolverIterations=8)
    client.setPhysicsEngineParameter(minimumSolverIslandSize=100)
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,0)
    client.setAdditionalSearchPath(pbd.getDataPath())
    client.loadURDF('plane.urdf')

    setObstacles(20)
    client.setGravity(0,0,-9.8)
    robots = []

    for j in range (matrix[1]):
        offsetX = 0
        for i in range(matrix[0]):
            offset=[offsetX,offsetY,0.1]
            #sim = client.loadURDF('racecar/racecar.urdf', offset)
            sim = RacecarController(client, additional_path, timeStep, offset, [0, 0, 0])
            robots.append(sim)
            offsetX += space 
        offsetY += space 
    
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,1)
    last_ori = 0.0
    for i in range (20000):
        for robot in robots:

            state = [robot.pos[0], robot.pos[1], robot.orient[2], robot.vel, robot.yaw_rate]
            u, traj = dwa_control(state, config, goal, np.array(obj_pos))
            #print(len(traj))
            print(u)
            # u[0] = 0.8
            robot.apply_action(u)

            robot.stepSim(False, 50)
        client.stepSimulation()
        # sleep(timeStep)