# -*- coding: utf-8 -*-
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import json
import socket
import random
import threading
import numpy as np
import matplotlib.pyplot as plt

import pybullet as pb
import pybullet_data as pbd
from pybullet_utils import bullet_client
from python_vers import RacecarController
additional_path = pbd.getDataPath()

timeStep = 1./30.

def setObstacles(number):
    for i in range(number):
        position = [10.0*random.random() - 5.0, 10.0*random.random() - 5.0, 0]
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

class RobotManager():
    def __init__(self):
        self.robots_cmd = {}
        self.socket_dict = {}
        self.robots_addr = {}
        self.robots = {}
        self.robots_goal = {}
        
        self.socket_dict['sim'] = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket_dict['sim'].bind(("", 10007))
        self.message_recv_thread = threading.Thread(
                target=self.message_recv, args=()
            )
        self.message_recv_thread.start()
    
    def encode_message(self, data, robot_id=0, mtype='sim', pri=5):
        data = {'Mtype':mtype, 'Pri':pri, 'Id':robot_id, 'Data':data}
        try:
            data = json.dumps(data).encode()
        except:
            print('Error', type(data['Data']), data, data['Data'])
            return "".encode()
        return data

    def send_simple_package(self, data, socket, address, debug=False):
        ret = socket.sendto(data,address)
        if debug:
            print('Simple send', data)
        return ret

    def send_message(self, data, address, mtype='sim', pri=5, debug=False):
        data = self.encode_message(data=data, robot_id=0, mtype=mtype, pri=pri)
        self.send_simple_package(data, self.socket_dict['sim'], address, debug=debug)
        
    def message_recv(self):
        while True:
            data,addr = self.socket_dict['sim'].recvfrom(65535)
            json_data = json.loads(data.decode('utf-8'))
            self.parse_message(json_data, addr)
            
    def parse_message(self, message, addr):
        message_type = message['Mtype']
        #pri = message['Pri']
        robot_id = message['Id']
        data = message['Data']
        if message_type == 'register':
            if robot_id in self.robots.keys():
                print('re-register robot', robot_id)
            else:
                print('register robot', robot_id)
            self.robots_cmd[robot_id] = [0., 0.]
            self.robots_addr[robot_id] = addr
            self.robots_goal[robot_id] = [0., 0.]
            
            position = [10.0*random.random() - 5.0, 10.0*random.random() - 5.0]
            robot = RacecarController(client, additional_path, timeStep, position, 0)
            self.robots[robot_id] = robot
            data = str(addr[0])+":"+str(addr[1])
            self.send_message(data, addr)
    
        elif message_type == 'cmd':
            self.robots_cmd[robot_id] = [data['v'], data['w']]
        elif message_type == 'goal':
            self.robots_goal[robot_id] = [data['x'], data['y']]
            draw_goals(self.robots_goal)
        else:
            print('Error:',message)
     
def draw_goals(goals):
    global client
    client.removeAllUserDebugItems()
    for robot_id, goal in goals.items():
        client.addUserDebugLine([goal[0], goal[1], 0.],
                                [goal[0], goal[1], 1.],
                                lineColorRGB = [1., 0., 0.],
                                lineWidth = 10.0)
        client.addUserDebugText("Goal",
                                [goal[0], goal[1], 1.],
                                textColorRGB = [1., 0., 0.],
                                textSize = 1.5)
        
if __name__ == "__main__":
    client = bullet_client.BulletClient(pb.GUI)
    
    textureId = -1
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,0)
    terrainShape = pb.createCollisionShape(shapeType = pb.GEOM_HEIGHTFIELD, meshScale=[0.05,0.05,16],fileName = "map.png")
    textureId = pb.loadTexture("map.png")
    terrain  = pb.createMultiBody(0, terrainShape)
    pb.changeVisualShape(terrain, -1, textureUniqueId = textureId)
    pb.changeVisualShape(terrain, -1, rgbaColor=[1,1,1,1])
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING,1)

    client.setTimeStep(timeStep)
    client.setPhysicsEngineParameter(numSolverIterations=8)
    client.setPhysicsEngineParameter(minimumSolverIslandSize=100)
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,1)
    client.setAdditionalSearchPath(pbd.getDataPath())
    client.loadURDF('plane.urdf')
    #setObstacles(5)
    client.setGravity(0,0,-9.8)

    x = []
    y = []
    manager = RobotManager()

    while True:
        for robot_id in list(manager.robots.keys()):
            robot = manager.robots[robot_id]
            actions = manager.robots_cmd[robot_id]
            robot.apply_action(actions)
            hit_pos = robot.stepSim(False, 200)
            if len(hit_pos) != 0:
                manager.send_message({
                        'goal': manager.robots_goal[robot_id],
                        'hit_pos': hit_pos,
                        'pos': manager.robots[robot_id].pos,
                        'ori': manager.robots[robot_id].orient,
                        'vel': manager.robots[robot_id].vel,
                        'yaw_rate':manager.robots[robot_id].yaw_rate
                        }, manager.robots_addr[robot_id])
        client.stepSimulation()
    print('Finish !')