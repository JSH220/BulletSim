# -*- coding: utf-8 -*-
import os
import inspect
from time import sleep
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import copy
import json
import socket
import random
import threading
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
matrix = [1,2]

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

class RobotManager():
    def __init__(self):
        self.robots_cmd = {}
        self.socket_dict = {}
        self.robots_addr = {}
        self.robots = {}
        
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
            self.robots_cmd[robot_id] = [0, 0]
            self.robots_addr[robot_id] = addr
            
            position = [random.randint(-5, 5), random.randint(-5, 5)]
            robot = RacecarController(client, additional_path, timeStep, position, 0)
            self.robots[robot_id] = robot
            data = str(addr[0])+":"+str(addr[1])
            self.send_message(data, addr)
    
        elif message_type == 'cmd':
            self.robots_cmd[robot_id] = [data['v'], data['w']]
        elif message_type == 'goal':
            self.robots[robot_id].goal = [data['x'], data['y']]
        else:
            print('Error:',message)
            
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

    x = []
    y = []
    manager = RobotManager()
    
    client.configureDebugVisualizer(client.COV_ENABLE_RENDERING,1)
    
    #position = [robots[0].pos[0], robots[0].pos[1]+2, robots[0].pos[2]]
    #obstacle_id = pb.createCollisionShape(pb.GEOM_BOX,halfExtents=[0.4, 0.4, 0.5])
    #pb.createMultiBody(baseMass=9999,baseCollisionShapeIndex=obstacle_id, basePosition=position)
    for i in range (200000):
        for robot_id in list(manager.robots.keys()):
            #print('robot:',manager.robots.keys())
            robot = manager.robots[robot_id]
            #actions = manager.robots_cmd[robot_id]
            #robot.apply_action(actions)
            hit_pos = robot.stepSim(False, 200)

            if len(hit_pos) != 0:
                #print(len(hit_pos))
                yaw = robot.orient

                hit_pos = np.array(hit_pos)
                _x = np.array([val - robot.pos[0] for val in hit_pos[:,0]])
                _y = np.array([val - robot.pos[1] for val in hit_pos[:,1]])

                x_ = _x*np.cos(yaw) - _y*np.sin(yaw)
                y_ = _x*np.sin(yaw) + _y*np.cos(yaw)
                #x.extend(list(x_))
                #y.extend(list(y_))
                manager.send_message({'x':list(x_), 'y':list(y_)}, manager.robots_addr[robot_id])

        client.stepSimulation()
        #sleep(0.01)
    print('Finish !')
    #x.extend(list(x_))
    #y.extend(list(y_))
    #plt.figure(num=1, figsize=(5, 5))
    #plt.xlim((-3, 3))
    #plt.ylim((-3, 3))
    #plt.scatter(x, y, s=1, alpha=0.6)
    #plt.show()