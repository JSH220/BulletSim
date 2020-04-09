#!/home/jshgod/anaconda3/bin/python
#-*-coding:utf-8-*-
import os
import copy
import math

import numpy as np
from batchRay import batchRay

class Racecar:

  def __init__(self, bullet_client, urdfRootPath='', timeStep = 0.01, startPos = [0, 0, .2], startOri = [0, 0, 0]):
    self.__urdfRootPath = urdfRootPath
    self.__timeStep = timeStep
    self.__p = bullet_client
    self.__carId = 0
    self.__reset(startPos)
    self.__pos = startPos
    self.__orient = self.__p.getQuaternionFromEuler(startOri)

    self.__maxForce = 20
    self.__speedMultiplier = 20.
    self.__steeringMultiplier = 0.5

    self.__maxForceUpperbound = 100
    self.__speedMultiplierUpperbound = 100
    self.__steeringMultiplierUpperbound = math.pi / 2

    # don't change them
    self.__steeringLinks = [0, 2]
    self.__nMotors = 2
    self.__motorizedwheels = [8, 15]
    self.__rayPos = [self.__pos[0], self.__pos[1], 0.3]
    self.__rays = batchRay(self.__p, self.__rayPos, 8, 512)
  
  @property
  def actionDimension(self):
    return self.__nMotors

  @property
  def orient(self):
    return self.__orient

  @property
  def pos(self):
    return self.__pos

  @property
  def maxForce(self):
    return self.__maxForce
  
  @maxForce.setter
  def maxForce(self, mf):
    assert mf > 0 and mf < self.__maxForceUpperbound, \
      'maxForce should between 0 and ' + str(self.__maxForceUpperbound)
    self.__maxForce = mf
  
  @property
  def speedMultiplier(self):
    return self.__speedMultiplier
  
  @speedMultiplier.setter
  def speedMultiplier(self, sm):
    assert sm > 0 and sm < self.__speedMultiplierUpperbound, \
      'speedMultiplier should between 0 and ' + str(self.__speedMultiplierUpperbound)
    self.__speedMultiplier = sm
  
  @property
  def steeringMultiplier(self):
    return self.__steeringMultiplier
  
  @steeringMultiplier.setter
  def steeringMultiplier(self, stm):
    assert stm > 0 and stm < self.__steeringMultiplierUpperbound, \
       'steeringMultiplier should between 0 and ' + str(self.__steeringMultiplierUpperbound)
    self.__steeringMultiplier = stm

  def __reset(self, pos = [0, 0, .2]):
    car = self.__p.loadURDF(os.path.join(self.__urdfRootPath, "racecar/racecar_differential.urdf"),
                           pos,
                           self.__p.getQuaternionFromEuler([0, 0, 0]),
                           useFixedBase=False)
    self.__carId = car

    for wheel in range(self.__p.getNumJoints(car)):
      self.__p.setJointMotorControl2(car,
                                    wheel,
                                    self.__p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0)
      self.__p.getJointInfo(car, wheel)

    c = self.__p.createConstraint(car,
                                 9,
                                 car,
                                 11,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 10,
                                 car,
                                 13,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 9,
                                 car,
                                 13,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 16,
                                 car,
                                 18,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 16,
                                 car,
                                 19,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 17,
                                 car,
                                 19,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self.__p.createConstraint(car,
                                 1,
                                 car,
                                 18,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
    c = self.__p.createConstraint(car,
                                 3,
                                 car,
                                 19,
                                 jointType=self.__p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self.__p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

  def applyAction(self, motorCommands):
    targetVelocity = motorCommands[0] * self.__speedMultiplier
    steeringAngle = motorCommands[1] * self.__steeringMultiplier

    for motor in self.__motorizedwheels:
      self.__p.setJointMotorControl2(self.__carId,
                                    motor,
                                    self.__p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocity,
                                    force=self.__maxForce)
    for steer in self.__steeringLinks:
      self.__p.setJointMotorControl2(self.__carId,
                                    steer,
                                    self.__p.POSITION_CONTROL,
                                    targetPosition=steeringAngle)
  def stepSim(self):
    self.__p.stepSimulation()
    self.__pos, self.__orient = self.__p.getBasePositionAndOrientation(self.__carId)
    self.__rays.rayPos = [self.__pos[0], self.__pos[1], 0.3]
    self.__rays.hitCheck()
    # self.__rays.drawRays()

if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from bClient import BulletClient

  p = BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = Racecar(p, pybullet_data.getDataPath())
  car.applyAction([0.7, 0.5])
  i = 0
  while True:
    i += 1
    car.stepSim()
    print("actionDimension: ", car.maxForce)
    print("pos: ", car.pos)
    print("orient: ", car.orient)
    print("maxForce: ", car.maxForce)
    print("speedMultiplier: ", car.speedMultiplier)
    print("steeringMultiplier: ", car.steeringMultiplier)
    car.maxForce = 20
    # car.maxForce = 150
    car.speedMultiplier = 80.
    # car.speedMultiplier = -50
    car.steeringMultiplier = 1.
    # car.steeringMultiplier = 3.
  p.disconnect()
