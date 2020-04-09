import os
import copy
import math

import numpy as np


class Racecar:

  def __init__(self, bullet_client, urdfRootPath='', timeStep = 0.01, startPos = [0, 0, .2]):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self._p = bullet_client
    self._carId = 0
    self._reset(startPos)
    self._pos = startPos
    self._orient = self._p.getQuaternionFromEuler([0, 0, 0])
  
  @property
  def actionDimension(self):
    return self.nMotors

  @property
  def observationDimension(self):
    return len(self._pos) + len(self._orient)

  @property
  def pos(self):
    return self._pos

  # @pos.setter
  # def pos(self, pos_set):
  #   assert isinstance(pos_set, list)
  #   assert len(pos_set) == 3
  #   self._pos = pos_set
  #   self._reset(pos_set)

  def _reset(self, pos = [0, 0, .2]):
    car = self._p.loadURDF(os.path.join(self.urdfRootPath, "racecar/racecar_differential.urdf"),
                           pos,
                           self._p.getQuaternionFromEuler([0, 0, 0]),
                           useFixedBase=False)
    self._carId = car

    for wheel in range(self._p.getNumJoints(car)):
      self._p.setJointMotorControl2(car,
                                    wheel,
                                    self._p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0)
      self._p.getJointInfo(car, wheel)

    c = self._p.createConstraint(car,
                                 9,
                                 car,
                                 11,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 10,
                                 car,
                                 13,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 9,
                                 car,
                                 13,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 16,
                                 car,
                                 18,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 16,
                                 car,
                                 19,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 17,
                                 car,
                                 19,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, maxForce=10000)

    c = self._p.createConstraint(car,
                                 1,
                                 car,
                                 18,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
    c = self._p.createConstraint(car,
                                 3,
                                 car,
                                 19,
                                 jointType=self._p.JOINT_GEAR,
                                 jointAxis=[0, 1, 0],
                                 parentFramePosition=[0, 0, 0],
                                 childFramePosition=[0, 0, 0])
    self._p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

    self.steeringLinks = [0, 2]
    self.maxForce = 20
    self.nMotors = 2
    self.motorizedwheels = [8, 15]
    self.speedMultiplier = 20.
    self.steeringMultiplier = 0.5

  def applyAction(self, motorCommands):
    targetVelocity = motorCommands[0] * self.speedMultiplier

    steeringAngle = motorCommands[1] * self.steeringMultiplier

    for motor in self.motorizedwheels:
      self._p.setJointMotorControl2(self._carId,
                                    motor,
                                    self._p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocity,
                                    force=self.maxForce)
    for steer in self.steeringLinks:
      self._p.setJointMotorControl2(self._carId,
                                    steer,
                                    self._p.POSITION_CONTROL,
                                    targetPosition=steeringAngle)
  def stepSim(self):
    self._p.stepSimulation()
    self._pos, self._orient = self._p.getBasePositionAndOrientation(self._carId)

if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from pybullet_utils import bullet_client

  p = bullet_client.BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = Racecar(p, pybullet_data.getDataPath())
  car.applyAction([0.1, 0.5])
  i = 0
  while True:
    i += 1
    car.stepSim()
    print(isinstance(car.pos, tuple))
    # if i == 1000:
    #   car.pos = [0, 3, .2]
    #   car.applyAction([0.1, 0.5])
  p.disconnect()
