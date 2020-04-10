import os
import math
from threading import Lock

import numpy as np
from sensor_rays import BatchRay
from racecar import Racecar

class RacecarController(Racecar):

  def __init__(self, bullet_client, urdfRootPath='', start_pos = [0, 0, .2], start_ori = [0, 0, 0]):
    assert isinstance(start_pos, (list, tuple)), \
      "Type Error: start pos should be a list or typle..."
    assert len(start_pos) == 3, \
      "Size Error: start pos should be 3 demension..."
    
    assert isinstance(start_ori, (list, tuple)), \
      "Type Error: start orientation should be a list or typle..."
    assert len(start_ori) == 3, \
      "Size Error: start orientation should be 3 demension..."
    
    super().__init__(bullet_client, urdfRootPath, start_pos, start_ori)
    
    self._lock = Lock()
    self._pos = start_pos
    self._ori = start_ori

    self._maxForceUpperbound = 100
    self._speedMultiplierUpperbound = 100
    self._steeringMultiplierUpperbound = math.pi / 2

    self._maxForce = 20
    self._speedMultiplier = 20.
    self._steeringMultiplier = 0.5

    # don't change them
    self._steering_links = [0, 2]
    self._num_motors = 2
    self._motorized_wheels = [8, 15]
    self._sensor_pos = [self._pos[0], self._pos[1], 0.3]
    self._rays = BatchRay(self._p, self._sensor_pos, 8, 512)

  @property
  def actionDimension(self):
    return self._num_motors

  @property
  def orient(self):
    return self._orient

  @property
  def pos(self):
    return self._pos

  @property
  def maxForce(self):
    return self._maxForce

  @maxForce.setter
  def maxForce(self, mf):
    assert mf > 0 and mf < self._maxForceUpperbound, \
      'maxForce should between 0 and ' + str(self._maxForceUpperbound)
    self._maxForce = mf

  @property
  def speedMultiplier(self):
    return self._speedMultiplier

  @speedMultiplier.setter
  def speedMultiplier(self, sm):
    assert sm > 0 and sm < self._speedMultiplierUpperbound, \
      'speedMultiplier should between 0 and ' + str(self._speedMultiplierUpperbound)
    self._speedMultiplier = sm

  @property
  def steeringMultiplier(self):
    return self._steeringMultiplier

  @steeringMultiplier.setter
  def steeringMultiplier(self, stm):
    assert stm > 0 and stm < self._steeringMultiplierUpperbound, \
      'steeringMultiplier should between 0 and ' + str(self._steeringMultiplierUpperbound)
    self._steeringMultiplier = stm

  @classmethod
  def getAllRayHitPos(cls):
    return cls._rayHitPos

  @classmethod
  def resetRayHitPos(cls):
    cls._rayHitPos = []

  def stepSim(self, drawRays = False, drawStep = 5, robotId = 0):
    assert isinstance(drawRays, bool), \
      "drawRays should be boolen type"
    self._p.stepSimulation()
    self._pos, self._orient = self._p.getBasePositionAndOrientation(self._carId)
    self._rays.rayPos = [self._pos[0], self._pos[1], 0.3]
    self._rays.scan_env()
    # don't support multi-robot until now
    if drawRays:
        self._rays.draw_debug(drawStep)
  def apply_action(self, motorCommands):
      targetVelocity = motorCommands[0] * self._speedMultiplier
      steeringAngle = motorCommands[1] * self._steeringMultiplier

      for motor in self._motorized_wheels:
        self._p.setJointMotorControl2(self._carId,
                                      motor,
                                      self._p.VELOCITY_CONTROL,
                                      targetVelocity=targetVelocity,
                                      force=self._maxForce)
      for steer in self._steering_links:
        self._p.setJointMotorControl2(self._carId,
                                      steer,
                                      self._p.POSITION_CONTROL,
                                      targetPosition=steeringAngle)

  
if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from bClient import BulletClient

  p = BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = RacecarController(p, pybullet_data.getDataPath())
  car.apply_action([0.7, 0.5])
  i = 0
  while True:
    i += 1
    car.stepSim(drawStep = 5)
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
