import os
import copy
import math
from threading import Lock

import numpy as np
from sensor_rays import BatchRay

class Racecar(object):

  def __init__(self, bullet_client, urdf_root_path, start_pos, start_ori):

    self._urdf_root_path = urdf_root_path
    self._p = bullet_client
    self._carId = 0
    self._reset(start_pos, start_ori)

  def _reset(self, start_pos, start_ori):
    car = self._p.loadURDF(os.path.join(self._urdf_root_path, "racecar/racecar_differential.urdf"),
                          start_pos,
                          self._p.getQuaternionFromEuler(start_ori),
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

  
if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from bullet_client import BulletClient

  p = BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = Racecar(p, pybullet_data.getDataPath(), [0,0,0], [0,0,0])
  while True:
    p.stepSimulation()
