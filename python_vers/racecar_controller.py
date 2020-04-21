import math
import numpy as np

from .racecar import Racecar
from .sensor_rays import BatchRay

class RacecarController(Racecar):

  def __init__(self, bullet_client, urdfRootPath, time_step, start_pos = [0, 0], start_ori = 0, goal = [0, 0, 0]):
    assert isinstance(start_pos, (list, tuple)), \
      "Type Error: start pos should be a list or typle..."
    assert len(start_pos) == 2, \
      "Size Error: start pos should be 2 demension..."
    
    assert isinstance(start_ori, (float, int)), \
      "Type Error: start orientation should be a float or int..."
    
    super().__init__(bullet_client, urdfRootPath, start_pos + [0.1,], [0, 0, start_ori])
    
    self._pos = start_pos
    self._vel = 0
    self._ori = start_ori
    self._yaw_rate = 0
    self._time_step = time_step
    
    self._goal = goal

    self._maxForceUpperbound = 100
    self._speedMultiplierUpperbound = 100
    self._steeringMultiplierUpperbound = math.pi / 2

    self._maxForce = 20
    self._speedMultiplier = 20.
    self._steeringMultiplier = 1.0

    # don't change them
    self._steering_links = [0, 2]
    self._num_motors = 2
    self._motorized_wheels = [8, 15]
    self._sensor_pos = self._pos + [0.05,]
    self._rays = BatchRay(self._p, self._sensor_pos, 8, 1024)

  @property
  def orient(self):
    return self._ori

  @property
  def time_step(self):
    return self._time_step

  @property
  def pos(self):
    return self._pos

  @property
  def goal(self):
    return self._goal

  @property
  def yaw_rate(self):
      return self._yaw_rate
  @property
  def vel(self):
      return self._vel
  
  @goal.setter
  def goal(self, goal):
    assert isinstance(goal, list), \
      "goal should be a list......"
    assert len(goal) == 2, \
      "goal should be 2 dimension......"
    self._goal = goal

  def stepSim(self, drawRays = False, drawStep = 5):
    assert isinstance(drawRays, bool), \
      "drawRays should be boolen type"
    pos, ori = self._p.getBasePositionAndOrientation(self._carId)
    self._pos, self._vel = list(pos)[0:2], np.linalg.norm((np.array(pos[0:2]) - np.array(self._pos))) / self._time_step
    ori = self._p.getEulerFromQuaternion(ori)[2]
    self._ori, self._yaw_rate = ori, (ori - self._ori) / self._time_step
    self._rays.set_sensor_pos(self._pos + [0.3,])

    hit_pos = self._rays.scan_env()
    hit_pos = [p[0:2] for p in hit_pos]

    # don't support multi-robot until now
    if drawRays:
        self._rays.draw_debug(drawStep)
    return hit_pos
        
  def apply_action(self, motorCommands):
      vel = motorCommands[0] * self._speedMultiplier
      steeringAngle = motorCommands[1] * self._steeringMultiplier

      for motor in self._motorized_wheels:
        self._p.setJointMotorControl2(self._carId,
                                      motor,
                                      self._p.VELOCITY_CONTROL,
                                      targetVelocity=vel,
                                      force=self._maxForce)
      for steer in self._steering_links:
        self._p.setJointMotorControl2(self._carId,
                                      steer,
                                      self._p.POSITION_CONTROL,
                                      targetPosition=steeringAngle)

  
if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from pybullet_utils import bullet_client
  import time
  time_step = 1./60
  p = bullet_client.BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = RacecarController(p, pybullet_data.getDataPath(), time_step)
  vel = 0.7
  angle = 0
  i = 0.7
  while True:
    i += 1
    car.apply_action([vel, angle])
    pb.setTimeStep(car.time_step)
    pb.stepSimulation()
    car.stepSim(True,drawStep = 5)
    print("pos: ", car.pos)
    print("orient: ", car.orient)

    time.sleep(time_step)
  p.disconnect()
