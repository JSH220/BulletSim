import math
from sensor_rays import BatchRay
from racecar import Racecar

class RacecarController(Racecar):

  def __init__(self, bullet_client, urdfRootPath, time_step, start_pos = [0, 0, .2], start_ori = [0, 0, 0]):
    assert isinstance(start_pos, (list, tuple)), \
      "Type Error: start pos should be a list or typle..."
    assert len(start_pos) == 3, \
      "Size Error: start pos should be 3 demension..."
    
    assert isinstance(start_ori, (list, tuple)), \
      "Type Error: start orientation should be a list or typle..."
    assert len(start_ori) == 3, \
      "Size Error: start orientation should be 3 demension..."
    
    super().__init__(bullet_client, urdfRootPath, start_pos, start_ori)
    
    self._pos = start_pos
    self._vel = 0
    self._ori = self._p.getQuaternionFromEuler(start_ori)
    self._yaw_rate = 0
    self._last_yaw = 0
    self._time_step = time_step

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
    self._sensor_pos = [self._pos[0], self._pos[1], 0.3]
    self._rays = BatchRay(self._p, self._sensor_pos, 8, 512)

  @property
  def actionDimension(self):
    return self._num_motors

  @property
  def orient(self):
    return self._p.getEulerFromQuaternion(self._ori)

  @property
  def yaw_rate(self):
    return self._yaw_rate

  @property
  def pos(self):
    return self._pos
  
  @property
  def vel(self):
    return self._vel

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

  def stepSim(self, drawRays = False, drawStep = 5):
    assert isinstance(drawRays, bool), \
      "drawRays should be boolen type"
    self._pos, self._ori = self._p.getBasePositionAndOrientation(self._carId)
    self._rays.set_sensor_pos([self._pos[0], self._pos[1], 0.3])
    hit_pos = self._rays.scan_env()
    # don't support multi-robot until now
    if drawRays:
        self._rays.draw_debug(drawStep)
    return hit_pos
        
  def apply_action(self, motorCommands):
      self._vel = motorCommands[0] * self._speedMultiplier
      steeringAngle = motorCommands[1] * self._steeringMultiplier
      self._yaw_rate, self._last_yaw = (steeringAngle - self._last_yaw) / self._time_step, steeringAngle

      for motor in self._motorized_wheels:
        self._p.setJointMotorControl2(self._carId,
                                      motor,
                                      self._p.VELOCITY_CONTROL,
                                      targetVelocity=self._vel,
                                      force=self._maxForce)
      for steer in self._steering_links:
        self._p.setJointMotorControl2(self._carId,
                                      steer,
                                      self._p.POSITION_CONTROL,
                                      targetPosition=steeringAngle)

  
if __name__ == '__main__':
  import pybullet as pb
  import pybullet_data
  from bullet_client import BulletClient
  import time
  time_step = 1./600
  p = BulletClient(pb.GUI)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  planID = p.loadURDF('plane.urdf')
  p.setGravity(0,0,-10)
  car = RacecarController(p, pybullet_data.getDataPath(), time_step)
  vel = 0.7
  angle = 1.0
  i = 0.7
  while True:
    i += 1
    car.apply_action([vel, angle])
    if i > 200:
      angle = -1.0
    if i > 400:
      vel = 0
    pb.stepSimulation()
    car.stepSim(True,drawStep = 5)
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
    time.sleep(time_step)
  p.disconnect()
