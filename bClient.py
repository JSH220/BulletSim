#!/home/jshgod/anaconda3/bin/python
#-*-coding:utf-8-*-
import functools
import inspect
import pybullet

class BulletClient(object):
  def __init__(self, connection_mode=pybullet.DIRECT):
    self._client = pybullet.connect(connection_mode)

  def __del__(self):
    if self._client>=0:
      try:
        pybullet.disconnect(physicsClientId=self._client)
        self._client = -1
      except pybullet.error:
        pass

  def __getattr__(self, name):
    attribute = getattr(pybullet, name)
    if inspect.isbuiltin(attribute):
      attribute = functools.partial(attribute, physicsClientId=self._client)
    if name=="disconnect":
      self._client = -1 
    return attribute
