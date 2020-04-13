#deal with mouse events
import pybullet as pb
from enum import unique, Enum

@unique
class EventType(Enum):
    TRIGGERED = 0
    DOWN = 1
    RELEASED = 2
    NO_EVENT = 3
    

class MouseEvents(object):
    
    def __init__(self, bullet_client):
        self._bullet_client = bullet_client
        self._moving = False
        self._buttons = EventType.NO_EVENT  # 0:left;  1:middle;  2:right
        self._move_pos = ()
        self._click_pos = ()
        self._release_pos = ()
        self._is_down = False
        self._new_click_event = False
    
    def get_click_event(self):
        self.check_event()
        return self._new_click_event

    @property
    def click_pos(self):
        return self._click_pos
    
    @property
    def release_pos(self):
        return self._release_pos
    
    @property
    def move_pos(self):
        return self._move_pos
    
    @property
    def is_down(self):
        return self._is_down

    def check_event(self):
        self._new_click_event = False
        event = self._bullet_client.getMouseEvents()
        event_type = {
            1 : self._move_event,
            2 : self._button_event
        }
        if event:
            event_type[event[0][0]](event[0])

    def _move_event(self, event):
        self._moving = True
        self._move_pos = (event[1], event[2])
    
    def _button_event(self, event):
        button_event_type = {
            0 : self._left_button,
            1 : self._mid_button,
            2 : self._right_button
        }
        button_event_type[event[3]](event)

    def _left_button(self, event):
        if event[4] == 3:
            self._click_pos = (event[1], event[2])
            self._is_down = True
            self._new_click_event = True
        elif event[4] == 4:
            self._release_pos = (event[1], event[2])
            self._is_down = False
        

    def _mid_button(self, event):
        pass

    def _right_button(self, event):
        pass