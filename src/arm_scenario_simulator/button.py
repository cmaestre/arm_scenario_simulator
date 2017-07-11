import rospy
import time
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8
from arm_scenario_simulator.msg import MaterialColor, Int8Stamped
from .parameters import COLOR_TYPE

class Button(GazeboObject):
    colorable_links = ['base','button']

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed_before = False        
        self._pressed = False
        self._changed = False
        rospy.Subscriber("/"+name+"/is_pressed", Int8Stamped, self.update_state, queue_size=1)

    def spawn(self, position, orientation = None, **extra):
        return GazeboObject.spawn(self, 'DREAM_push_button', position, orientation, **extra)

    def update_state(self, message):        
        self._pressed_before = self._pressed
        time.sleep(0.05)
        self._pressed = message.data==1
        self._changed = self._pressed_before != self._pressed        

    def is_pressed(self):
        return self._pressed
        
    def state_changed(self):
        return self._changed
        
    def set_base_color(self, rgba):
        self.set_color(rgba,'base')

    def set_button_color(self, rgba):
        self.set_color(rgba,'button')
