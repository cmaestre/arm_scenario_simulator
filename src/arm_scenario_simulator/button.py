import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8
from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

class Button(GazeboObject):
    colorable_links = ['base','button']

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False
        rospy.Subscriber("/"+name+"/is_pressed", Int8, self.update_state)

    def spawn(self, position, orientation = None, **extra):
        return GazeboObject.spawn(self, 'DREAM_push_button', position, orientation, **extra)

    def update_state(self, message):
        self._pressed = message.data==1

    def is_pressed(self):
        return self._pressed

    def set_base_color(self, rgba):
        self.set_color(rgba,'base')

    def set_button_color(self, rgba):
        self.set_color(rgba,'button')
