import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8
from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

class Button(GazeboObject):

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False
        rospy.Subscriber("/"+name+"/is_pressed", Int8, self.update_state)
        self.base_color_pub = rospy.Publisher('/'+name+'/base/visual/set_color', MaterialColor, queue_size=1)
        self.button_color_pub = rospy.Publisher('/'+name+'/button/visual/set_color', MaterialColor, queue_size=1)

    def spawn(self, position, orientation = None, **extra):
        return GazeboObject.spawn(self, 'DREAM_push_button', position, orientation, **extra)

    def __del__(self):
        GazeboObject.__del__(self)

    def update_state(self, message):
        self._pressed = message.data==1

    def is_pressed(self):
        return self._pressed

    def set_base_color(self, r,g,b,a=None):
        self.set_color(r,g,b,a,self.base_color_pub)

    def set_button_color(self, r,g,b,a=None):
        self.set_color(r,g,b,a,self.button_color_pub)
