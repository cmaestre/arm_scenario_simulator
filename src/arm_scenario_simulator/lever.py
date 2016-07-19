import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8
from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

class Lever(GazeboObject):

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pushed = None
        rospy.Subscriber("/"+name+"/is_pushed", Int8, self.update_state)
        self.base_color_pub = rospy.Publisher('/'+name+'/base/visual/set_color', MaterialColor, queue_size=1)
        self.lever_color_pub = rospy.Publisher('/'+name+'/lever/visual/set_color', MaterialColor, queue_size=1)

    def spawn(self, position, orientation = None, **extra):
        return GazeboObject.spawn(self, 'DREAM_lever', position, orientation, **extra)

    def __del__(self):
        GazeboObject.__del__(self)

    def update_state(self, message):
        self._pushed = message.data==1

    def is_pushed(self):
        return self._pushed

    def set_base_color(self, r,g,b,a=None):
        self.set_color(r,g,b,a,self.base_color_pub)

    def set_lever_color(self, r,g,b,a=None):
        self.set_color(r,g,b,a,self.lever_color_pub)
