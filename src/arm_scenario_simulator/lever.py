import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8

class Lever(GazeboObject):

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pushed = None
        rospy.Subscriber("/"+name+"/is_pushed", Int8, self.update_state)
        self.base_color_pub = rospy.Publisher('/'+name+'/base/visual/set_color', ColorRGBA, queue_size=1)
        self.lever_color_pub = rospy.Publisher('/'+name+'/lever/visual/set_color', ColorRGBA, queue_size=1)

    def spawn(self, position, orientation = None):
        return GazeboObject.spawn(self, 'DREAM_lever', position, orientation)

    def __del__(self):
        GazeboObject.__del__(self)

    def update_state(self, message):
        self._pushed = message.data==1

    def is_pushed(self):
        return self._pushed

    def set_base_color(self, r,g,b,a=None):
        if a is None: a = self.color_range
        self.base_color_pub.publish( ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range) )

    def set_lever_color(self, r,g,b,a=None):
        if a is None: a = self.color_range
        self.lever_color_pub.publish( ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range) )
