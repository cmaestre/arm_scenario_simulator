import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA, Int8

class Button(GazeboObject):

    def __init__(self, name):
        GazeboObject.__init__(self, name)
        self._pressed = False
        rospy.Subscriber("/"+name+"/is_pressed", Int8, self.update_state)
        self.base_color_pub = rospy.Publisher('/'+name+'/base/visual/set_color', ColorRGBA, queue_size=1)
        self.button_color_pub = rospy.Publisher('/'+name+'/button/visual/set_color', ColorRGBA, queue_size=1)

    def spawn(self, position, orientation = None):
        return GazeboObject.spawn(self, 'DREAM_push_button', position, orientation)

    def __del__(self):
        GazeboObject.__del__(self)

    def update_state(self, message):
        self._pressed = message.data==1

    def is_pressed(self):
        return self._pressed

    def set_base_color(self, r,g,b,a=None):
        if a is None: a = self.color_range
        self.base_color_pub.publish( ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range) )

    def set_button_color(self, r,g,b,a=None):
        if a is None: a = self.color_range
        self.button_color_pub.publish( ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range) )
