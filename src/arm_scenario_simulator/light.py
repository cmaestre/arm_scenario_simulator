import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA

class Light(GazeboObject):
    types = {'square':'DREAM_light_square', 'round':'DREAM_light_round'}

    def __init__(self, name, color):
        GazeboObject.__init__(self, name)
        self.color = color
        self._on = False
        self.publisher = rospy.Publisher('/'+name+'/lamp/visual/set_color', ColorRGBA, queue_size=1)
        self.set_color(color['r'], color['g'], color['b'], color['a'] if 'a' in color.keys() else None)

    def spawn(self, shape, position, orientation = None):
        return GazeboObject.spawn(self, Light.types[shape], position,orientation)

    def __del__(self):
        GazeboObject.__del__(self)

    def set_color(self, r,g,b,a=None):
        if a is None: a = self.color_range
        self.color = ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range)
        if self._on: self.set_on(True)

    def turn_on(self, force = False):
        if self._on and not force: return
        self.set_on(True)

    def turn_off(self, force = False):
        if not self._on and not force: return
        self.set_on(False)

    def set_on(self, on):
        self.publisher.publish(self.color if on else ColorRGBA(0.0,0.0,0.0,1))
        self._on = on