import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import ColorRGBA
from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

class Light(GazeboObject):
    types = {'square':'DREAM_light_square', 'round':'DREAM_light_round'}
    off_color = ColorRGBA(0.0,0.0,0.0,1)

    def __init__(self, name, color=None):
        GazeboObject.__init__(self, name)
        self.publisher = rospy.Publisher('/'+name+'/lamp/visual/set_color', MaterialColor, queue_size=1)
        self.color = None
        self._on = False
        if color: self.set_color(color)
        self._send_color_cmd(Light.off_color) # This is of course useful only if the python object is being asociated with an already spawend model. Otherwise, it just does nothing

    def spawn(self, shape, position, orientation = None, **extra):
        return GazeboObject.spawn(self, Light.types[shape], position,orientation, **extra)

    def set_color(self, color):
        if len(color) is 3: color += [self.color_range]
        previous_color = self.color
        self.color = ColorRGBA(color[0]/self.color_range, color[1]/self.color_range, color[2]/self.color_range, color[3]/self.color_range)
        if self.color != previous_color and self._on : self._send_color_cmd(self.color)

    def turn_on(self, force = False):
        self.set_state(True, force = force)

    def turn_off(self, force = False):
        self.set_state(False, force = force)

    def set_on(self, on):
        rospy.loginfo("DEPRECATED : recommended to use set_state instead")
        self.set_state(on)

    def set_state(self, on, color=None, force = False):
        if color is not None: self.set_color(color)
        if self._on!=on or force:
            color = self.color if on else Light.off_color
            if on and color is None : raise Exception('You must set the color before turning on the light')
            self._send_color_cmd(color)
            self._on = on

    def _send_color_cmd(self, color):
        message = MaterialColor()
        message.color_type.append(COLOR_TYPE['emissive'])
        message.color.append(color)
        self.publisher.publish(message)
