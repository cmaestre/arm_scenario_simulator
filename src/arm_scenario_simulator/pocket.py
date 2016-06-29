import rospy
from .gazeboObject import GazeboObject
from std_msgs.msg import Int8

class Pocket(GazeboObject):

    def __init__(self, pocket_name):
        GazeboObject.__init__(self, pocket_name)
        self._active = None
        self._last_time_active = None
        self._off_delay = 0.1
        rospy.Subscriber("/"+pocket_name+"/contact", Int8, self.update_state)

    def update_state(self, message):
        self._active = message.data==1 or ((self._active is True)  and rospy.get_time() < self._last_time_active+self._off_delay )
        if message.data==1: self._last_time_active = rospy.get_time()

    def is_active(self):
        return self._active
