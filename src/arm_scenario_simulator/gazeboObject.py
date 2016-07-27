import rospy
import rospkg
rospack = rospkg.RosPack()

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)
from std_msgs.msg import ColorRGBA
from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

import xml.etree.ElementTree as ET

class GazeboObject:
    models_path = rospack.get_path('arm_scenario_simulator')+'/models/'
    spawn_sdf_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    colorable_links = ['link']

    def __init__(self, gazebo_name):
        self.gazebo_name = gazebo_name
        self.color_range = 255.0 #The max_limit of arguments given to set_color methods
        self.spawned = False    # says if it is an object that has been spawned by the class, or if the object is associated to a already existing model (in which case the model should not be deleted at the end of the object life)
        self.color_publishers = {}
        self.make_publishers()

    def make_publishers(self):
        for link in self.colorable_links:
            self.color_publishers[link] = rospy.Publisher('/'+self.gazebo_name+'/'+link+'/visual/set_color', MaterialColor, queue_size=1)

    def spawn(self, type_name, position, orientation = None, static = False):
        try:
            path_to_sdf = GazeboObject.models_path + type_name +'/model.sdf'
            with open (path_to_sdf, "r") as f:
                xml=f.read().replace('\n', '')
            if static:
                root = ET.fromstring(xml) #parse the xml and retrieve the root element (sdf tag)*
                static_tag = ET.Element(tag='static')
                static_tag.text='true'
                root.find('model').append(static_tag) # find the model tag and insert a static child
                xml = ET.tostring(root) # get the new xml
                print(xml)

            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            resp_sdf = GazeboObject.spawn_sdf_srv(self.gazebo_name, xml, "/", Pose(position=position, orientation=orientation), "world")
            rospy.loginfo("creation of "+ self.gazebo_name + " ("+type_name+") successful")
            self.spawned = True
            return self
        except Exception as e:
            rospy.loginfo("Could not spawn "+ type_name)
            rospy.loginfo(e)
            return None

    def delete(self):
        if self.spawned: return
        try:
            rospy.loginfo("Deleting "+self.gazebo_name)
            resp_delete = GazeboObject.delete_model_srv(self.gazebo_name)
            self.spawned = False
        except rospy.ServiceException, e: pass # Don't know why, but an exception is raised by ROS whereas the deletion is actually successful ... So ignore the exception

    def set_pose(self, position, orientation = None):
        rospy.loginfo("GazeboObject.set_pose is not implemented yet.")
        pass

    def set_color(self, rgba, link="link", ambient_coeff=0.7, specular_coeff=0.7):
        rgba = list(rgba)
        if len(rgba) is 3: rgba +=[self.color_range]
        r,g,b,a = tuple(rgba)
        a = self.color_range if a is None else a
        message = MaterialColor()
        message.color_type = [COLOR_TYPE['diffuse'], COLOR_TYPE['ambient'], COLOR_TYPE['specular']]
        message.color.append(ColorRGBA(r/self.color_range, g/self.color_range, b/self.color_range, a/self.color_range))
        message.color.append(ColorRGBA(0.7*r/self.color_range, 0.7*g/self.color_range, 0.7*b/self.color_range, a/self.color_range))
        message.color.append(ColorRGBA(1-specular_coeff*(1-r/self.color_range), 1-specular_coeff*(1-g/self.color_range), 1-specular_coeff*(1-b/self.color_range), a/self.color_range))
        self.color_publishers[link].publish(message)

    def set_color_range(self,R):
        self.color_range = float(R)
        assert(self.color_range==R and self.color_range>0)
