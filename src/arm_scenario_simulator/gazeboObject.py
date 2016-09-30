import os
import rospy
import rospkg
rospack = rospkg.RosPack()

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

from arm_scenario_simulator.msg import MaterialColor
from .parameters import COLOR_TYPE

import xml.etree.ElementTree as ET

def spawn_sdf(gazebo_name, path_to_sdf, position, orientation, static):
    try:
        with open (path_to_sdf, "r") as f:
            xml=f.read().replace('\n', '')
        if static:
            root = ET.fromstring(xml) #parse the xml and retrieve the root element (sdf tag)*
            static_tag = ET.Element(tag='static')
            static_tag.text='true'
            root.find('model').append(static_tag) # find the model tag and insert a static child
            xml = ET.tostring(root) # get the new xml
            print(xml)

        GazeboObject.spawn_sdf_srv.wait_for_service()
        resp_spawn = GazeboObject.spawn_sdf_srv(gazebo_name, xml, "/", Pose(position=position, orientation=orientation), "world")
        if resp_spawn.success:
            rospy.loginfo("creation of "+ gazebo_name + " successful")
            return True
        else:
            rospy.logerr("Could not spawn "+path_to_sdf+" (from "+path_to_sdf+"), status : "+resp_spawn.status_message)
            return False
    except Exception as e:
        rospy.logerr("Could not spawn "+path_to_sdf+" (from "+path_to_sdf+")" )
        rospy.logerr(e)
        return False



class GazeboObject:
    models_paths = []
    try: models_paths = os.environ['GAZEBO_MODEL_PATH'].split(':')
    except: pass
    models_paths = [os.path.join(os.environ['HOME'],'.gazebo','models')]+models_paths
    
    spawn_sdf_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    colorable_links = ['link'] # this is the base list of colorable links in a model. Specialized classes may override this class attribute to allow the user to change links color


    def __init__(self, gazebo_name):
        self.gazebo_name = gazebo_name
        self.color_range = 255.0 #The max_limit of arguments given to set_color methods
        self.spawned = False    # says if it is an object that has been spawned by the class, or if the object is associated to a already existing model (in which case the model should not be deleted at the end of the object life)
        self.color_publishers = {}
        self.make_publishers()


    def make_publishers(self):
        for link in self.colorable_links:
            self.color_publishers[link] = rospy.Publisher('/'+self.gazebo_name+'/'+link+'/visual/set_color', MaterialColor, queue_size=1)
            
    def resolve_model_path(self, model_name):
        for direc in GazeboObject.models_paths:
            attempt = os.path.join(direc, model_name)
            if os.path.isdir(attempt): return attempt
        raise Exception('Model directory not found for '+model_name+'. Searched in the following directories ($GAZEBO_MODEL_PATH) : \n'+('\n'.join(GazeboObject.models_paths)) )


    def spawn(self, model_name, position, orientation = None, static = False):
        path_to_sdf = self.resolve_model_path(model_name)+'/model.sdf'
        if not os.path.isfile(path_to_sdf): raise Exception("The model folder does not contain any model.sdf file")
        if spawn_sdf(self.gazebo_name, path_to_sdf, position, orientation, static):
            self.spawned = True
            return self


    def delete(self):
        if not self.spawned: return
        try:
            rospy.loginfo("Deleting "+self.gazebo_name)
            GazeboObject.delete_model_srv.wait_for_service()
            resp_delete = GazeboObject.delete_model_srv(self.gazebo_name)
            self.spawned = False
        except rospy.ServiceException, e: 
            pass # Don't know why, but an exception is raised by ROS whereas the deletion is actually successful ... So ignore the exception


    def set_state(self, position, orientation=None, linear_twist=None, angular_twist=None, reference_frame="world"):
        '''GazeboObject.set_state(self, position, orientation=None, linear_twist=None, angular_twist=None, reference_frame="world")
        
        Set the state (pose + twist) of the objects according to the given reference_frame
        If an element is omitted, the current value is conserved.
        '''
        message = ModelState(self.gazebo_name, Pose(position, orientation), Twist(linear_twist, angular_twist), reference_frame)
        if not(orientation and linear_twist and angular_twist):
            current_state = self.get_state(reference_frame)
            if not orientation: message.pose.orientation = current_state.pose.orientation
            if not linear_twist: message.twist.linear = current_state.twist.linear
            if not angular_twist: message.twist.angular = current_state.twist.angular
        GazeboObject.set_model_state_srv.wait_for_service()
        resp_set = GazeboObject.set_model_state_srv(message)
        if not resp_set.success: rospy.logerr("Could not set state of "+self.gazebo_name+" , status : "+resp_set.status_message) 


    def get_state(self, reference_frame="world"):
        '''GazeboObject.get_state(self, reference_frame="world")
        
        Retrieves the state (pose + twist) of the objects according to the given reference_frame
        A pose is made of a position and orintation.
        A twist is made of a linear twist and angular twist.
        '''
        GazeboObject.get_model_state_srv.wait_for_service()
        resp_get = GazeboObject.get_model_state_srv(self.gazebo_name, reference_frame)
        if not resp_get.success: raise Exception("Could not get state of "+self.gazebo_name+" , status : "+resp_get.status_message)
        else: return resp_get


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
