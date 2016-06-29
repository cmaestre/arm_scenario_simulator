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

class GazeboObject:
    models_path = rospack.get_path('arm_scenario_simulator')+'/models/'
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def __init__(self, gazebo_name):
        self.gazebo_name = gazebo_name
        self.color_range = 255.0 #The max_limit of arguments given to set_color methods
        self.spawned = False    # says if it is an object that has been spawned by the class, or if the object is associated to a already existing model (in which case the model should not be deleted at the end of the object life)

    def spawn(self, type_name, position, orientation = None):
        try:
            path_to_sdf = GazeboObject.models_path + type_name +'/model.sdf'
            with open (path_to_sdf, "r") as f:
                xml=f.read().replace('\n', '')
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            resp_sdf = GazeboObject.spawn_sdf(self.gazebo_name, xml, "/", Pose(position=position, orientation=orientation), "world")
            rospy.loginfo("creation of "+ self.gazebo_name + " ("+type_name+") successful")
            self.spawned = True
            return self
        except Exception as e:
            rospy.loginfo("Could not spawn "+ type_name)
            return None

    def __del__(self):
        if not self.spawned: return
        try:
            resp_delete = GazeboObject.delete_model(self.gazebo_name)
            rospy.loginfo("deletion of "+ self.gazebo_name + " successful")
            self.spawned = False
        except rospy.ServiceException, e:
            rospy.logerr("Delete Model service call failed: {0}".format(e))

    def set_pose(self, position, orientation = None):
        pass

    def set_color_range(self,R):
        self.color_range = float(R)
        assert(self.color_range==R and self.color_range>0)
