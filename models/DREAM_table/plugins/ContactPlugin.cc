#include "ContactPlugin.hh"

#include <gazebo/physics/physics.hh>
#include <boost/regex.hpp>

using namespace gazebo;

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
      gzerr << "ContactPlugin requires a ContactSensor.\n";
      return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&ContactPlugin::OnUpdate, this));
    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, this->parentSensor->GetName()+"_rosnode", ros::init_options::NoSigintHandler);
    }
    // Create our ROS node. This acts in a similar manner to the Gazebo node
    // Spin up the queue helper thread.
    boost::regex re1("::");
    boost::regex re2(this->parentSensor->GetWorldName());
    std::string topicName = regex_replace(this->parentSensor->GetScopedName(), re1, "/");
    topicName = regex_replace(topicName, re2, "");
    /*
    std::string topicName = this->parentSensor->GetTopic();
    std::cout << std::endl << std::endl << std::endl << topicName << std::endl << std::endl << std::endl << std::endl;
    */
    /*
    physics::EntityPtr link = this->parentSensor->world->GetEntity(this->parentSensor->GetParentName());
    physics::ModelPtr model = link->world->GetModel(link->GetParentName());
    std::string topicName = "/"+model->GetName()+"/"+link->GetName()+"/sensor";
    */
    this->rosPub = this->rosNode.advertise<std_msgs::Int8>(topicName, 1);
}

/// \brief Callback that receives the contact sensor's update signal.
void ContactPlugin::OnUpdate(){
  int8_t value(0);
  // Get all the contacts.
  msgs::Contacts contacts = this->parentSensor->GetContacts();
  if (contacts.contact_size()>0) {value = 1;}

  std_msgs::Int8 ros_message; // ros message
  ros_message.data = value;
  this->rosPub.publish(ros_message);
}

GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
