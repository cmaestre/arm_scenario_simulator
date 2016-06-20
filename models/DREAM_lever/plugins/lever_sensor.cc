#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "ros/ros.h"
#include "std_msgs/Int8.h"

namespace gazebo
{
  class LeverSensor : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::PublisherPtr state_publisher;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ros::NodeHandle rosNode;
    private: ros::Publisher rosPub;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Create our Gazebo node
      this->node = transport::NodePtr(new transport::Node());
      // Initialize the node with the world name
      this->node->Init(this->model->GetName());
      this->state_publisher = this->node->Advertise<msgs::Int>("~/is_pushed");
      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LeverSensor::OnUpdate, this, _1));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->model->GetName()+"_rosnode", ros::init_options::NoSigintHandler);
      }
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      // Spin up the queue helper thread.
      this->rosPub = this->rosNode.advertise<std_msgs::Int8>(this->model->GetName()+"/is_pushed", 1);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      int8_t value(0);
      gazebo::math::Vector3 angles = this->model->GetLink("lever")->GetRelativePose().rot.GetAsEuler();
      if (angles.x>0){value=1;}

      msgs::Int message; //gazebo message
      message.set_data(value);
      this->state_publisher->Publish(message);

      std_msgs::Int8 ros_message; // ros message
      ros_message.data = value;
      this->rosPub.publish(ros_message);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeverSensor)
}
