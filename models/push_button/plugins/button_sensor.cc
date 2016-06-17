#include <boost/bind.hpp>
#include <stdio.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  class ButtonSensor : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::PublisherPtr pressed_publisher;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      this->node = transport::NodePtr(new transport::Node());
      // Initialize the node with the world name
      this->node->Init(this->model->GetName());
      this->pressed_publisher = this->node->Advertise<msgs::Int>("~/is_pressed");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ButtonSensor::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      gazebo::math::Vector3 p = this->model->GetLink("button")->GetRelativePose().pos;
      msgs::Int message;
      message.set_data(0);
      if (p.z<0.01)
      {
        message.set_data(1);
      }
      this->pressed_publisher->Publish(message);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ButtonSensor)
}
