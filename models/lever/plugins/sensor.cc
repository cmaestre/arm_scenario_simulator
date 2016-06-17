#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  class LeverSensor : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::PublisherPtr state_publisher;
    //private: int c;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      //this->c = 0;

      this->node = transport::NodePtr(new transport::Node());
      // Initialize the node with the world name
      this->node->Init(this->model->GetName());
      this->state_publisher = this->node->Advertise<msgs::Int>("~/is_pushed");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LeverSensor::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      gazebo::math::Vector3 angles = this->model->GetLink("lever")->GetRelativePose().rot.GetAsEuler();
      msgs::Int message;
      message.set_data(0);
      if (angles.x>0)
      {
        message.set_data(1);
      }
      this->state_publisher->Publish(message);

      /*
      this->c++;
      if(this->c>400){
        this->c = 0;
        std::cout << std::endl << angles.x << std::endl << angles.y << std::endl << angles.z << std::endl;
      }*/
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeverSensor)
}
