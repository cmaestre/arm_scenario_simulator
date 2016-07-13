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
  class ButtonSensor : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: transport::PublisherPtr pressed_publisher;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: ros::NodeHandle rosNode;
    private: ros::Publisher rosPub;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetName());
      this->pressed_publisher = this->node->Advertise<msgs::Int>("~/is_pressed");
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ButtonSensor::OnUpdate, this, _1));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      // Spin up the queue helper thread.
      this->rosPub = this->rosNode.advertise<std_msgs::Int8>(this->model->GetName()+"/is_pressed", 1);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      int8_t value(0);
      gazebo::math::Vector3 p = this->model->GetLink("button")->GetRelativePose().pos;
      if (p.z<0.01){value=1;}

      msgs::Int message;
      message.set_data(value);
      this->pressed_publisher->Publish(message);

      std_msgs::Int8 ros_message; // ros message
      ros_message.data = value;
      this->rosPub.publish(ros_message);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ButtonSensor)
}
