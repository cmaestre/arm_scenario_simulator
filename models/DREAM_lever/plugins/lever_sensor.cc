#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "ros/ros.h"
#include "arm_scenario_simulator/Int8Stamped.h"

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
      this->model = _parent;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->model->GetName());
      this->state_publisher = this->node->Advertise<msgs::Int>("~/is_pushed");
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LeverSensor::OnUpdate, this, _1));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      // Spin up the queue helper thread.
      this->rosPub = this->rosNode.advertise<arm_scenario_simulator::Int8Stamped>(this->model->GetName()+"/is_pushed", 1);
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

      arm_scenario_simulator::Int8Stamped ros_message; // ros message
      ros_message.header.stamp = ros::Time::now();
      ros_message.header.seq++;
      ros_message.data = value;
      this->rosPub.publish(ros_message);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeverSensor)
}
