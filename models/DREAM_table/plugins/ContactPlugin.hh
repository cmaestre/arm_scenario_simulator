#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "ros/ros.h"
#include "std_msgs/Int8.h"

namespace gazebo
{
  // An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    // Constructor.
    public: ContactPlugin();

    // Destructor.
    public: virtual ~ContactPlugin();

    // Load the sensor plugin
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    // Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    // Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    // Connection that maintains a link between the contact sensor's updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;


    private: ros::NodeHandle rosNode;
    private: ros::Publisher rosPub;
  };
}
#endif
