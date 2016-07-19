#include <boost/bind.hpp>
#include <stdio.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "arm_scenario_simulator/MaterialColor.h"

namespace gazebo
{
  class ChangeColor : public VisualPlugin
  {
    private: rendering::VisualPtr visual;
    private: ros::NodeHandle* rosNode;
    private: ros::Subscriber rosSub;

    // Destructor
    ~ChangeColor()
    {
      delete this->rosNode;
    }

    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/){
      this->visual = _parent;

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      boost::regex re("::");
      std::string name = regex_replace(this->visual->GetName(), re, "/");
      //std::cout << "name : " << name << std::endl;
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      this->rosNode = new ros::NodeHandle(name);
      this->rosSub = this->rosNode->subscribe("set_color", 1, &ChangeColor::colorCallback, this);
    }

    public: void colorCallback(const arm_scenario_simulator::MaterialColor::ConstPtr& msg){
      if (msg->color_type.size()!= msg->color.size()){
        throw "Unconsistent fields size in MaterialColor message";
      }
      int type;
      std_msgs::ColorRGBA rgba;
      common::Color color;
      for (unsigned int i=0; i<msg->color_type.size(); i++) {
        rgba = msg->color[i];
        type = msg->color_type[i];
        color = common::Color(rgba.r, rgba.g, rgba.b, rgba.a);
        //std::cout << "Receiving ::" << std::endl  << color.r << std::endl << color.g << std::endl << color.b << std::endl;
        switch(type){
          case 0: this->visual->SetDiffuse(color); break;
          case 1: this->visual->SetAmbient(color); break;
          case 2: this->visual->SetSpecular(color); break;
          case 3: this->visual->SetEmissive(color);  break;
          default : std::cout << "Unknown color_type index : " << type << std::endl; break;
        }
      }
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_VISUAL_PLUGIN(ChangeColor)
}
