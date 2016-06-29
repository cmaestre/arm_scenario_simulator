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

namespace gazebo{

  class ChangeColor : public VisualPlugin {

    private: rendering::VisualPtr visual;
    private: ros::NodeHandle rosNode;
    private: ros::Subscriber rosSub;

    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/){
      this->visual = _parent;

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized()){
        std::cout << std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< "ROS init call" << std::endl<< std::endl<< std::endl<< std::endl<< std::endl<< std::endl;
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->visual->GetName()+"_rosnode", ros::init_options::NoSigintHandler);
      }

      boost::regex re("::");
      std::string name = regex_replace(this->visual->GetName(), re, "/");
      //std::cout << name << std::endl;

      this->rosSub = this->rosNode.subscribe(name+"/set_color", 1, &ChangeColor::colorCallback, this);
    }

    public: void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg){
      common::Color color(msg->r, msg->g, msg->b, msg->a);
      //std::cout << "Receiving ::" << std::endl  << color.r << std::endl << color.g << std::endl << color.b << std::endl;
      this->visual->SetEmissive(color);
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_VISUAL_PLUGIN(ChangeColor)
}
