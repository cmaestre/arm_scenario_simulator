#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <stdio.h>
#include <cstdlib>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

namespace gazebo
{
  class ChangeColor : public VisualPlugin
  {
    // Pointer to the model
    private: rendering::VisualPtr visual;
    // Listen to the update event. This event is broadcast pre-render update???

    private: ros::NodeHandle rosNode;
    //private: ros::Publisher rosPub;
    // A ROS subscriber
    private: ros::Subscriber rosSub;

    private: common::Color color;

    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/){
      this->visual = _parent;

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->visual->GetName()+"_rosnode", ros::init_options::NoSigintHandler);
      }
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      // Spin up the queue helper thread.
      boost::regex re("::");
      std::string name = regex_replace(this->visual->GetName(), re, "/");
      std::cout << name << std::endl;

      //this->rosPub = this->rosNode.advertise<std_msgs::String>(name+"/color", 1);
      this->rosSub = this->rosNode.subscribe(name+"/set_color", 1, &ChangeColor::colorCallback, this);

      this->color = common::Color(0.4, 0.4, 0.4);

      this->visual->SetAmbient(common::Color(0.7, 0.7, 0.7, 0.7));
      this->visual->SetDiffuse(common::Color(0.5, 0.5, 0.5, 0.9));
      this->visual->SetSpecular(common::Color(0.9, 0.9, 0.9, 1.0));
    }


    public: void colorCallback(const std_msgs::String::ConstPtr& msg)
    {
      std::string message = msg->data;
      //this->visual->SetVisible(false);
      boost::regex rgb("0x[[:xdigit:]]{6}");
      boost::sregex_token_iterator iter(message.begin(), message.end(), rgb, 0);
      boost::sregex_token_iterator end;

      for( ; iter != end; ++iter ){
          int r,g,b;
          std::string str(*iter);
          //std::cout << "Found this string: " << str <<'\n';

          std::stringstream sstr1,sstr2,sstr3;
          sstr1 << str.substr(2,2) << std::hex;
          sstr1 >> r;
          sstr2 << str.substr(4,2) << std::hex;
          sstr2 >> g;
          sstr3 << str.substr(6,2) << std::hex;
          sstr3 >> b;

          this->color = common::Color((float)r/255.0, (float)g/255.0, (float)b/255.0, 0.8);
          //std::cout << this->color.r << std::endl << this->color.g << std::endl << this->color.b << std::endl;

          this->visual->SetEmissive(this->color);
          break;
      }
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_VISUAL_PLUGIN(ChangeColor)
}
