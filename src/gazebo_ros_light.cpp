#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo_ros_light.h"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(GazeboRosLight)

  // Constructor
  GazeboRosLight::GazeboRosLight() :
    nh_("gazebo_light_node")
  {
  }

  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
  }

  void GazeboRosLight::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
      //this->light_node = transport::Node();
      transport::NodePtr light_node(new transport::Node());

      // Initialize the node with the world name
      light_node->Init(_world->GetName());

      // Create a publisher on the ~/physics topic
      this->lightsPub = light_node->Advertise<msgs::Light>("~/light");


    ROS_INFO("Lights Plugin Initialized");
    this->set_light_st_subscriber_ = this->nh_.subscribe("set_light_state", 1, &GazeboRosLight::set_light_st_callback, this);
  }


  void GazeboRosLight::set_light_st_callback(const gazebo_ros_light::LightStateConstPtr& msg)
  {
    ROS_DEBUG("New light message received!");
    
    // 1. Set light name
    lightMsg.set_name(msg->light_name);

    // 2. Set light type
    if(msg->type == "POINT")
        lightMsg.set_type(gazebo::msgs::Light_LightType_POINT);
    else if(msg->type == "SPOT")
        lightMsg.set_type(gazebo::msgs::Light_LightType_SPOT);
    else if(msg->type == "DIRECTIONAL")
        lightMsg.set_type(gazebo::msgs::Light_LightType_DIRECTIONAL);

    // 3. Set camera pose
    gazebo::math::Vector3 position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    gazebo::math::Quaternion orientation(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
    gazebo::math::Pose pose(position,orientation);
    gazebo::msgs::Set(lightMsg.mutable_pose(), pose); // fill message

    // 4. Set color diffuse parameters
    gazebo::common::Color diffuse(msg->diffuse.r,msg->diffuse.g,msg->diffuse.b,msg->diffuse.a);
    gazebo::msgs::Set(lightMsg.mutable_diffuse(), diffuse); // fill message

    // 5. Set color specular parameters
    gazebo::common::Color specular(msg->specular.r,msg->specular.g,msg->specular.b,msg->specular.a);
    gazebo::msgs::Set(lightMsg.mutable_diffuse(), diffuse); // fill message

    // 6. Set attenuation constant
    lightMsg.set_attenuation_constant(msg->attenuation_constant);

    // 7. Set attenuation linear
    lightMsg.set_attenuation_linear(msg->attenuation_linear);

    // 8. Set attenuation quadratic
    lightMsg.set_attenuation_quadratic(msg->attenuation_quadratic);

    // 9. Set direction
    gazebo::math::Vector3 direction(msg->direction.x, msg->direction.y, msg->direction.z);
    gazebo::msgs::Set(lightMsg.mutable_direction(), direction);

    // 10. Set range
    lightMsg.set_range(msg->range);

    // 11. Cast shadows
    lightMsg.set_cast_shadows(msg->cast_shadows);

    // Spot light specific options
    if(msg->type == "SPOT")
    {
      // 12. Set inner_angle
      lightMsg.set_spot_inner_angle(msg->spot_inner_angle);

      // 13. Set outer_angle
      lightMsg.set_spot_outer_angle(msg->spot_outer_angle);

      // 14. Set falloff
      lightMsg.set_spot_falloff(msg->spot_falloff);
    }

    lightsPub->Publish(this->lightMsg);
  }
}
