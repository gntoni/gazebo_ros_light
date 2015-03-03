/*
 * Desc: Gazebo lights plugin.
 * Author: Toni Gabas (agabas@iri.upc.edu)
 * Date: 02/03/2015
 */

#ifndef GAZEBO_ROS_LIGHT_HH
#define GAZEBO_ROS_LIGHT_HH

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/light.pb.h"
#include "gazebo/transport/transport.hh"

#include <gazebo_ros_light/LightState.h>

namespace gazebo
{

   class GazeboRosLight : public WorldPlugin
   {
      public:
        /// \brief Constructor
        GazeboRosLight();

        /// \brief Destructor
        virtual ~GazeboRosLight();

        /// \brief Load the controller
        void Load( physics::WorldPtr _world, sdf::ElementPtr _sdf );

      protected:
        /// \brief Update the controller
        //virtual void UpdateChild();
      
      private:
        //transport::NodePtr light_node;
        msgs::Light lightMsg;
        transport::PublisherPtr lightsPub;
        ros::NodeHandle nh_;
        ros::Subscriber set_light_st_subscriber_;
        void set_light_st_callback(const gazebo_ros_light::LightStateConstPtr& msg);

   };

}

#endif

