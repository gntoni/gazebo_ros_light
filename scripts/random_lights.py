#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 3/4/15

@author: sampfeiffer

random_lights.py contains a node that publishes random lights in gazebo
using the gazebo_ros_light plugin.
"""


# System imports
import random

# ROS imports
import rospy

# ROS messages imports
from gazebo_ros_light.msg import LightState
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA

PUB_TOPIC = '/gazebo_light_node/set_light_state'

def create_light_msg(light_name,
                     type="POINT",
                     pose=Pose(),
                     diffuse=ColorRGBA(0.5, 0.5, 0.5, 1.0),
                     specular=ColorRGBA(0.1, 0.1, 0.1, 1.0),
                     attenuation_constant=0.5,
                     attenuation_linear=0.01,
                     attenuation_quadratic=0.001,
                     direction=Vector3(),
                     range=5.0,
                     cast_shadows=True,
                     spot_inner_angle=0.0,
                     spot_outer_angle=0.0,
                     spot_falloff=0.0):
    """
    Creates a gazebo_ros_light/LightState message with the params provided.
    :param light_name: string name of the light
    :param type: string representing type of light: POINT | SPOT | DIRECTIONAL
    :param pose: geometry_msgs/Pose desired pose in the reference frame of the world
    :param diffuse: std_msgs/ColorRGBA with the diffusion color
    :param specular: std_msgs/ColorRGBA with the specular color
    :param attenuation_constant: float64 desired constant attenuation
    :param attenuation_linear: float64 desired linear attenuation
    :param attenuation_quadratic: float64 desired quadratic attenuation
    :param direction: geometry_msgs/Vector3 light direction
    :param range: float64 desired light range
    :param cast_shadows: bool if true, this light will cast model shadows
    :param spot_inner_angle: float64
    :param spot_outer_angle: float64
    :param spot_falloff: float64
    :return: gazebo_ros_light/LightState
    """
    ls = LightState()
    ls.light_name = light_name
    ls.type = type
    ls.pose = pose
    ls.diffuse = diffuse
    ls.specular = specular
    ls.attenuation_constant = attenuation_constant
    ls.attenuation_linear = attenuation_linear
    ls.attenuation_quadratic = attenuation_quadratic
    ls.direction = direction
    ls.range = range
    ls.cast_shadows = cast_shadows
    ls.spot_inner_angle = spot_inner_angle
    ls.spot_outer_angle = spot_outer_angle
    ls.spot_falloff = spot_falloff
    return ls

class RandomLights():
    """This class creates random lights using the gazebo_ros_light plugin"""

    def __init__(self):
        rospy.loginfo("Setting publisher to " + PUB_TOPIC)
        self.pub_topic = rospy.Publisher(PUB_TOPIC, LightState, queue_size=1)
        self.max_lights = 20


    def publish_random_lights(self):
        while not rospy.is_shutdown():
            for i in range(self.max_lights):
                random_light = self.create_random_light("light_" + str(i))
                self.pub_topic.publish(random_light)
                rospy.sleep(0.25)

    def create_random_light(self, light_name):
        random_pose = Pose(position=Point( random.randrange(-50, 50, 1) * 0.1,
                                           random.randrange(-50, 50, 1) * 0.1,
                                           random.randrange(1, 100, 1)  * 0.1),
                           orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        random_diffuse = ColorRGBA(random.random(),
                                 random.random(),
                                 random.random(),
                                 random.random())
        random_specular = ColorRGBA(random.random(),
                                 random.random(),
                                 random.random(),
                                 random.random())
        return create_light_msg(light_name, pose=random_pose, diffuse=random_diffuse, specular=random_specular)

if __name__ == '__main__':
    rospy.init_node('random_lights_node')
    node = RandomLights()
    node.publish_random_lights()

    