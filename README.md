# gazebo_ros_light

The gazebo_ros_light node allows to create and control the lights in the gazebo world using ros messages.

**usage**
This node creates a topic called "/gazebo_light_node/set_light_state". Publishing a LightState message on this topic will create or edit existing lights in the gazebo world.
