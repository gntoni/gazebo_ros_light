# gazebo_ros_light

The gazebo_ros_light node allows to create and control the lights in the gazebo world using ros messages.

**install**
========
After compiling using catkin_make, add the plugin by adding the line:
<plugin name="gazebo_lights_plugin" filename="libgazebo_ros_light.so"/>
to the world file (see test_lights.world as a reference).

**usage**
========
This node creates a topic called "/gazebo_light_node/set_light_state". Publishing a LightState message on this topic will create or edit existing lights in the gazebo world.

**test**
========
  roslaunch gazebo_ros_light test_lights.launch

rostopic pub /gazebo_light_node/set_light_state gazebo_ros_light/LightState "light_name: 'test_light'
type: 'POINT'
pose:
  position: {x: 0.0, y: 0.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
diffuse: {r: 0.5, g: 0.5, b: 0.5, a: 1.0}
specular: {r: 0.1, g: 0.1, b: 0.1, a: 1.0}
attenuation_constant: 0.5
attenuation_linear: 0.01
attenuation_quadratic: 0.001
direction: {x: 0.0, y: 0.0, z: 0.0}
range: 20.0
cast_shadows: true 
spot_inner_angle: 0.0
spot_outer_angle: 0.0
spot_falloff: 0.0" 
