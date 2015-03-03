# Create or edit gazebo Light 
string light_name              # name of the light
string type                    # type of light: POINT|SPOT|DIRECTIONAL
geometry_msgs/Pose pose        # desired pose in the reference frame
std_msgs/ColorRGBA diffuse     
std_msgs/ColorRGBA specular
float64 attenuation_constant   # desired constant attenuation
float64 attenuation_linear     # desired linear attenuation
float64 attenuation_quadratic  # desired quadratic attenuation
geometry_msgs/Vector3 direction # light direction
float64 range                  # desired light range
bool cast_shadows            # if true, this light will cast model shadows
float64 spot_inner_angle
float64 spot_outer_angle
float64 spot_falloff
