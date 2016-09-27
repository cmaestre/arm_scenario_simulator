# This is a message to specify a kind of color.
# Gazebo uses 4 kinds of color : Diffuse, Ambient, Specular and Emissive 
# http://sdformat.org/spec?elem=material
# http://blog.lexique-du-net.com/index.php?post/2009/07/24/AmbientDiffuseEmissive-and-specular-colorSome-examples

int8[] color_type
# 0: Diffuse
# 1: Ambient
# 2: Specular
# 3: Emissive

std_msgs/ColorRGBA[] color

# example. If color_type is [2, 0] and color is [color1, color2], then color1 will be considered as a specular color and color2 will be seen as a diffuse color 
