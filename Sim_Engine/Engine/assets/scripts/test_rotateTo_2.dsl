# Load Object
load(obj, Engine/assets/objects/Shapes/cube.fbx)

# Colour Pink
set(colour, {0.5,0,1})

# Rotate 90 degrees about X, max 360 deg/s
rotateTo(obj1, {x}, 90, 360)
# Rotate 90 degrees about Y, max 360 deg/s 
rotateTo(obj1, {y}, 90, 360)
# Rotate 90 degrees about Z, max 360 deg/s 
rotateTo(obj1, {z}, 90, 360)