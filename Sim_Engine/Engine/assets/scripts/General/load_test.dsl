# Basic sanity
set(integrator, rk4)

# Spawn Object
rotate({x,y,z}, 30, 0, 90)
rotate(obj, 15, 0, 45)

# Remove object (cube, etc.), spawn robotic arm
rotate(link01, 10, 0, 30)
