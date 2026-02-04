# Rotation Test 3
set(integrator, rk4)
load(obj, "Engine/assets/Objects/My Model.obj")   # quotes + spaces
colour(obj, 1.0, 0.25, 0.1)

rotate({x,y,z}, 30, 0, 90)
rotate({x y}, 15, 0, 45)
rotate(obj, 10, 0, 15)
rotate(link:elbow, 20, 0, 60)
