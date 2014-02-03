# The Message type for a Cuboid in 3-dimensional space
# Since the width / height / depth for a object are not determined uniquely
# in suturo_perception_match_cuboid, the dimensions of the cuboid will
# just be referenced as length1,2 and 3.
float32 length1
float32 length2
float32 length3
# The pose of the Cuboid 
geometry_msgs/Pose pose
# The volume of this cuboid (e.g. length1 * length2 * length3)
float32 volume
