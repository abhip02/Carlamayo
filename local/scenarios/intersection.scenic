# Initial conditions for the existing empty Town03 intersection.
# The YAML still selects the map, junction entry, instruction, duration and seeds.
distance = Range(20, 25)
speed = Range(3, 4)
heading = Range(-5, 5)
param distance_before_intersection_m = distance
param lateral_offset_m = Range(-0.3, 0.3)
param speed_mps = speed
param heading_offset_deg = heading

# ScenicSampler requires an ego Object. CARLAMayo uses the four params above,
# not this marker's pose, to spawn and initialize its actual ego vehicle.
ego = new Object

# At the sampled speed, allow at least six seconds of nominal approach distance.
# This is an initial-state relationship, not a guarantee about policy behavior.
require distance >= 6 * speed

# Keep the magnitude of the initial heading error below 4 degrees.
require abs(heading) <= 4
