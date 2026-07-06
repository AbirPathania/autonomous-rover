# A single ground-penetrating-radar A-scan (amplitude vs depth) tagged with the
# world position of the sensor head when it was captured. A sequence of these,
# as the rover moves, forms a B-scan in which buried point objects appear as
# hyperbolae.
std_msgs/Header header
float64 x            # sensor world x (map/odom frame)
float64 y            # sensor world y
float32 dz           # depth-bin size (metres)
float32[] trace      # A-scan amplitude per depth bin (index 0 = surface)
