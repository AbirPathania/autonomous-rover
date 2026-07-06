# A processed anomaly reading from one buried-threat sensor, localised to a world
# position. Produced by the per-sensor processing stages and consumed by the
# fusion node.
std_msgs/Header header
string sensor        # "gpr" | "metal" | "voc"
float64 x            # world x of the anomaly / sensor
float64 y            # world y
float32 value        # normalised anomaly strength (0..1)
float32 depth        # estimated depth (m); 0 if not applicable
