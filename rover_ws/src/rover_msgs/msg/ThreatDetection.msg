# A fused, classified threat detection emitted by the fusion node.
std_msgs/Header header
float64 x            # world x
float64 y            # world y
string type          # e.g. "metal_object" | "chemical" | "buried_object" | "unknown"
float32 confidence   # fused confidence (0..1)
float32 depth        # estimated depth (m)
