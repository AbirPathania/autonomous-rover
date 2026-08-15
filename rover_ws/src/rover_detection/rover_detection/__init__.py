"""Shared helpers for the detection pipeline (ground-truth threats, TF pose)."""
import math

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class BuriedThreat:
    """Ground-truth buried object used by the sensor simulator and evaluator."""

    __slots__ = ('x', 'y', 'depth', 'ttype', 'metal', 'voc', 'size')

    def __init__(self, x, y, depth, ttype, metal, voc, size):
        self.x = float(x)
        self.y = float(y)
        self.depth = float(depth)
        self.ttype = str(ttype)
        self.metal = float(metal)   # metal/magnetic signature strength 0..1
        self.voc = float(voc)       # volatile-compound emission strength 0..1
        self.size = float(size)     # radar cross-section scale (m)


def load_threats(node):
    """Read parallel ground-truth arrays from parameters into BuriedThreat list.

    Parameters (all same length): threat_x, threat_y, threat_depth,
    threat_metal, threat_voc, threat_size (doubles) and threat_type (strings).
    """
    # Explicit-type descriptors, no default value: declaring an empty list
    # ([]) as the default is ambiguous (rclpy infers it as BYTE_ARRAY), which
    # then conflicts with the actual DOUBLE_ARRAY/STRING_ARRAY values loaded
    # from detection.yaml and throws InvalidParameterTypeException.
    double_array = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
    string_array = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
    node.declare_parameter('threat_x', descriptor=double_array)
    node.declare_parameter('threat_y', descriptor=double_array)
    node.declare_parameter('threat_depth', descriptor=double_array)
    node.declare_parameter('threat_metal', descriptor=double_array)
    node.declare_parameter('threat_voc', descriptor=double_array)
    node.declare_parameter('threat_size', descriptor=double_array)
    node.declare_parameter('threat_type', descriptor=string_array)

    xs = list(node.get_parameter('threat_x').value or [])
    ys = list(node.get_parameter('threat_y').value or [])
    ds = list(node.get_parameter('threat_depth').value or [])
    ms = list(node.get_parameter('threat_metal').value or [])
    vs = list(node.get_parameter('threat_voc').value or [])
    ss = list(node.get_parameter('threat_size').value or [])
    ts = list(node.get_parameter('threat_type').value or [])

    n = len(xs)
    threats = []
    for i in range(n):
        threats.append(BuriedThreat(
            xs[i], ys[i],
            ds[i] if i < len(ds) else 0.2,
            ts[i] if i < len(ts) else 'unknown',
            ms[i] if i < len(ms) else 0.0,
            vs[i] if i < len(vs) else 0.0,
            ss[i] if i < len(ss) else 0.1,
        ))
    return threats


def lookup_xy(tf_buffer, target_frame, source_frame, stamp, timeout_s=0.1):
    """Return (x, y) of source_frame origin in target_frame, or None."""
    import rclpy
    try:
        tf = tf_buffer.lookup_transform(
            target_frame, source_frame, stamp,
            timeout=rclpy.duration.Duration(seconds=timeout_s))
    except Exception:
        return None
    return tf.transform.translation.x, tf.transform.translation.y


def sigmoid(z):
    return 1.0 / (1.0 + math.exp(-z)) if abs(z) < 60 else (0.0 if z < 0 else 1.0)


def gaussian(dist, sigma):
    return float(np.exp(-(dist * dist) / (2.0 * sigma * sigma)))
