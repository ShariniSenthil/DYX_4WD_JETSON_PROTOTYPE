"""Let the node lifecycle tests run where ROS 2 is not installed.

The lifecycle tests never construct a real rclpy Node: they bind real
TrajectoryGenerator methods onto a plain namespace. They only need the
node module to import. When rclpy is genuinely available (the Jetson),
nothing here runs and the real packages are used.
"""

import importlib.util
import sys
import types


_ROS_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "geographic_msgs",
    "geographic_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "mavros_msgs",
    "mavros_msgs.msg",
    "mavros_msgs.srv",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
)


class _Placeholder:
    def __init__(self, *_args, **_kwargs):
        pass


class _Node:
    def __init__(self, *_args, **_kwargs):
        pass


def _stub_module(name):
    module = types.ModuleType(name)
    module.__getattr__ = lambda _attr: _Placeholder
    return module


if importlib.util.find_spec("rclpy") is None:
    for _name in _ROS_MODULES:
        sys.modules.setdefault(_name, _stub_module(_name))
    sys.modules["rclpy.node"].Node = _Node
