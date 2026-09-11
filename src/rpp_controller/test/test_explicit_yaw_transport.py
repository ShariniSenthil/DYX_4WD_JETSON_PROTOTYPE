"""Patch-3 RPP yaw propagation tests; bridge remains Patch-2 fail-closed."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[3]
RPP = ROOT / "src/rpp_controller/rpp_controller/rpp_controller_node.py"
BRIDGE = ROOT / "src/jetson_4wd_control/jetson_4wd_control/cmd_vel_bridge.py"
LAUNCH = ROOT / "src/rover_bringup/launch/rover.launch.py"
MODE = "rpp_explicit_yaw_enabled"
TOPICS = {False: "/rpp/velocity_ned", True: "/rpp/command"}


def class_ast(path, name):
    return next(n for n in ast.parse(path.read_text()).body
                if isinstance(n, ast.ClassDef) and n.name == name)


def method(path, owner, name):
    return next(n for n in class_ast(path, owner).body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def execute(nodes, namespace):
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    exec(compile(module, "<transport source>", "exec"), namespace)


def mode_setup(path, owner):
    return [n for n in method(path, owner, "__init__").body
            if isinstance(n, (ast.Expr, ast.Assign)) and MODE in ast.unparse(n)]


class Message:
    def __init__(self):
        self.header = NS(stamp=None, frame_id="")
        self.vector = NS(x=0.0, y=0.0, z=0.0)
        self.velocity = NS(x=0.0, y=0.0, z=0.0)
        self.position = NS(x=0.0, y=0.0, z=0.0)
        self.acceleration_or_force = NS(x=0.0, y=0.0, z=0.0)


class Vector(Message):
    pass


class Atomic(Message):
    pass


class Target(Message):
    FRAME_LOCAL_NED = 1
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048


class Time:
    nanoseconds = 0

    def __sub__(self, other):
        return NS(nanoseconds=self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return self.nanoseconds


class Publisher:
    def __init__(self, kind, topic):
        self.kind, self.topic, self.messages = kind, topic, []

    def publish(self, message):
        assert isinstance(message, self.kind)
        self.messages.append(message)


class Node:
    overrides = {}

    def __init__(self, name):
        self.parameters, self.descriptors = {}, {}
        self.publishers, self.subscriptions, self.logs = [], [], []
        self.time = Time()

    def declare_parameter(self, name, default, descriptor=None):
        self.parameters[name] = self.overrides.get(name, default)
        self.descriptors[name] = descriptor

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_publisher(self, kind, topic, qos):
        pub = Publisher(kind, topic)
        self.publishers.append(pub)
        return pub

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append(
            NS(kind=kind, topic=topic, callback=callback, qos=qos)
        )

    def create_timer(self, period, callback):
        self.period = period
        return callback

    def get_clock(self):
        return NS(now=lambda: self.time)

    def get_logger(self):
        return NS(warn=self.logs.append, info=self.logs.append, error=self.logs.append)


def namespace():
    policy = NS(KEEP_LAST=1, BEST_EFFORT=2, RELIABLE=3,
                VOLATILE=4, TRANSIENT_LOCAL=5)
    return dict(Node=Node, math=math, Any=object, Vector3Stamped=Vector,
                RppCommand=Atomic, PositionTarget=Target, State=Message,
                Bool=Message, UInt64=Message, ParameterDescriptor=NS,
                QoSProfile=NS, HistoryPolicy=policy, ReliabilityPolicy=policy,
                DurabilityPolicy=policy)


def bridge(enabled=None):
    Node.overrides = {} if enabled is None else {MODE: enabled}
    env = namespace()
    execute([class_ast(BRIDGE, "CmdVelBridge")], env)
    return env["CmdVelBridge"]()


def rpp(enabled=None):
    Node.overrides = {} if enabled is None else {MODE: enabled}
    node = Node("rpp_controller")
    env = namespace()
    env.update(self=node, command_qos=NS())
    execute([class_ast(RPP, "_ExplicitYawStagingPublisher")], env)
    execute(mode_setup(RPP, "RPPController"), env)
    selection = [n for n in method(RPP, "RPPController", "__init__").body
                 if isinstance(n, ast.If)
                 and ast.unparse(n.test) == "self.rpp_explicit_yaw_enabled"]
    assert len(selection) == 1
    execute(selection, env)
    return node, env


def test_launch_has_one_false_authority_passed_identically_to_both_nodes():
    tree = ast.parse(LAUNCH.read_text())
    assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name)
                           and t.id == "RPP_EXPLICIT_YAW_ENABLED" for t in n.targets)]
    assert len(assignments) == 1
    assert ast.literal_eval(assignments[0].value) is False
    wired = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        if call.func.id != "Node":
            continue
        kwargs = {k.arg: k.value for k in call.keywords}
        if "parameters" not in kwargs:
            continue
        for item in ast.walk(kwargs["parameters"]):
            if isinstance(item, ast.Dict):
                for key, value in zip(item.keys, item.values):
                    if isinstance(key, ast.Constant) and key.value == MODE:
                        assert isinstance(value, ast.Name)
                        assert value.id == "RPP_EXPLICIT_YAW_ENABLED"
                        wired.append(ast.literal_eval(kwargs["name"]))
    assert sorted(wired) == ["cmd_vel_bridge", "rpp_controller"]


def test_node_defaults_are_false_and_parameter_is_read_only():
    for node in (rpp()[0], bridge()):
        assert node.get_parameter(MODE).value is False
        assert node.rpp_explicit_yaw_enabled is False
        assert node.descriptors[MODE].read_only is True


@pytest.mark.parametrize("enabled", [False, True])
def test_exactly_one_command_publisher_and_subscription(enabled):
    controller, _ = rpp(enabled)
    consumer = bridge(enabled)
    pubs = [p for p in controller.publishers if p.topic in TOPICS.values()]
    subs = [s for s in consumer.subscriptions if s.topic in TOPICS.values()]
    expected_type = Atomic if enabled else Vector
    assert [(p.topic, p.kind) for p in pubs] == [(TOPICS[enabled], expected_type)]
    assert [(s.topic, s.kind) for s in subs] == [(TOPICS[enabled], expected_type)]
    policies = namespace()
    reliability = policies["ReliabilityPolicy"]
    assert subs[0].qos.history == policies["HistoryPolicy"].KEEP_LAST
    assert subs[0].qos.depth == 1
    assert subs[0].qos.reliability == (
        reliability.RELIABLE if enabled else reliability.BEST_EFFORT
    )
    assert subs[0].qos.durability == policies["DurabilityPolicy"].VOLATILE
    label = "B / EXPLICIT_YAW" if enabled else "A / LEGACY_VELOCITY"
    for node in (controller, consumer):
        assert any(label in line for line in node.logs)
        if enabled:
            assert any("actuation disabled" in line for line in node.logs)
    # Catch additional command publisher/subscription sites outside selection.
    for path, operation in ((RPP, "create_publisher"), (BRIDGE, "create_subscription")):
        sites = [n for n in ast.walk(ast.parse(path.read_text()))
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                 and n.func.attr == operation and len(n.args) > 1
                 and isinstance(n.args[1], ast.Constant)
                 and n.args[1].value in TOPICS.values()]
        assert len(sites) == 2


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("precision", [False, True])
def test_both_existing_rpp_publication_paths_use_selected_transport(enabled, precision):
    node, env = rpp(enabled)
    node.MAXIMUM_MOVING_SPEED_MPS = node.cruise_speed = 1.0
    node.precision_minimum_moving_speed = 0.04
    node.precision_speed_config = NS(hardware_speed_ceiling_mps=1.0)
    node.acceleration_speed_limit = node.command_speed_slew_limit = lambda v: v
    node.reset_deceleration_profile = lambda: None
    node.publish_motion_profile_monitor = lambda v: None
    node._record_published_translational_speed = lambda v: None
    node._record_precision_tracking_metrics = lambda v: None
    node._publish_speed_debug = lambda *a, **kw: None
    name = "publish_precision_velocity_ned" if precision else "publish_velocity_ned"
    execute([method(RPP, "RPPController", name)], env)

    if precision:
        result = env[name](node, 0.0, NS(requested_speed_mps=0.4))
    else:
        result = env[name](node, 0.0, 0.4, yaw_enu_rad=0.0)

    assert result == (0.0, 0.4, 0.4)
    message, = node.publishers[0].messages
    assert message.header.frame_id == "map_ned"
    if enabled:
        assert (message.velocity_north_mps, message.velocity_east_mps) == (0.0, 0.4)
        assert message.yaw_valid is True
        assert message.yaw_enu_rad == 0.0
    else:
        assert (message.vector.x, message.vector.y, message.vector.z) == (0.0, 0.4, 0.0)


@pytest.mark.parametrize("yaw", [None, math.nan, math.inf, -math.inf])
def test_b_generic_moving_missing_or_nonfinite_yaw_fails_closed(yaw):
    node, env = rpp(True)
    node.MAXIMUM_MOVING_SPEED_MPS = 1.0
    node.acceleration_speed_limit = node.command_speed_slew_limit = lambda v: v
    node.reset_deceleration_profile = lambda: None
    node.reset_speed_profiles = lambda: None
    node.publish_motion_profile_monitor = lambda v: None
    node.command_slew_speed = 0.0
    node.command_slew_last_time = None
    execute([method(RPP, "RPPController", "publish_velocity_ned")], env)

    kwargs = {} if yaw is None else {"yaw_enu_rad": yaw}
    result = env["publish_velocity_ned"](node, 0.3, 0.4, **kwargs)

    assert result == (0.0, 0.0, 0.0)
    message, = node.publishers[0].messages
    assert (message.velocity_north_mps, message.velocity_east_mps) == (0.0, 0.0)
    assert message.yaw_valid is False
    assert message.yaw_enu_rad == 0.0
    assert any("WITHOUT FINITE EXPLICIT YAW" in line for line in node.logs)


def test_b_zero_translation_remains_yaw_invalid_in_patch3():
    node, env = rpp(True)
    node.MAXIMUM_MOVING_SPEED_MPS = 1.0
    node.reset_speed_profiles = lambda: None
    node.publish_motion_profile_monitor = lambda v: None
    node.command_slew_speed = 0.0
    node.command_slew_last_time = None
    execute([method(RPP, "RPPController", "publish_velocity_ned")], env)

    result = env["publish_velocity_ned"](
        node,
        0.0,
        0.0,
        yaw_enu_rad=1.2,
    )

    assert result == (0.0, 0.0, 0.0)
    message, = node.publishers[0].messages
    assert (message.velocity_north_mps, message.velocity_east_mps) == (0.0, 0.0)
    assert message.yaw_valid is False
    assert message.yaw_enu_rad == 0.0


def test_every_generic_moving_call_site_supplies_owning_yaw():
    tree = ast.parse(RPP.read_text())
    calls = [
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "self"
        and call.func.attr == "publish_velocity_ned"
    ]

    without_yaw = [
        call
        for call in calls
        if not any(keyword.arg == "yaw_enu_rad" for keyword in call.keywords)
    ]

    # The only generic publication allowed to omit yaw is publish_stop():
    # literal zero is still yaw_valid=false until Patch 5.
    assert len(without_yaw) == 1
    stop_call = without_yaw[0]
    assert len(stop_call.args) >= 2
    assert ast.literal_eval(stop_call.args[0]) == 0.0
    assert ast.literal_eval(stop_call.args[1]) == 0.0


def test_patch3_adapter_never_reconstructs_yaw_from_velocity():
    adapter = ast.unparse(class_ast(RPP, "_ExplicitYawStagingPublisher"))
    assert "atan2" not in adapter
    assert "publish_with_yaw" in adapter
    assert "yaw_valid = True" in adapter


def make_ready(node):
    node.emergency_stop = False
    node.mission_enabled = node.connected = node.armed = True
    node.mode = "OFFBOARD"
    node.latest_backend_heartbeat_time = node.get_clock().now()


def assert_stop(node):
    message = node.setpoint_pub.messages[-1]
    assert (message.velocity.x, message.velocity.y, message.velocity.z) == (0.0, 0.0, 0.0)
    assert message.type_mask == 3527
    assert message.yaw == message.yaw_rate == 0.0


def test_a_bridge_preserves_enu_mask_stream_rate_and_timeout():
    node = bridge(False)
    make_ready(node)
    msg = Vector()
    msg.vector.x, msg.vector.y = 0.3, 0.4
    node.subscriptions[0].callback(msg)
    node._control_loop()
    out = node.setpoint_pub.messages[-1]
    assert (out.velocity.x, out.velocity.y, out.velocity.z) == (0.4, 0.3, 0.0)
    assert out.type_mask == 3527
    assert out.yaw == out.yaw_rate == 0.0
    assert node.period == 1.0 / 50.0
    stale = Time()
    stale.nanoseconds = -300_000_000
    node.latest_command_time = stale
    node._control_loop()
    assert_stop(node)


@pytest.mark.parametrize("yaw_valid,yaw", [(False, 0.0), (True, 1.2),
                                          (True, math.nan), (True, math.inf)])
def test_b_rejects_every_command_even_valid_yaw_and_clears_cached_motion(yaw_valid, yaw):
    node = bridge(True)
    make_ready(node)
    node._control_loop()
    assert_stop(node)
    node.latest_north, node.latest_east = 0.3, 0.4
    node.latest_command_time = node.get_clock().now()
    msg = Atomic()
    msg.velocity_north_mps, msg.velocity_east_mps = 0.3, 0.4
    msg.yaw_valid, msg.yaw_enu_rad = yaw_valid, yaw
    node.subscriptions[0].callback(msg)
    assert node.latest_north == node.latest_east == 0.0
    assert node.latest_command_time is None
    for _ in range(3):
        node._control_loop()
        assert_stop(node)


@pytest.mark.parametrize("rpp_mode", [False, True])
def test_mismatched_modes_have_no_matching_endpoint_and_stop(rpp_mode):
    producer, _ = rpp(rpp_mode)
    consumer = bridge(not rpp_mode)
    make_ready(consumer)
    assert producer.publishers[0].topic != consumer.subscriptions[0].topic
    assert consumer.latest_command_time is None
    consumer._control_loop()
    assert_stop(consumer)


def test_rpp_existing_restart_only_callback_rejects_new_parameter():
    env = namespace()
    for n in ast.parse(RPP.read_text()).body:
        if isinstance(n, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == "_RUNTIME_SETTABLE_EXEMPT"
                for t in n.targets):
            # Use the actual approved gates, with no ROS module imports.
            gates_path = RPP.with_name("feature_gates.py")
            gates = {}
            exec(compile(gates_path.read_text(), str(gates_path), "exec"), gates)
            env.update(gates)
            execute([n], env)
    env["SetParametersResult"] = NS
    execute([method(RPP, "RPPController", "_on_set_precision_feature_gates")], env)
    result = env["_on_set_precision_feature_gates"](NS(), [NS(name=MODE, value=True)])
    assert result.successful is False
    assert "restart-only" in result.reason
    assert MODE not in env["PRECISION_FEATURE_GATES"]
    assert MODE not in env["_RUNTIME_SETTABLE_EXEMPT"]


@pytest.mark.parametrize("path,owner", [(RPP, "RPPController"), (BRIDGE, "CmdVelBridge")])
@pytest.mark.parametrize("enabled", [False, True])
def test_real_ros_rejects_single_and_mixed_runtime_toggle(path, owner, enabled):
    rclpy = pytest.importorskip("rclpy")
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.context import Context
    from rclpy.node import Node as RosNode
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(context=context)
    node = RosNode("patch2_descriptor_test", context=context,
                   start_parameter_services=False, use_global_arguments=False,
                   parameter_overrides=[Parameter(MODE, value=enabled)])
    try:
        execute(mode_setup(path, owner), dict(self=node, ParameterDescriptor=ParameterDescriptor))
        node.declare_parameter("unrelated_test_parameter", False)
        toggle = Parameter(MODE, value=not enabled)
        assert not node.set_parameters([toggle])[0].successful
        result = node.set_parameters_atomically([
            Parameter("unrelated_test_parameter", value=True), toggle,
        ])
        assert not result.successful
        assert node.get_parameter(MODE).value is enabled
        assert node.rpp_explicit_yaw_enabled is enabled
        assert node.get_parameter("unrelated_test_parameter").value is False
        assert node.set_parameters([
            Parameter("unrelated_test_parameter", value=True),
        ])[0].successful
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
