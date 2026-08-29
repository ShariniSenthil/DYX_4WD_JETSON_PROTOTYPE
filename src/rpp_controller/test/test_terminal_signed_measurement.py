"""Unit tests for the RPP signed terminal 2D measurement.

``RPPController`` subclasses ``rclpy.node.Node``, so importing the module
normally needs a ROS 2 runtime. On a real ROS 2 host, the ``rpp_controller_class``
fixture below imports the module against the *real* rclpy/message packages --
no stand-ins are installed and ``sys.modules`` is left untouched. Only when a
real ``rclpy`` is not importable does it install minimal stand-ins, and even
then only for the duration of the one import call: ``sys.modules`` is
snapshotted first and restored immediately afterward, so the stand-ins never
leak into other test modules or a later real-ROS import in the same process.
No production geometry, FSM, or reporting logic is reimplemented anywhere in
this file: every assertion below calls the real bound ``RPPController``
methods (or the real ``TerminalConfig`` / ``TerminalResult`` /
``PrecisionTerminalCertificate`` / ``TerminalInput`` / ``ControllerPose`` /
``TerminalStopStateMachine`` dataclasses/classes from
``terminal_certificate.py``) against a lightweight, hand-populated instance.
"""

from __future__ import annotations

import importlib
import math
import sys
import types

import pytest

from rpp_controller.terminal_certificate import (
    ControllerPose,
    PrecisionTerminalCertificate,
    TerminalConfig,
    TerminalDirective,
    TerminalInput,
    TerminalResult,
    TerminalState,
    TerminalStopStateMachine,
)


_ROS_STUB_MODULE_NAMES = (
    "rclpy",
    "rclpy.node",
    "rclpy.parameter",
    "rclpy.qos",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "std_msgs",
    "std_msgs.msg",
    "tf_transformations",
)


def _real_ros_is_available():
    """Return True only if every module rpp_controller_node.py imports at
    module scope is the real package, not a stand-in installed by an earlier
    (unrelated) test run in this same process."""

    try:
        import rclpy  # noqa: F401
        import rclpy.node  # noqa: F401
        import rclpy.parameter  # noqa: F401
        import rclpy.qos  # noqa: F401
        import geometry_msgs.msg  # noqa: F401
        import nav_msgs.msg  # noqa: F401
        import rcl_interfaces.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401
        import tf_transformations  # noqa: F401
    except ImportError:
        return False
    return not getattr(sys.modules.get("rclpy"), "_rpp_test_stub", False)


def _build_ros_stub_modules():
    """Build minimal rclpy/message stand-ins, just enough for
    rpp_controller_node.py to import and for RPPController.__new__() to
    produce an (uninitialized) instance. Nothing here re-implements
    controller logic -- only the handful of names touched at
    class-definition time.
    """

    class _StubMsg:
        def __init__(self, *args, **kwargs):
            pass

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module._rpp_test_stub = True
    rclpy_module.init = lambda *a, **k: None
    rclpy_module.spin = lambda *a, **k: None
    rclpy_module.ok = lambda *a, **k: False
    rclpy_module.shutdown = lambda *a, **k: None

    node_module = types.ModuleType("rclpy.node")

    class _StubNode:
        def __init__(self, *args, **kwargs):
            pass

    node_module.Node = _StubNode

    parameter_module = types.ModuleType("rclpy.parameter")

    class _StubParameterType:
        BOOL = object()

    class _StubParameter:
        Type = _StubParameterType

    parameter_module.Parameter = _StubParameter

    qos_module = types.ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = types.SimpleNamespace(
        TRANSIENT_LOCAL=1, VOLATILE=2
    )
    qos_module.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos_module.ReliabilityPolicy = types.SimpleNamespace(
        RELIABLE=1, BEST_EFFORT=2
    )

    class _StubQoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    qos_module.QoSProfile = _StubQoSProfile

    geometry_msgs_module = types.ModuleType("geometry_msgs")
    geometry_msgs_msg_module = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.PoseStamped = _StubMsg
    geometry_msgs_msg_module.Vector3Stamped = _StubMsg
    geometry_msgs_module.msg = geometry_msgs_msg_module

    nav_msgs_module = types.ModuleType("nav_msgs")
    nav_msgs_msg_module = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg_module.Odometry = _StubMsg
    nav_msgs_msg_module.Path = _StubMsg
    nav_msgs_module.msg = nav_msgs_msg_module

    rcl_interfaces_module = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg_module = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg_module.SetParametersResult = _StubMsg
    rcl_interfaces_module.msg = rcl_interfaces_msg_module

    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    for name in (
        "Bool",
        "Float64",
        "Int32MultiArray",
        "String",
        "UInt8MultiArray",
    ):
        setattr(std_msgs_msg_module, name, _StubMsg)
    std_msgs_module.msg = std_msgs_msg_module

    tf_transformations_module = types.ModuleType("tf_transformations")
    tf_transformations_module.euler_from_quaternion = (
        lambda quat: (0.0, 0.0, 0.0)
    )

    return {
        "rclpy": rclpy_module,
        "rclpy.node": node_module,
        "rclpy.parameter": parameter_module,
        "rclpy.qos": qos_module,
        "geometry_msgs": geometry_msgs_module,
        "geometry_msgs.msg": geometry_msgs_msg_module,
        "nav_msgs": nav_msgs_module,
        "nav_msgs.msg": nav_msgs_msg_module,
        "rcl_interfaces": rcl_interfaces_module,
        "rcl_interfaces.msg": rcl_interfaces_msg_module,
        "std_msgs": std_msgs_module,
        "std_msgs.msg": std_msgs_msg_module,
        "tf_transformations": tf_transformations_module,
    }


@pytest.fixture(scope="module")
def rpp_controller_class():
    """Import the real RPPController class.

    On a real ROS 2 host this imports rpp_controller_node.py against the
    real rclpy/message packages and never touches sys.modules. Only in a
    no-ROS environment are stand-ins installed, and only for the duration of
    this one import: sys.modules is snapshotted first and restored right
    after, so no stand-in is left behind for any other test or process.
    """

    if _real_ros_is_available():
        module = importlib.import_module("rpp_controller.rpp_controller_node")
        return module.RPPController

    saved = {name: sys.modules.get(name) for name in _ROS_STUB_MODULE_NAMES}
    for name, stub_module in _build_ros_stub_modules().items():
        sys.modules[name] = stub_module
    try:
        module = importlib.import_module("rpp_controller.rpp_controller_node")
        return module.RPPController
    finally:
        for name, prior_module in saved.items():
            if prior_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prior_module


class _FakeLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _FakeClock:
    def now(self):
        return types.SimpleNamespace(nanoseconds=0)


class _FakePublisher:
    def __init__(self):
        self.last = None

    def publish(self, message):
        self.last = message


class _CapturingFsm:
    """Stands in for TerminalStopStateMachine to capture the exact
    TerminalInput the production code builds and hands to fsm.step(), without
    re-implementing any FSM transition logic (that is covered separately by
    test_terminal_certificate.py). Returns a fixed, well-formed TerminalResult.
    """

    def __init__(self, result):
        self.result = result
        self.received_sample = None

    def step(self, sample):
        self.received_sample = sample
        return self.result


def _bare(rpp_controller_class, **attrs):
    """A real, uninitialized RPPController instance (no ROS I/O performed).

    __new__ skips Node.__init__, so only attributes the exercised method
    actually reads need to be present.
    """

    controller = rpp_controller_class.__new__(rpp_controller_class)
    for key, value in attrs.items():
        setattr(controller, key, value)
    return controller


# ---------------------------------------------------------------------------
# Along-track sign convention (spec items 1-3): + before, 0 on, - overshoot.
# ---------------------------------------------------------------------------


def test_along_8mm_before_waypoint(rpp_controller_class):
    c = _bare(rpp_controller_class, current_x=0.0, current_y=0.0)
    assert c.along_track_remaining(0.0, 0.008, 0.0) == pytest.approx(0.008)


def test_along_exact_on_waypoint(rpp_controller_class):
    c = _bare(rpp_controller_class, current_x=0.0, current_y=0.0)
    assert c.along_track_remaining(0.0, 0.0, 0.0) == pytest.approx(0.0)


def test_along_8mm_overshoot(rpp_controller_class):
    c = _bare(rpp_controller_class, current_x=0.0, current_y=0.0)
    assert c.along_track_remaining(0.0, -0.008, 0.0) == pytest.approx(-0.008)


# ---------------------------------------------------------------------------
# Cross internal/external sign convention (spec items 4-5).
# ---------------------------------------------------------------------------


def test_cross_left_internal_positive_published_negative(rpp_controller_class):
    # bearing 0 (east); rover north of the goal's east-west line -> LEFT.
    c = _bare(rpp_controller_class, current_x=0.0, current_y=0.02)
    internal_cross = c._terminal_cross_internal(0.0, 0.0, 0.0)
    assert internal_cross == pytest.approx(0.02)
    assert rpp_controller_class.ground_xtrack(internal_cross) == pytest.approx(
        -0.02
    )


def test_cross_right_internal_negative_published_positive(rpp_controller_class):
    c = _bare(rpp_controller_class, current_x=0.0, current_y=-0.02)
    internal_cross = c._terminal_cross_internal(0.0, 0.0, 0.0)
    assert internal_cross == pytest.approx(-0.02)
    assert rpp_controller_class.ground_xtrack(internal_cross) == pytest.approx(
        0.02
    )


# ---------------------------------------------------------------------------
# Headings, non-contamination, and the radial**2 = along**2 + cross**2
# invariant (spec items 6-8), all from one closed-form construction so any
# (heading, along, cross) triple is exactly reproducible from a rover pose.
# ---------------------------------------------------------------------------


def _pose_for_along_cross(goal_x, goal_y, bearing, along, cross):
    """Return (current_x, current_y) that reproduces the requested Along/Cross
    for the given goal and bearing exactly (closed-form, invertible 2x2)."""

    delta_east = math.cos(bearing) * along + math.sin(bearing) * cross
    delta_north = math.sin(bearing) * along - math.cos(bearing) * cross
    return goal_x - delta_east, goal_y - delta_north


@pytest.mark.parametrize("heading_deg", [0.0, 90.0, 37.0, -125.0])
def test_headings_reproduce_along_and_cross_exactly(
    rpp_controller_class, heading_deg
):
    bearing = math.radians(heading_deg)
    goal_x, goal_y = 12.5, -4.25
    for along, cross in ((0.5, 0.0), (0.5, 0.1), (0.5, -0.1), (0.0, 0.2)):
        current_x, current_y = _pose_for_along_cross(
            goal_x, goal_y, bearing, along, cross
        )
        c = _bare(rpp_controller_class, current_x=current_x, current_y=current_y)
        assert c.along_track_remaining(bearing, goal_x, goal_y) == pytest.approx(
            along, abs=1e-9
        )
        assert c._terminal_cross_internal(
            bearing, goal_x, goal_y
        ) == pytest.approx(cross, abs=1e-9)


def test_lateral_displacement_does_not_contaminate_along(rpp_controller_class):
    bearing = math.radians(37.0)
    goal_x, goal_y = 3.0, 7.0
    along = 0.5
    for cross in (-0.3, -0.05, 0.0, 0.05, 0.3):
        current_x, current_y = _pose_for_along_cross(
            goal_x, goal_y, bearing, along, cross
        )
        c = _bare(rpp_controller_class, current_x=current_x, current_y=current_y)
        assert c.along_track_remaining(bearing, goal_x, goal_y) == pytest.approx(
            along, abs=1e-9
        )


@pytest.mark.parametrize("heading_deg", [0.0, 90.0, 37.0, -125.0])
def test_synthetic_radial_equals_along_squared_plus_cross_squared(
    rpp_controller_class, heading_deg
):
    bearing = math.radians(heading_deg)
    goal_x, goal_y = -8.0, 2.5
    for along, cross in ((0.5, 0.1), (0.008, 0.0), (-0.008, 0.03), (0.02, -0.02)):
        current_x, current_y = _pose_for_along_cross(
            goal_x, goal_y, bearing, along, cross
        )
        c = _bare(rpp_controller_class, current_x=current_x, current_y=current_y)
        radial = math.hypot(goal_x - current_x, goal_y - current_y)
        computed_along = c.along_track_remaining(bearing, goal_x, goal_y)
        computed_cross = c._terminal_cross_internal(bearing, goal_x, goal_y)
        residual = radial**2 - (computed_along**2 + computed_cross**2)
        assert abs(residual) < 1e-9


# ---------------------------------------------------------------------------
# Real end-to-end wiring through the production
# RPPController._step_precision_terminal_for_cycle(): the exact TerminalInput
# it builds and hands to the FSM is captured and asserted here, rather than
# recomputing any of the geometry outside production code (item 10 folds in
# here too -- distance_to_terminal_m's clamp is exercised live).
# ---------------------------------------------------------------------------


def _step_cycle_controller(rpp_controller_class, *, fsm_result):
    """A bare controller wired just enough for the real
    _step_precision_terminal_for_cycle() to run its full body: current-cycle
    projection, synchronized goal identity, tangent latch, Along/Cross
    construction, and the FSM step -- nothing about that path is stubbed out
    or reimplemented, only its ROS-only side effects (clock/logger/publish).
    """

    c = _bare(
        rpp_controller_class,
        precision_terminal_cycle_token="prior-cycle",
        precision_cycle_token="cycle-1",
        geometry_last_projection_cycle_token="cycle-1",
        geometry_last_projection=types.SimpleNamespace(projected_s=10.0),
        precision_guidance_result=object(),
        geometry_contract_synchronized=True,
        geometry_goal_binding=types.SimpleNamespace(
            raw_path_index=1, active_goal_identity="goal-1"
        ),
        geometry_pending_goal_metadata={
            "path_signature": "sig-1",
            "raw_path_index": 1,
            "active_goal_identity": "goal-1",
            "mission_run_id": "run-1",
            "goal_instance_id": "inst-1",
        },
        geometry_installed_signature="sig-1",
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        c_line_bearing=0.0,
        current_x=10.008,
        current_y=0.0,
        current_yaw=0.0,
        current_speed_mps=0.0,
        current_yaw_rate_radps=0.0,
        last_odom_time=None,
        precision_terminal_telemetry_timeout_sec=1.0,
        geometry_active_span=types.SimpleNamespace(stop_s=10.0),
        precision_cycle_dt_sec=0.1,
        precision_terminal_config=TerminalConfig(),
        precision_terminal_historical_certificate=None,
        precision_terminal_request_armed=False,
        precision_terminal_identity=None,
        precision_terminal_identity_components=None,
        precision_terminal_last_sample=None,
        precision_terminal_last_result=None,
        precision_terminal_speed_override_mps=None,
        precision_terminal_enabled=True,
        precision_terminal_last_reset_reason="INITIALIZE",
        precision_terminal_reset_count=0,
    )
    c.get_logger = lambda: _FakeLogger()
    c.get_clock = lambda: _FakeClock()
    c.terminal_certificate_pub = _FakePublisher()
    c.is_fresh = lambda timestamp, timeout: False
    c.precision_terminal_fsm = _CapturingFsm(fsm_result)
    return c


def test_live_step_produces_signed_along_and_internal_cross_via_real_fsm_call(
    rpp_controller_class,
):
    fsm_result = TerminalResult(
        state=TerminalState.BRAKE,
        previous_state=TerminalState.APPROACH,
        directive=TerminalDirective.BRAKE,
        terminal_identity="RUN:run-1|PATH:sig-1|RAW:1|GOAL:goal-1|INSTANCE:inst-1",
        zero_latched=False,
        transition_reason="TEST_STEP",
        bounded_dt_sec=0.1,
        state_elapsed_sec=0.0,
        settle_held_sec=0.0,
        motion_evidence_valid=True,
        currently_valid=True,
        certificate=None,
    )
    c = _step_cycle_controller(rpp_controller_class, fsm_result=fsm_result)

    # goal at (10.0, 0.0); rover 8mm past it along the 0rad tangent, exactly
    # on the line (no lateral offset) -- and the span projection is already
    # endpoint-clamped (projected_s == stop_s == 10.0).
    result = c._step_precision_terminal_for_cycle(
        goal_distance=0.008,
        path_heading_error=0.0,
        first_approach=True,
        path_bearing=0.0,
        goal_x=10.0,
        goal_y=0.0,
    )

    assert result is fsm_result
    sample = c.precision_terminal_fsm.received_sample
    assert sample is not None
    assert sample.distance_to_terminal_m == pytest.approx(0.0)
    assert sample.along_track_error_m == pytest.approx(-0.008)
    assert sample.cross_track_error_m == pytest.approx(0.0)
    assert sample.radial_error_m == pytest.approx(0.008)

    # The measurement tangent actually used (and latched for this identity)
    # was runtime-entry authority, not a fabricated or independently-derived
    # bearing.
    assert c.precision_terminal_measurement_bearing == pytest.approx(0.0)
    assert (
        c.precision_terminal_measurement_bearing_source
        == "RUNTIME_ENTRY_C_TO_P1"
    )


# ---------------------------------------------------------------------------
# Latch reset / cross-goal isolation (item 2): the real
# _reset_precision_terminal() must clear the latch, and a later terminal
# identity's latch must never observe the prior terminal's bearing.
# ---------------------------------------------------------------------------


def test_reset_clears_latch_and_next_terminal_cannot_reuse_prior_bearing(
    rpp_controller_class,
):
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        precision_terminal_cycle_token="cycle-1",
        precision_terminal_last_result="stale-result",
        precision_terminal_request_armed=True,
        precision_terminal_identity="stale-identity",
        precision_terminal_identity_components={"stale": True},
        precision_terminal_last_sample="stale-sample",
        precision_terminal_speed_override_mps=0.04,
        precision_terminal_historical_certificate="stale-certificate",
        precision_terminal_last_reset_reason="INITIALIZE",
        precision_terminal_reset_count=0,
        precision_terminal_config=TerminalConfig(),
        c_line_bearing=0.35,
        path_geometry=None,
        geometry_goal_binding=None,
    )
    c.get_logger = lambda: _FakeLogger()
    c.get_clock = lambda: _FakeClock()
    # The real FSM class (not a capturing stand-in): reset() is plain-Python
    # state-machine bookkeeping with no ROS dependency.
    c.precision_terminal_fsm = TerminalStopStateMachine(c.precision_terminal_config)

    # Terminal A latches bearing A via runtime-entry authority.
    bearing_a, source_a = c._resolve_precision_terminal_measurement_bearing(
        first_approach=True,
        path_bearing=1.2,
    )
    assert bearing_a == pytest.approx(0.35)
    assert source_a == "RUNTIME_ENTRY_C_TO_P1"
    assert c.precision_terminal_measurement_bearing == pytest.approx(0.35)

    # Real semantic-boundary reset.
    c._reset_precision_terminal("TEST_BOUNDARY")
    assert c.precision_terminal_measurement_bearing is None
    assert c.precision_terminal_measurement_bearing_source is None
    assert c.precision_terminal_last_reset_reason == "TEST_BOUNDARY"
    assert c.precision_terminal_reset_count == 1
    assert c.precision_terminal_request_armed is False
    assert c.precision_terminal_identity is None
    assert c.precision_terminal_last_sample is None

    # Terminal B resolves/latches a different bearing via a different source.
    # If A had leaked, this would observe 0.35/RUNTIME_ENTRY_C_TO_P1 instead.
    c.c_line_bearing = 9.99  # first_approach is False below; must be ignored.
    anchor_b = types.SimpleNamespace(incoming_heading_rad=1.05)
    c.path_geometry = types.SimpleNamespace(
        semantic_anchor_at=lambda raw_index: anchor_b
    )
    c.geometry_goal_binding = types.SimpleNamespace(raw_path_index=7)
    bearing_b, source_b = c._resolve_precision_terminal_measurement_bearing(
        first_approach=False,
        path_bearing=2.4,
    )
    assert bearing_b == pytest.approx(1.05)
    assert source_b == "SEMANTIC_INCOMING"
    assert bearing_b != pytest.approx(bearing_a)
    assert c.precision_terminal_measurement_bearing == pytest.approx(1.05)


# ---------------------------------------------------------------------------
# Measurement-tangent latch: runtime-entry / semantic / degenerate fallback,
# and latch stability across a later path_bearing change (items 19-22).
# ---------------------------------------------------------------------------


def test_first_approach_latches_runtime_entry_c_line_bearing(rpp_controller_class):
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        c_line_bearing=0.35,
        path_geometry=None,
        geometry_goal_binding=None,
    )
    c.get_logger = lambda: _FakeLogger()
    bearing, source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=True,
        path_bearing=1.2,
    )
    assert bearing == pytest.approx(0.35)
    assert source == "RUNTIME_ENTRY_C_TO_P1"
    assert c.precision_terminal_measurement_bearing == pytest.approx(0.35)


def test_first_approach_with_non_finite_c_line_bearing_does_not_fabricate(
    rpp_controller_class,
):
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        c_line_bearing=None,
    )
    c.get_logger = lambda: _FakeLogger()
    bearing, source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=True,
        path_bearing=1.2,
    )
    assert bearing is None
    assert source is None
    assert c.precision_terminal_measurement_bearing is None


def test_normal_waypoint_latches_semantic_incoming_heading(rpp_controller_class):
    anchor = types.SimpleNamespace(incoming_heading_rad=0.77)
    path_geometry = types.SimpleNamespace(
        semantic_anchor_at=lambda raw_index: anchor
    )
    binding = types.SimpleNamespace(raw_path_index=4)
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        path_geometry=path_geometry,
        geometry_goal_binding=binding,
    )
    c.get_logger = lambda: _FakeLogger()
    bearing, source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=False,
        path_bearing=1.9,
    )
    assert bearing == pytest.approx(0.77)
    assert source == "SEMANTIC_INCOMING"


def test_degenerate_semantic_anchor_falls_back_to_active_nav_and_warns(
    rpp_controller_class,
):
    path_geometry = types.SimpleNamespace(
        semantic_anchor_at=lambda raw_index: None
    )
    binding = types.SimpleNamespace(raw_path_index=4)
    logger = _FakeLogger()
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        path_geometry=path_geometry,
        geometry_goal_binding=binding,
    )
    c.get_logger = lambda: logger
    bearing, source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=False,
        path_bearing=1.9,
    )
    assert bearing == pytest.approx(1.9)
    assert source == "ACTIVE_NAV_FALLBACK"
    assert any("ACTIVE_NAV_FALLBACK" in message for message in logger.warnings)


def test_degenerate_with_no_active_path_bearing_does_not_fabricate(
    rpp_controller_class,
):
    path_geometry = types.SimpleNamespace(
        semantic_anchor_at=lambda raw_index: None
    )
    binding = types.SimpleNamespace(raw_path_index=4)
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        path_geometry=path_geometry,
        geometry_goal_binding=binding,
    )
    c.get_logger = lambda: _FakeLogger()
    bearing, source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=False,
        path_bearing=None,
    )
    assert bearing is None
    assert source is None


def test_latch_is_stable_once_set_even_if_path_bearing_later_changes(
    rpp_controller_class,
):
    c = _bare(
        rpp_controller_class,
        precision_terminal_measurement_bearing=None,
        precision_terminal_measurement_bearing_source=None,
        c_line_bearing=0.35,
    )
    c.get_logger = lambda: _FakeLogger()
    first, _ = c._resolve_precision_terminal_measurement_bearing(
        first_approach=True,
        path_bearing=1.2,
    )
    assert first == pytest.approx(0.35)

    # Runtime bearing changes after latch -- the latched value must not move.
    c.c_line_bearing = 1.11
    second, second_source = c._resolve_precision_terminal_measurement_bearing(
        first_approach=True,
        path_bearing=2.4,
    )
    assert second == pytest.approx(0.35)
    assert second_source == "RUNTIME_ENTRY_C_TO_P1"


# ---------------------------------------------------------------------------
# Diagnostic-only consistency check: warns but never raises or gates.
# ---------------------------------------------------------------------------


def test_consistency_check_is_silent_when_consistent(rpp_controller_class):
    logger = _FakeLogger()
    c = _bare(rpp_controller_class)
    c.get_logger = lambda: logger
    c._check_precision_terminal_measurement_consistency(
        radial_error_m=0.5,
        along_error_m=0.4,
        cross_error_m=0.3,
    )
    assert logger.warnings == []


def test_consistency_check_warns_but_does_not_raise_when_inconsistent(
    rpp_controller_class,
):
    logger = _FakeLogger()
    c = _bare(rpp_controller_class)
    c.get_logger = lambda: logger
    c._check_precision_terminal_measurement_consistency(
        radial_error_m=1.0,
        along_error_m=0.0,
        cross_error_m=0.0,
    )
    assert len(logger.warnings) == 1


def test_consistency_check_ignores_non_finite_input(rpp_controller_class):
    logger = _FakeLogger()
    c = _bare(rpp_controller_class)
    c.get_logger = lambda: logger
    c._check_precision_terminal_measurement_consistency(
        radial_error_m=math.nan,
        along_error_m=0.0,
        cross_error_m=0.0,
    )
    assert logger.warnings == []


# ---------------------------------------------------------------------------
# publish_terminal_result: tolerance_override_m contract (item 8), single
# Cross inversion (item 13), and mm conversion (item 24).
# ---------------------------------------------------------------------------


def _publish_result_controller(rpp_controller_class, *, precision_terminal_enabled):
    return _bare(
        rpp_controller_class,
        _terminal_result_sent=None,
        segment_goal_x=1.0,
        segment_goal_y=2.0,
        segment_goal_number=3,
        current_speed_mps=0.0,
        waypoint_tolerance=0.03,
        precision_terminal_enabled=precision_terminal_enabled,
        precision_terminal_identity="RUN:x|PATH:y|RAW:1|GOAL:g|INSTANCE:i",
        precision_terminal_identity_components={
            "mission_run_id": "run-1",
            "goal_instance_id": "goal-1",
            "path_signature": "sig-1",
            "raw_path_index": 1,
            "active_goal_identity": "goal-1",
        },
        precision_terminal_config=TerminalConfig(),
        current_yaw_rate_radps=0.0,
        last_odom_time=None,
        precision_terminal_telemetry_timeout_sec=1.0,
        get_clock=lambda: _FakeClock(),
        get_logger=lambda: _FakeLogger(),
        terminal_result_pub=_FakePublisher(),
    )


def test_legacy_caller_without_override_keeps_waypoint_tolerance(
    rpp_controller_class,
):
    c = _publish_result_controller(rpp_controller_class, precision_terminal_enabled=False)
    c.publish_terminal_result(
        "CAPTURED",
        reason="RADIUS_30MM_STATIONARY",
        target_distance=0.010,
        signed_cross_track=0.004,
        along_remaining=0.006,
    )
    msg = c.terminal_result_pub.last
    assert msg is not None
    assert '"tolerance_mm":30.0' in msg.data


def test_precision_caller_with_override_uses_10mm_tolerance_and_single_inversion(
    rpp_controller_class,
):
    c = _publish_result_controller(rpp_controller_class, precision_terminal_enabled=True)
    c.publish_terminal_result(
        "CAPTURED",
        reason="PRECISION_TERMINAL_CERTIFIED_V2",
        target_distance=0.009,
        signed_cross_track=0.0035,  # internal LEFT+
        along_remaining=-0.0021,  # signed overshoot
        tolerance_override_m=0.010,
    )
    msg = c.terminal_result_pub.last
    assert msg is not None
    assert '"tolerance_mm":10.0' in msg.data
    # Single inversion: internal LEFT+0.0035 -> published RIGHT? no: ground
    # convention is LEFT-/RIGHT+, so LEFT internal + becomes negative once.
    assert '"cross_track_error_mm":-3.5' in msg.data
    assert '"along_track_error_mm":-2.1' in msg.data


def test_goal_passed_moving_away_with_precision_enabled_keeps_legacy_tolerance(
    rpp_controller_class,
):
    # Mirrors the real control_loop call site for GOAL_PASSED_MOVING_AWAY,
    # which never passes tolerance_override_m -- precision being enabled
    # must not leak the 10mm precision spec into this legacy-reason result.
    c = _publish_result_controller(rpp_controller_class, precision_terminal_enabled=True)
    c.publish_terminal_result(
        "MISSED",
        reason="GOAL_PASSED_MOVING_AWAY",
        target_distance=0.05,
        signed_cross_track=-0.006,  # internal RIGHT-
        along_remaining=-0.02,
    )
    msg = c.terminal_result_pub.last
    assert msg is not None
    assert '"tolerance_mm":30.0' in msg.data
    # Single inversion: internal RIGHT-0.006 -> published LEFT? ground
    # convention flips sign once: RIGHT- becomes positive.
    assert '"cross_track_error_mm":6.0' in msg.data


# ---------------------------------------------------------------------------
# _publish_precision_terminal_result_if_ready: CAPTURED / HOLD_FAIL bridge
# uses the (now signed) sample.along_track_error_m / cross_track_error_m,
# each certified at the 10mm precision tolerance (items 14-15).
# ---------------------------------------------------------------------------


def _certificate(**overrides):
    values = dict(
        version=2,
        terminal_identity="RUN:x|PATH:y|RAW:1|GOAL:g|INSTANCE:i",
        precision_pass=True,
        radial_error_mm=9.0,
        cross_error_mm=3.5,
        along_error_mm=-2.1,
        heading_error_deg=1.0,
        measured_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
        stop_spec_mm=10.0,
        settle_sec=0.3,
        first_capture_pose=None,
        final_settled_pose=None,
        max_radial_during_settle_mm=9.0,
        capture_timestamp_sec=1.0,
        settle_started_timestamp_sec=1.0,
        certified_timestamp_sec=1.3,
    )
    values.update(overrides)
    return PrecisionTerminalCertificate(**values)


def _sample(**overrides):
    values = dict(
        monotonic_time_sec=1.0,
        dt_sec=0.1,
        terminal_requested=True,
        terminal_identity="RUN:x|PATH:y|RAW:1|GOAL:g|INSTANCE:i",
        distance_to_terminal_m=0.0,
        radial_error_m=0.009,
        cross_track_error_m=0.0035,
        along_track_error_m=-0.0021,
        measured_linear_speed_mps=0.0,
        measured_yaw_rate_radps=0.0,
        telemetry_fresh=True,
        braking_required=True,
        heading_error_deg=1.0,
        current_pose=ControllerPose(1.0, 2.0, 0.0),
    )
    values.update(overrides)
    return TerminalInput(**values)


def test_precision_captured_bridges_signed_along_and_cross_at_10mm(
    rpp_controller_class,
):
    c = _publish_result_controller(rpp_controller_class, precision_terminal_enabled=True)
    c.precision_terminal_last_sample = _sample()
    c.precision_terminal_historical_certificate = None
    certificate = _certificate()
    result = TerminalResult(
        state=TerminalState.CERTIFIED,
        previous_state=TerminalState.SETTLE,
        directive=TerminalDirective.HOLD_ZERO,
        terminal_identity=c.precision_terminal_identity,
        zero_latched=True,
        transition_reason="CERTIFIED",
        bounded_dt_sec=0.1,
        state_elapsed_sec=0.3,
        settle_held_sec=0.3,
        motion_evidence_valid=True,
        currently_valid=True,
        certificate=certificate,
    )
    c._publish_precision_terminal_result_if_ready(result)
    msg = c.terminal_result_pub.last
    assert msg is not None
    assert '"outcome":"CAPTURED"' in msg.data
    assert '"tolerance_mm":10.0' in msg.data
    assert '"along_track_error_mm":-2.1' in msg.data
    assert '"cross_track_error_mm":-3.5' in msg.data
    assert '"radial_error_mm":9.0' in msg.data


def test_precision_hold_fail_bridges_signed_along_and_cross_at_10mm(
    rpp_controller_class,
):
    c = _publish_result_controller(rpp_controller_class, precision_terminal_enabled=True)
    c.precision_terminal_last_sample = _sample(
        cross_track_error_m=-0.006,
        along_track_error_m=0.012,
    )
    c.precision_terminal_historical_certificate = None
    result = TerminalResult(
        state=TerminalState.HOLD_FAIL,
        previous_state=TerminalState.SETTLE,
        directive=TerminalDirective.HOLD_FAIL,
        terminal_identity=c.precision_terminal_identity,
        zero_latched=True,
        transition_reason="SETTLE_RADIAL_EXCEEDED",
        bounded_dt_sec=0.1,
        state_elapsed_sec=0.3,
        settle_held_sec=0.1,
        motion_evidence_valid=True,
        currently_valid=False,
        certificate=None,
    )
    c._publish_precision_terminal_result_if_ready(result)
    msg = c.terminal_result_pub.last
    assert msg is not None
    assert '"outcome":"MISSED"' in msg.data
    assert '"tolerance_mm":10.0' in msg.data
    assert '"along_track_error_mm":12.0' in msg.data
    assert '"cross_track_error_mm":6.0' in msg.data


# ---------------------------------------------------------------------------
# ground_terminal_certificate_payload: exactly one Cross inversion (item 13).
# ---------------------------------------------------------------------------


def test_certificate_payload_inverts_cross_exactly_once(rpp_controller_class):
    c = _bare(rpp_controller_class)
    certificate = _certificate(cross_error_mm=4.2)
    payload = c.ground_terminal_certificate_payload(certificate)
    assert payload["cross_error_mm"] == pytest.approx(-4.2)
    # Certificate storage itself (frozen at construction) keeps the internal
    # convention untouched by the payload conversion.
    assert certificate.cross_error_mm == pytest.approx(4.2)


def test_certificate_payload_none_passthrough(rpp_controller_class):
    c = _bare(rpp_controller_class)
    assert c.ground_terminal_certificate_payload(None) is None


# ---------------------------------------------------------------------------
# Hold/refresh geometry consistency (item 23): a refreshed radial can never
# be combined with a stale Along/Cross, and vice versa.
# ---------------------------------------------------------------------------


def test_hold_cycle_refreshes_radial_along_and_cross_together(
    rpp_controller_class,
):
    bearing = math.radians(20.0)
    goal_x, goal_y = 5.0, -1.0
    prior_current_x, prior_current_y = _pose_for_along_cross(
        goal_x, goal_y, bearing, 0.5, 0.1
    )
    prior_sample = _sample(
        radial_error_m=math.hypot(goal_x - prior_current_x, goal_y - prior_current_y),
        along_track_error_m=0.5,
        cross_track_error_m=0.1,
        current_pose=ControllerPose(prior_current_x, prior_current_y, 0.0),
    )

    new_current_x, new_current_y = _pose_for_along_cross(
        goal_x, goal_y, bearing, 0.2, -0.05
    )
    c = _bare(
        rpp_controller_class,
        precision_terminal_request_armed=True,
        precision_terminal_last_sample=prior_sample,
        precision_terminal_cycle_token="cycle-1",
        precision_cycle_token="cycle-2",
        precision_terminal_measurement_bearing=bearing,
        precision_terminal_measurement_bearing_source="SEMANTIC_INCOMING",
        current_x=new_current_x,
        current_y=new_current_y,
        current_yaw=0.0,
        current_speed_mps=0.0,
        current_yaw_rate_radps=0.0,
        segment_goal_x=goal_x,
        segment_goal_y=goal_y,
        last_odom_time=None,
        precision_terminal_telemetry_timeout_sec=1.0,
        precision_terminal_identity="RUN:x|PATH:y|RAW:1|GOAL:g|INSTANCE:i",
        precision_cycle_dt_sec=0.1,
        precision_terminal_config=TerminalConfig(),
        precision_terminal_historical_certificate=None,
        precision_terminal_enabled=True,
        precision_terminal_identity_components={},
        waypoint_tolerance=0.03,
        segment_goal_number=1,
        _terminal_result_sent=None,
    )
    c.get_logger = lambda: _FakeLogger()
    c.get_clock = lambda: _FakeClock()
    c.terminal_certificate_pub = _FakePublisher()
    c.terminal_result_pub = _FakePublisher()
    c.is_fresh = lambda timestamp, timeout: False

    class _FrozenFsm:
        def step(self, sample):
            return TerminalResult(
                state=TerminalState.APPROACH,
                previous_state=TerminalState.APPROACH,
                directive=TerminalDirective.BRAKE,
                terminal_identity=sample.terminal_identity,
                zero_latched=False,
                transition_reason="HOLD_REFRESH",
                bounded_dt_sec=sample.dt_sec,
                state_elapsed_sec=0.1,
                settle_held_sec=0.0,
                motion_evidence_valid=True,
                currently_valid=True,
                certificate=None,
            )

    c.precision_terminal_fsm = _FrozenFsm()

    result = c._step_precision_terminal_hold_cycle()
    assert result is not None
    stepped_sample = c.precision_terminal_last_sample
    assert stepped_sample.radial_error_m == pytest.approx(
        math.hypot(goal_x - new_current_x, goal_y - new_current_y)
    )
    assert stepped_sample.along_track_error_m == pytest.approx(0.2, abs=1e-9)
    assert stepped_sample.cross_track_error_m == pytest.approx(-0.05, abs=1e-9)


def test_hold_cycle_preserves_whole_prior_sample_when_pose_invalid(
    rpp_controller_class,
):
    prior_sample = _sample(
        radial_error_m=0.30,
        along_track_error_m=0.25,
        cross_track_error_m=0.06,
    )
    c = _bare(
        rpp_controller_class,
        precision_terminal_request_armed=True,
        precision_terminal_last_sample=prior_sample,
        precision_terminal_cycle_token="cycle-1",
        precision_cycle_token="cycle-2",
        precision_terminal_measurement_bearing=0.4,
        precision_terminal_measurement_bearing_source="SEMANTIC_INCOMING",
        current_x=None,  # invalid pose -- cannot refresh.
        current_y=None,
        current_yaw=None,
        current_speed_mps=0.0,
        current_yaw_rate_radps=0.0,
        segment_goal_x=5.0,
        segment_goal_y=-1.0,
        last_odom_time=None,
        precision_terminal_telemetry_timeout_sec=1.0,
        precision_terminal_identity="RUN:x|PATH:y|RAW:1|GOAL:g|INSTANCE:i",
        precision_cycle_dt_sec=0.1,
        precision_terminal_config=TerminalConfig(),
        precision_terminal_historical_certificate=None,
        precision_terminal_enabled=True,
        precision_terminal_identity_components={},
        waypoint_tolerance=0.03,
        segment_goal_number=1,
        _terminal_result_sent=None,
    )
    c.get_logger = lambda: _FakeLogger()
    c.get_clock = lambda: _FakeClock()
    c.terminal_certificate_pub = _FakePublisher()
    c.terminal_result_pub = _FakePublisher()
    c.is_fresh = lambda timestamp, timeout: False

    class _FrozenFsm:
        def step(self, sample):
            return TerminalResult(
                state=TerminalState.APPROACH,
                previous_state=TerminalState.APPROACH,
                directive=TerminalDirective.BRAKE,
                terminal_identity=sample.terminal_identity,
                zero_latched=False,
                transition_reason="HOLD_REFRESH",
                bounded_dt_sec=sample.dt_sec,
                state_elapsed_sec=0.1,
                settle_held_sec=0.0,
                motion_evidence_valid=True,
                currently_valid=True,
                certificate=None,
            )

    c.precision_terminal_fsm = _FrozenFsm()

    result = c._step_precision_terminal_hold_cycle()
    assert result is not None
    stepped_sample = c.precision_terminal_last_sample
    assert stepped_sample.radial_error_m == pytest.approx(0.30)
    assert stepped_sample.along_track_error_m == pytest.approx(0.25)
    assert stepped_sample.cross_track_error_m == pytest.approx(0.06)
