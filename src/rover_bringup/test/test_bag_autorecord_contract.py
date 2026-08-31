"""Protect the precision-control rosbag evidence contract."""

import ast
from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[3]
AUTORECORDER = REPO_ROOT / "scripts" / "bag_autorecord.py"
FIELD_LOGGER = REPO_ROOT / "scripts" / "start_field_test_logging.sh"
QOS_OVERRIDES = REPO_ROOT / "config" / "rosbag_qos_overrides.yaml"

PRECISION_TOPICS = {
    "/trajectory_generator/path_signature",
    "/mission_manager/segment_goal_metadata",
    "/rpp/geometry_debug",
    "/rpp/guidance_debug",
    "/rpp/speed_debug",
    "/rpp/tracking_debug",
    "/rpp/pivot_debug",
    "/rpp/terminal_certificate",
    "/rpp/terminal_result",
    "/rpp/debug",
}


def _autorecorder_topics():
    module = ast.parse(AUTORECORDER.read_text(encoding="utf-8"))
    for node in module.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == "TOPICS"
                   for target in node.targets):
                return set(ast.literal_eval(node.value))
    raise AssertionError("bag_autorecord.py does not define TOPICS")


def _field_logger_topics():
    source = FIELD_LOGGER.read_text(encoding="utf-8")
    match = re.search(
        r"^TOPICS=\(\n(?P<body>.*?)^\)\n",
        source,
        re.MULTILINE | re.DOTALL,
    )
    assert match is not None, "field logger does not define a TOPICS array"
    return set(
        re.findall(r"^\s*(/\S+)\s*$", match.group("body"), re.MULTILINE)
    )


def test_precision_topics_are_recorded_by_both_entry_points():
    """Require precision evidence in automatic and manual recorders."""
    assert PRECISION_TOPICS <= _autorecorder_topics()
    assert PRECISION_TOPICS <= _field_logger_topics()


def test_path_signature_uses_latched_publisher_qos():
    """Match the path-signature publisher's retained QoS contract."""
    source = QOS_OVERRIDES.read_text(encoding="utf-8")
    match = re.search(
        r"^/trajectory_generator/path_signature:\n(?P<body>(?:  .+\n)+)",
        source,
        re.MULTILINE,
    )
    assert match is not None, "path signature QoS override is missing"
    qos = match.group("body")
    assert "reliability: reliable" in qos
    assert "durability: transient_local" in qos


def test_volatile_precision_topics_have_no_transient_local_override():
    """Avoid incompatible retained subscriptions to volatile publishers."""
    source = QOS_OVERRIDES.read_text(encoding="utf-8")
    retained_topics = {"/trajectory_generator/path_signature"}
    volatile_topics = PRECISION_TOPICS - retained_topics
    for topic in volatile_topics:
        assert f"\n{topic}:" not in source


def test_parameter_snapshots_allow_slow_rpp_dump_and_report_failures():
    """Keep complete, diagnosable controller parameter evidence."""
    source = AUTORECORDER.read_text(encoding="utf-8")
    assert 'BAG_PARAM_DUMP_TIMEOUT_S", "15"' in source
    assert "timeout=PARAM_DUMP_TIMEOUT_S" in source
    assert '"errors": errors' in source
    module = ast.parse(source)
    param_nodes = next(
        ast.literal_eval(node.value)
        for node in module.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "PARAM_NODES"
            for target in node.targets
        )
    )
    assert "/ntrip_to_px4_node" in param_nodes
    assert "/rtk_correction_bridge" not in param_nodes
