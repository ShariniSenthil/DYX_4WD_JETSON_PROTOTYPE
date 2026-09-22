"""The fixed RTK placement dwell and its parameter are gone for good.

Operators must not be able to set a parameter that looks like it gates
trajectory preparation but does nothing.
"""

from pathlib import Path


PACKAGE = Path(__file__).parents[1]
NODE = PACKAGE / "trajectory_generator" / "trajectory_generator_node.py"
LAUNCH = (
    PACKAGE.parent / "rover_bringup" / "launch" / "rover.launch.py"
)


def test_node_has_no_rtk_dwell_state_or_parameter():
    source = NODE.read_text(encoding="utf-8")
    assert "rtk_stable_sec" not in source
    assert "rtk_ready_since" not in source


def test_launch_does_not_configure_removed_dwell():
    if not LAUNCH.exists():
        return
    assert "rtk_stable_sec" not in LAUNCH.read_text(encoding="utf-8")
