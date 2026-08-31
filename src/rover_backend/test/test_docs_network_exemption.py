"""Focused guard for the public-docs network-gate exemption.

_is_public_docs_request() lives in rover_backend.main, a module that
transitively imports rclpy/mavros_msgs at import time. Importing it here
would require the full ROS environment, so this test extracts the
function's own source via AST and execs it standalone -- the same
isolation technique used by rpp_controller's
test_runtime_entry_authority.py. Only GET/HEAD to the exact docs paths
must be exempted; every other method or path must still hit the network
allowlist gate.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MAIN_PATH = PACKAGE_ROOT / "rover_backend" / "main.py"
MAIN_SOURCE = MAIN_PATH.read_text(encoding="utf-8")
MAIN_TREE = ast.parse(MAIN_SOURCE)
sys.path.insert(0, str(PACKAGE_ROOT))


def _load_function(name: str):
    node = next(
        item
        for item in MAIN_TREE.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    module = ast.fix_missing_locations(
        ast.Module(body=[node], type_ignores=[])
    )
    namespace: dict = {}
    exec(compile(module, str(MAIN_PATH), "exec"), namespace)
    return namespace[name]


_is_public_docs_request = _load_function("_is_public_docs_request")


@dataclass
class _Url:
    path: str


@dataclass
class _Request:
    method: str
    url: _Url


def test_get_docs_is_exempt():
    assert _is_public_docs_request(_Request("GET", _Url("/api/docs")))


def test_head_docs_is_exempt():
    assert _is_public_docs_request(_Request("HEAD", _Url("/api/docs")))


def test_get_openapi_json_is_exempt():
    assert _is_public_docs_request(_Request("GET", _Url("/api/openapi.json")))


def test_post_to_docs_path_is_not_exempt():
    """A read-only exemption must not cover a mutating method."""
    assert not _is_public_docs_request(_Request("POST", _Url("/api/docs")))


def test_unrelated_get_path_is_not_exempt():
    assert not _is_public_docs_request(_Request("GET", _Url("/api/missions")))


def test_docs_path_prefix_is_not_exempt():
    """Only the exact docs paths are public, not everything under them."""
    assert not _is_public_docs_request(
        _Request("GET", _Url("/api/docs/oauth2-redirect"))
    )
