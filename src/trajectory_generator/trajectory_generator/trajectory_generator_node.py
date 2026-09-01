#!/usr/bin/env python3

"""Production trajectory generator for the DYX 4WD marking rover.

Runtime mission source:

    /home/flash/rover_ws/missions/mission.csv

Mission settings source:

    /home/flash/.local/share/dyx_rover/mission_metadata.json

Responsibilities:

- Read the single active mission.csv.
- Preserve the exact uploaded marking-point order.
- Convert PX4-origin GPS candidates through explicit NED, then MAVROS ENU.
- Generate navigation interpolation at the configured spacing.
- Generate dummy alignment points for short row transitions.
- Publish original marking points separately from navigation points.
- Never publish movement commands.
- Never start the mission.

Extension behaviour:

ENABLE:
    For every pair of consecutive original marking points, calculate
    the distance.

    When the distance is below row_transition_threshold_m, use the
    following original marking point to determine the new-row direction.

    Example:

        1 -> 2 -> 3
                  \
                   \
        6 <- 5 <- 4 <- Dummy

    Actual navigation order:

        1 -> 2 -> 3 -> Dummy -> 4 -> 5 -> 6

DISABLE:
    No dummy points are generated.
    Only original CSV coordinates remain marking points.
"""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os
import struct
import threading

from pathlib import Path as FilePath
from typing import Any

import rclpy

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

from trajectory_generator.localization_frame import GeographicOrigin
from trajectory_generator.localization_frame import project_geodetic_to_px4_ned
from trajectory_generator.localization_frame import transform_ned_to_enu


class TrajectoryGenerator(Node):
    """Prepare and publish one active rover mission."""

    WGS84_A_M = 6_378_137.0
    WGS84_F = 1.0 / 298.257223563

    CONTROL_HZ = 5.0

    POINT_TYPE_PASS_THROUGH = 0
    POINT_TYPE_DUMMY_ALIGNMENT = 1
    POINT_TYPE_MARKING = 2

    LATITUDE_HEADERS = (
        "latitude",
        "lat",
        "gps_lat",
        "gps_latitude",
    )

    LONGITUDE_HEADERS = (
        "longitude",
        "lon",
        "lng",
        "long",
        "gps_lon",
        "gps_longitude",
    )

    VALID_EXTENSION_MODES = {
        "ENABLE",
        "DISABLE",
    }

    VALID_LOCALIZATION_MODES = {
        "legacy",
        "px4_origin",
        "shadow",
    }

    def __init__(self) -> None:
        super().__init__("trajectory_generator")

        # ======================================================
        # Parameters
        # ======================================================

        self.declare_parameter(
            "mission_file",
            ("/home/flash/rover_ws/" "missions/mission.csv"),
        )

        self.declare_parameter(
            "mission_metadata_file",
            ("/home/flash/.local/share/" "dyx_rover/mission_metadata.json"),
        )

        self.declare_parameter(
            "frame_id",
            "map",
        )

        self.declare_parameter(
            "global_position_topic",
            "/mavros/global_position/raw/fix",
        )

        self.declare_parameter(
            "gp_origin_topic",
            "/mavros/global_position/gp_origin",
        )

        self.declare_parameter(
            "fused_global_position_topic",
            "/mavros/global_position/global",
        )

        self.declare_parameter(
            "localization_mode",
            "shadow",
        )

        self.declare_parameter(
            "local_odom_topic",
            "/mavros/local_position/odom",
        )

        self.declare_parameter(
            "gps_status_topic",
            "/mavros/gpsstatus/gps1/raw",
        )

        self.declare_parameter(
            "rtk_health_topic",
            "/rtk_correction_bridge/healthy",
        )

        self.declare_parameter(
            "rtk_correction_age_topic",
            ("/rtk_correction_bridge/" "correction_age_sec"),
        )

        self.declare_parameter(
            "required_gps_fix_type",
            6,
        )

        self.declare_parameter(
            "rtk_stable_sec",
            3.0,
        )

        self.declare_parameter(
            "max_correction_age_sec",
            2.0,
        )

        self.declare_parameter(
            "reference_timeout_sec",
            1.0,
        )

        self.declare_parameter(
            "max_reference_skew_sec",
            0.25,
        )

        self.declare_parameter(
            "max_target_distance_m",
            1000.0,
        )

        self.declare_parameter(
            "max_abs_coordinate_m",
            10000.0,
        )

        self.declare_parameter(
            "maximum_marking_points",
            10000,
        )

        self.declare_parameter(
            "maximum_navigation_points",
            200000,
        )

        self.declare_parameter(
            "interpolation_spacing_m",
            0.05,
        )

        self.declare_parameter(
            "minimum_segment_length_m",
            0.001,
        )

        self.declare_parameter(
            "minimum_dummy_clearance_m",
            0.05,
        )

        self.mission_file = FilePath(
            os.path.abspath(
                os.path.expanduser(str(self.get_parameter("mission_file").value))
            )
        )

        self.metadata_file = FilePath(
            os.path.abspath(
                os.path.expanduser(
                    str(self.get_parameter("mission_metadata_file").value)
                )
            )
        )

        self.frame_id = str(self.get_parameter("frame_id").value).strip()

        self.global_position_topic = str(
            self.get_parameter("global_position_topic").value
        ).strip()

        self.gp_origin_topic = str(
            self.get_parameter("gp_origin_topic").value
        ).strip()

        self.fused_global_position_topic = str(
            self.get_parameter("fused_global_position_topic").value
        ).strip()

        self.localization_mode = str(
            self.get_parameter("localization_mode").value
        ).strip().lower()

        self.local_odom_topic = str(
            self.get_parameter("local_odom_topic").value
        ).strip()

        self.gps_status_topic = str(
            self.get_parameter("gps_status_topic").value
        ).strip()

        self.rtk_health_topic = str(
            self.get_parameter("rtk_health_topic").value
        ).strip()

        self.rtk_correction_age_topic = str(
            self.get_parameter("rtk_correction_age_topic").value
        ).strip()

        self.required_gps_fix_type = int(
            self.get_parameter("required_gps_fix_type").value
        )

        self.rtk_stable_sec = float(self.get_parameter("rtk_stable_sec").value)

        self.max_correction_age_sec = float(
            self.get_parameter("max_correction_age_sec").value
        )

        self.reference_timeout_sec = float(
            self.get_parameter("reference_timeout_sec").value
        )

        self.max_reference_skew_sec = float(
            self.get_parameter("max_reference_skew_sec").value
        )

        self.max_target_distance_m = float(
            self.get_parameter("max_target_distance_m").value
        )

        self.max_abs_coordinate_m = float(
            self.get_parameter("max_abs_coordinate_m").value
        )

        self.maximum_marking_points = int(
            self.get_parameter("maximum_marking_points").value
        )

        self.maximum_navigation_points = int(
            self.get_parameter("maximum_navigation_points").value
        )

        self.interpolation_spacing_m = float(
            self.get_parameter("interpolation_spacing_m").value
        )

        self.minimum_segment_length_m = float(
            self.get_parameter("minimum_segment_length_m").value
        )

        self.minimum_dummy_clearance_m = float(
            self.get_parameter("minimum_dummy_clearance_m").value
        )

        self._validate_parameters()

        # ======================================================
        # QoS
        # ======================================================

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ======================================================
        # Publishers
        # ======================================================

        self.mission_waypoints_pub = self.create_publisher(
            NavPath,
            "/mission_waypoints",
            retained_qos,
        )

        self.nav_path_pub = self.create_publisher(
            NavPath,
            "/nav_path",
            retained_qos,
        )

        self.path_types_pub = self.create_publisher(
            UInt8MultiArray,
            ("/trajectory_generator/" "path_types"),
            retained_qos,
        )

        self.marking_indices_pub = self.create_publisher(
            Int32MultiArray,
            ("/trajectory_generator/" "marking_indices"),
            retained_qos,
        )

        self.path_signature_pub = self.create_publisher(
            String,
            "/trajectory_generator/path_signature",
            retained_qos,
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "/trajectory_generator/ready",
            retained_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/trajectory_generator/status",
            retained_qos,
        )

        # ======================================================
        # Subscribers
        # ======================================================

        self.create_subscription(
            NavSatFix,
            self.global_position_topic,
            self._global_position_callback,
            sensor_qos,
        )

        self.create_subscription(
            GeoPointStamped,
            self.gp_origin_topic,
            self._gp_origin_callback,
            retained_qos,
        )

        self.create_subscription(
            NavSatFix,
            self.fused_global_position_topic,
            self._fused_global_position_callback,
            sensor_qos,
        )

        self.create_subscription(
            Odometry,
            self.local_odom_topic,
            self._local_odom_callback,
            sensor_qos,
        )

        self.create_subscription(
            GPSRAW,
            self.gps_status_topic,
            self._gps_status_callback,
            sensor_qos,
        )

        self.create_subscription(
            Bool,
            self.rtk_health_topic,
            self._rtk_health_callback,
            sensor_qos,
        )

        self.create_subscription(
            Float32,
            self.rtk_correction_age_topic,
            self._rtk_correction_age_callback,
            sensor_qos,
        )

        # ======================================================
        # Services
        # ======================================================

        self.create_service(
            Trigger,
            "/trajectory_generator/prepare",
            self._prepare_callback,
        )

        self.create_service(
            Trigger,
            "/trajectory_generator/clear",
            self._clear_callback,
        )

        # ======================================================
        # Runtime state
        # ======================================================

        self._lock = threading.RLock()

        self.latest_global_fix: NavSatFix | None = None

        self.latest_global_time = None

        self.latest_gp_origin: GeoPointStamped | None = None

        self.latest_fused_global_fix: NavSatFix | None = None

        self.latest_fused_global_time = None

        self.latest_local_odom: Odometry | None = None

        self.latest_local_time = None

        self.latest_gps_status: GPSRAW | None = None

        self.latest_gps_status_time = None

        self.localization_shadow_summary: dict[str, Any] = {
            "mode": self.localization_mode,
            "candidate_available": False,
            "reason": "not evaluated",
        }

        self.rtk_healthy = False
        self.correction_age_sec = math.inf
        self.rtk_ready_since = None

        self.prepare_requested = False
        self.preparing = False
        self.ready = False

        self.raw_coordinate_mode: str | None = None

        self.raw_marking_points: list[tuple[float, float]] = []

        self.extension_mode: str | None = None

        self.dummy_point_distance_m: float | None = None

        self.row_transition_threshold_m: float | None = None

        self.mission_id: str | None = None
        self.mission_checksum: str | None = None

        self.prepared_marking_points: list[tuple[float, float]] = []

        self.prepared_navigation_points: list[tuple[float, float]] = []

        self.prepared_path_types: list[int] = []

        self.prepared_marking_indices: list[int] = []

        self.prepared_path_signature: str | None = None

        self.last_error: str | None = None
        self.last_wait_log_time = self.get_clock().now()

        self._publish_ready(False)
        self._publish_empty_outputs()

        self._publish_status(
            state="IDLE",
            message=("Waiting for mission upload " "and prepare request"),
        )

        self.control_timer = self.create_timer(
            1.0 / self.CONTROL_HZ,
            self._control_loop,
        )

        self.get_logger().warn("===== TRAJECTORY GENERATOR STARTED =====")

        self.get_logger().warn(f"Mission file  : {self.mission_file}")

        self.get_logger().warn(f"Metadata file : {self.metadata_file}")

        self.get_logger().warn(f"Localization  : {self.localization_mode}")

        self.get_logger().warn("Prepare       : " "/trajectory_generator/prepare")

        self.get_logger().warn("Clear         : " "/trajectory_generator/clear")

    # ==========================================================
    # Parameter validation
    # ==========================================================

    def _validate_parameters(
        self,
    ) -> None:
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")

        topics = {
            "global_position_topic": self.global_position_topic,
            "gp_origin_topic": self.gp_origin_topic,
            "fused_global_position_topic": self.fused_global_position_topic,
            "local_odom_topic": self.local_odom_topic,
            "gps_status_topic": self.gps_status_topic,
            "rtk_health_topic": self.rtk_health_topic,
            "rtk_correction_age_topic": self.rtk_correction_age_topic,
        }

        for name, value in topics.items():
            if not value:
                raise ValueError(f"{name} must not be empty")

        if self.localization_mode not in self.VALID_LOCALIZATION_MODES:
            allowed = ", ".join(sorted(self.VALID_LOCALIZATION_MODES))
            raise ValueError(
                f"localization_mode must be one of: {allowed}"
            )

        if self.required_gps_fix_type < 0:
            raise ValueError("required_gps_fix_type must be >= 0")

        if self.maximum_marking_points < 2:
            raise ValueError("maximum_marking_points must be >= 2")

        if self.maximum_navigation_points < self.maximum_marking_points:
            raise ValueError(
                "maximum_navigation_points must " "be >= maximum_marking_points"
            )

        positive_values = {
            "rtk_stable_sec": (self.rtk_stable_sec),
            "max_correction_age_sec": (self.max_correction_age_sec),
            "reference_timeout_sec": (self.reference_timeout_sec),
            "max_reference_skew_sec": (self.max_reference_skew_sec),
            "max_target_distance_m": (self.max_target_distance_m),
            "max_abs_coordinate_m": (self.max_abs_coordinate_m),
            "interpolation_spacing_m": (self.interpolation_spacing_m),
            "minimum_segment_length_m": (self.minimum_segment_length_m),
            "minimum_dummy_clearance_m": (self.minimum_dummy_clearance_m),
        }

        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")

    # ==========================================================
    # ROS input callbacks
    # ==========================================================

    def _global_position_callback(
        self,
        message: NavSatFix,
    ) -> None:
        with self._lock:
            self.latest_global_fix = message
            self.latest_global_time = self.get_clock().now()

    def _gp_origin_callback(
        self,
        message: GeoPointStamped,
    ) -> None:
        latitude = float(message.position.latitude)
        longitude = float(message.position.longitude)

        if (
            not math.isfinite(latitude)
            or not math.isfinite(longitude)
            or abs(latitude) > 90.0
            or abs(longitude) > 180.0
            or (latitude == 0.0 and longitude == 0.0)
        ):
            self.get_logger().error(
                "Ignoring invalid PX4 global origin: "
                f"lat={latitude}, lon={longitude}"
            )

            return

        with self._lock:
            previous = self.latest_gp_origin
            self.latest_gp_origin = message

        if previous is None:
            self.get_logger().warn(
                "PX4 global origin received for shadow diagnostics: "
                f"lat={latitude:.7f}, lon={longitude:.7f}"
            )

        elif (
            float(previous.position.latitude) != latitude
            or float(previous.position.longitude) != longitude
        ):
            self.get_logger().error(
                "PX4 global origin changed during shadow observation: "
                f"lat={latitude:.7f}, lon={longitude:.7f}; "
                "legacy coordinates remain authoritative"
            )

    def _fused_global_position_callback(
        self,
        message: NavSatFix,
    ) -> None:
        with self._lock:
            self.latest_fused_global_fix = message
            self.latest_fused_global_time = self.get_clock().now()

    def _local_odom_callback(
        self,
        message: Odometry,
    ) -> None:
        with self._lock:
            self.latest_local_odom = message
            self.latest_local_time = self.get_clock().now()

    def _gps_status_callback(
        self,
        message: GPSRAW,
    ) -> None:
        with self._lock:
            self.latest_gps_status = message
            self.latest_gps_status_time = self.get_clock().now()

    def _rtk_health_callback(
        self,
        message: Bool,
    ) -> None:
        with self._lock:
            self.rtk_healthy = bool(message.data)

    def _rtk_correction_age_callback(
        self,
        message: Float32,
    ) -> None:
        value = float(message.data)

        with self._lock:
            self.correction_age_sec = value if math.isfinite(value) else math.inf

    # ==========================================================
    # Service callbacks
    # ==========================================================

    def _prepare_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Load mission.csv and request path preparation."""

        with self._lock:
            try:
                (
                    metadata,
                    coordinate_mode,
                    raw_points,
                ) = self._load_mission_source()

                self._clear_prepared_state(publish_empty=True)

                self.raw_coordinate_mode = coordinate_mode

                self.raw_marking_points = raw_points

                self.extension_mode = str(metadata["extension_mode"]).strip().upper()

                self.row_transition_threshold_m = float(
                    metadata["row_transition_threshold_m"]
                )

                dummy_value = metadata.get("dummy_point_distance_m")

                self.dummy_point_distance_m = (
                    float(dummy_value) if dummy_value is not None else None
                )

                self.mission_id = str(metadata["mission_id"])

                self.mission_checksum = str(metadata["checksum_sha256"])

                self.prepare_requested = True
                self.preparing = True
                self.ready = False
                self.last_error = None
                self.rtk_ready_since = None

                self._publish_ready(False)

                self._publish_status(
                    state="PREPARING",
                    message=("Mission accepted for " "trajectory preparation"),
                )

                response.success = True

                response.message = (
                    "Preparation accepted: "
                    f"{len(raw_points)} markings, "
                    f"mode={self.extension_mode}"
                )

                self.get_logger().warn("===== PREPARE REQUEST ACCEPTED =====")

                self.get_logger().warn(f"Mission ID      : " f"{self.mission_id}")

                self.get_logger().warn(f"Marking points  : " f"{len(raw_points)}")

                self.get_logger().warn(f"Coordinate mode : " f"{coordinate_mode}")

                self.get_logger().warn(f"Extension mode  : " f"{self.extension_mode}")

                if self.extension_mode == "ENABLE":
                    self.get_logger().warn(
                        "Dummy distance  : " f"{self.dummy_point_distance_m:.3f} m"
                    )

                    self.get_logger().warn(
                        "Transition limit: " f"{self.row_transition_threshold_m:.3f} m"
                    )

            except (
                OSError,
                ValueError,
                KeyError,
                json.JSONDecodeError,
            ) as error:
                self._set_error(str(error))

                response.success = False
                response.message = str(error)

        return response

    def _clear_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Clear generated paths but retain mission.csv."""

        with self._lock:
            self._reset_all_runtime(publish_empty=True)

            self._publish_status(
                state="IDLE",
                message=("Generated trajectory cleared; " "mission.csv retained"),
            )

        response.success = True
        response.message = "Generated trajectory cleared; " "mission.csv retained"

        return response

    # ==========================================================
    # Mission and metadata loading
    # ==========================================================

    @staticmethod
    def _find_header(
        headers: dict[str, str],
        aliases: tuple[str, ...],
    ) -> str | None:
        for alias in aliases:
            if alias in headers:
                return headers[alias]

        return None

    @staticmethod
    def _sha256(
        data: bytes,
    ) -> str:
        return hashlib.sha256(data).hexdigest()

    def _load_mission_source(
        self,
    ) -> tuple[
        dict[str, Any],
        str,
        list[tuple[float, float]],
    ]:
        if not self.mission_file.is_file():
            raise ValueError("mission.csv does not exist")

        if not self.metadata_file.is_file():
            raise ValueError("Mission metadata does not exist")

        metadata = json.loads(self.metadata_file.read_text(encoding="utf-8"))

        if not isinstance(
            metadata,
            dict,
        ):
            raise ValueError("Mission metadata must be " "a JSON object")

        mission_bytes = self.mission_file.read_bytes()

        expected_checksum = str(
            metadata.get(
                "checksum_sha256",
                "",
            )
        ).strip()

        actual_checksum = self._sha256(mission_bytes)

        if not expected_checksum or expected_checksum != actual_checksum:
            raise ValueError("mission.csv checksum does not " "match mission metadata")

        coordinate_mode = (
            str(
                metadata.get(
                    "coordinate_mode",
                    "",
                )
            )
            .strip()
            .lower()
        )

        if coordinate_mode not in {
            "gps",
            "local",
        }:
            raise ValueError("Mission coordinate mode must " "be gps or local")

        extension_mode = (
            str(
                metadata.get(
                    "extension_mode",
                    "",
                )
            )
            .strip()
            .upper()
        )

        if extension_mode not in self.VALID_EXTENSION_MODES:
            raise ValueError("Mission extension mode must " "be ENABLE or DISABLE")

        try:
            transition_threshold = float(metadata["row_transition_threshold_m"])

        except (
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            raise ValueError(
                "Mission transition threshold " "is missing or invalid"
            ) from error

        if not math.isfinite(transition_threshold) or transition_threshold <= 0.0:
            raise ValueError("Mission transition threshold " "must be finite and > 0")

        dummy_value = metadata.get("dummy_point_distance_m")

        if extension_mode == "ENABLE":
            if dummy_value is None:
                raise ValueError("ENABLE requires " "dummy_point_distance_m")

            try:
                dummy_distance = float(dummy_value)

            except (
                TypeError,
                ValueError,
            ) as error:
                raise ValueError("Dummy-point distance " "is invalid") from error

            if not math.isfinite(dummy_distance) or dummy_distance <= 0.0:
                raise ValueError("Dummy-point distance must " "be finite and > 0")

        csv_text = mission_bytes.decode("utf-8-sig")

        points = self._read_csv_points(
            csv_text=csv_text,
            coordinate_mode=coordinate_mode,
        )

        try:
            expected_total = int(metadata["total_points"])

        except (
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            raise ValueError("Mission total_points is invalid") from error

        if expected_total != len(points):
            raise ValueError(
                "mission.csv point count does " "not match mission metadata"
            )

        return (
            metadata,
            coordinate_mode,
            points,
        )

    def _read_csv_points(
        self,
        *,
        csv_text: str,
        coordinate_mode: str,
    ) -> list[tuple[float, float]]:
        reader = csv.DictReader(
            io.StringIO(
                csv_text,
                newline="",
            )
        )

        if reader.fieldnames is None:
            raise ValueError("mission.csv has no header row")

        headers = {
            str(name).strip().lower(): str(name).strip()
            for name in reader.fieldnames
            if (name is not None and str(name).strip())
        }

        if coordinate_mode == "gps":
            first_column = self._find_header(
                headers,
                self.LATITUDE_HEADERS,
            )

            second_column = self._find_header(
                headers,
                self.LONGITUDE_HEADERS,
            )

            if first_column is None or second_column is None:
                raise ValueError("GPS mission requires " "latitude,longitude columns")

        else:
            first_column = headers.get("x")

            second_column = headers.get("y")

            if first_column is None or second_column is None:
                raise ValueError("Local mission requires " "x,y columns")

        points: list[tuple[float, float]] = []

        point_set: set[tuple[float, float]] = set()

        for row_number, row in enumerate(
            reader,
            start=2,
        ):
            if not any(
                value is not None and str(value).strip() for value in row.values()
            ):
                continue

            try:
                first_value = float(str(row[first_column]).strip())

                second_value = float(str(row[second_column]).strip())

            except (
                KeyError,
                TypeError,
                ValueError,
            ) as error:
                raise ValueError(
                    "Invalid coordinate at " f"CSV row {row_number}"
                ) from error

            if not all(
                math.isfinite(value)
                for value in (
                    first_value,
                    second_value,
                )
            ):
                raise ValueError("Non-finite coordinate at " f"CSV row {row_number}")

            if coordinate_mode == "gps":
                if not (-90.0 <= first_value <= 90.0):
                    raise ValueError(
                        "Latitude outside range " f"at CSV row {row_number}"
                    )

                if not (-180.0 <= second_value <= 180.0):
                    raise ValueError(
                        "Longitude outside range " f"at CSV row {row_number}"
                    )

            elif (
                abs(first_value) > self.max_abs_coordinate_m
                or abs(second_value) > self.max_abs_coordinate_m
            ):
                raise ValueError(
                    "Local coordinate outside "
                    "allowed range at "
                    f"CSV row {row_number}"
                )

            point = (
                first_value,
                second_value,
            )

            if point in point_set:
                raise ValueError("Duplicate marking point at " f"CSV row {row_number}")

            point_set.add(point)

            points.append(point)

            if len(points) > self.maximum_marking_points:
                raise ValueError("Mission exceeds maximum " "marking-point count")

        if len(points) < 2:
            raise ValueError("Mission requires at least " "two marking points")

        return points

    # ==========================================================
    # RTK and reference validation
    # ==========================================================

    def _age_seconds(
        self,
        timestamp: Any,
    ) -> float:
        if timestamp is None:
            return math.inf

        return (self.get_clock().now() - timestamp).nanoseconds / 1e9

    def _rover_start_is_ready(
        self,
    ) -> tuple[bool, str]:
        """Check that current local odometry can be used as path start."""

        if self.latest_local_odom is None:
            return (
                False,
                "No local odometry for rover start",
            )

        if self._age_seconds(self.latest_local_time) > self.reference_timeout_sec:
            return (
                False,
                "Local odometry for rover start is stale",
            )

        start_east = float(self.latest_local_odom.pose.pose.position.x)

        start_north = float(self.latest_local_odom.pose.pose.position.y)

        if not all(
            math.isfinite(value)
            for value in (
                start_east,
                start_north,
            )
        ):
            return (
                False,
                "Rover start position is non-finite",
            )

        return (
            True,
            "Rover start position ready",
        )

    def _current_rover_start_point(
        self,
    ) -> tuple[float, float]:
        ready, reason = self._rover_start_is_ready()

        if not ready:
            raise ValueError(reason)

        assert self.latest_local_odom is not None

        return (
            float(self.latest_local_odom.pose.pose.position.x),
            float(self.latest_local_odom.pose.pose.position.y),
        )

    def _reference_is_ready(
        self,
    ) -> tuple[bool, str]:
        if self.latest_global_fix is None:
            return (
                False,
                "No global GPS fix",
            )

        if self.latest_local_odom is None:
            return (
                False,
                "No local odometry",
            )

        if self.latest_gps_status is None:
            return (
                False,
                "No GPS status",
            )

        if self._age_seconds(self.latest_global_time) > self.reference_timeout_sec:
            return (
                False,
                "Global GPS fix is stale",
            )

        if self._age_seconds(self.latest_local_time) > self.reference_timeout_sec:
            return (
                False,
                "Local odometry is stale",
            )

        if self._age_seconds(self.latest_gps_status_time) > self.reference_timeout_sec:
            return (
                False,
                "GPS status is stale",
            )

        if int(self.latest_gps_status.fix_type) < self.required_gps_fix_type:
            return (
                False,
                (
                    "RTK FIXED required; "
                    f"fix_type="
                    f"{self.latest_gps_status.fix_type}"
                ),
            )

        if not self.rtk_healthy:
            return (
                False,
                "RTK correction bridge unhealthy",
            )

        if (
            not math.isfinite(self.correction_age_sec)
            or self.correction_age_sec > self.max_correction_age_sec
        ):
            return (
                False,
                ("RTK correction age " f"{self.correction_age_sec:.2f}s"),
            )

        reference_skew = (
            abs((self.latest_global_time - self.latest_local_time).nanoseconds) / 1e9
        )

        if reference_skew > self.max_reference_skew_sec:
            return (
                False,
                ("GPS/local timestamp skew " f"{reference_skew:.3f}s"),
            )

        values = (
            float(self.latest_global_fix.latitude),
            float(self.latest_global_fix.longitude),
            float(self.latest_global_fix.altitude),
            float(self.latest_local_odom.pose.pose.position.x),
            float(self.latest_local_odom.pose.pose.position.y),
        )

        if not all(math.isfinite(value) for value in values):
            return (
                False,
                "GPS/local reference is non-finite",
            )

        return (
            True,
            "Reference ready",
        )

    # ==========================================================
    # GPS conversion
    # ==========================================================

    @classmethod
    def _geodetic_to_ecef(
        cls,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
    ) -> tuple[float, float, float]:
        latitude = math.radians(latitude_deg)

        longitude = math.radians(longitude_deg)

        eccentricity_sq = cls.WGS84_F * (2.0 - cls.WGS84_F)

        sin_latitude = math.sin(latitude)

        cos_latitude = math.cos(latitude)

        sin_longitude = math.sin(longitude)

        cos_longitude = math.cos(longitude)

        prime_vertical = cls.WGS84_A_M / math.sqrt(
            1.0 - eccentricity_sq * sin_latitude * sin_latitude
        )

        x = (prime_vertical + altitude_m) * cos_latitude * cos_longitude

        y = (prime_vertical + altitude_m) * cos_latitude * sin_longitude

        z = (prime_vertical * (1.0 - eccentricity_sq) + altitude_m) * sin_latitude

        return (
            x,
            y,
            z,
        )

    @classmethod
    def _geodetic_delta_to_enu(
        cls,
        *,
        target_latitude: float,
        target_longitude: float,
        reference_latitude: float,
        reference_longitude: float,
        reference_altitude: float,
    ) -> tuple[float, float]:
        (
            reference_x,
            reference_y,
            reference_z,
        ) = cls._geodetic_to_ecef(
            reference_latitude,
            reference_longitude,
            reference_altitude,
        )

        (
            target_x,
            target_y,
            target_z,
        ) = cls._geodetic_to_ecef(
            target_latitude,
            target_longitude,
            reference_altitude,
        )

        delta_x = target_x - reference_x

        delta_y = target_y - reference_y

        delta_z = target_z - reference_z

        latitude = math.radians(reference_latitude)

        longitude = math.radians(reference_longitude)

        sin_latitude = math.sin(latitude)

        cos_latitude = math.cos(latitude)

        sin_longitude = math.sin(longitude)

        cos_longitude = math.cos(longitude)

        east = -sin_longitude * delta_x + cos_longitude * delta_y

        north = (
            -sin_latitude * cos_longitude * delta_x
            - sin_latitude * sin_longitude * delta_y
            + cos_latitude * delta_z
        )

        return (
            east,
            north,
        )

    def _convert_markings_legacy(
        self,
    ) -> list[tuple[float, float]]:
        if self.raw_coordinate_mode == "local":
            return list(self.raw_marking_points)

        if self.latest_global_fix is None or self.latest_local_odom is None:
            raise ValueError("GPS/local reference unavailable")

        reference_latitude = float(self.latest_global_fix.latitude)

        reference_longitude = float(self.latest_global_fix.longitude)

        reference_altitude = float(self.latest_global_fix.altitude)

        reference_east = float(self.latest_local_odom.pose.pose.position.x)

        reference_north = float(self.latest_local_odom.pose.pose.position.y)

        local_points: list[tuple[float, float]] = []

        self.get_logger().warn(
            "GPS/local reference locked: "
            f"lat={reference_latitude:.9f}, "
            f"lon={reference_longitude:.9f}, "
            f"E={reference_east:.3f}, "
            f"N={reference_north:.3f}"
        )

        for index, (
            latitude,
            longitude,
        ) in enumerate(
            self.raw_marking_points,
            start=1,
        ):
            (
                east_offset,
                north_offset,
            ) = self._geodetic_delta_to_enu(
                target_latitude=latitude,
                target_longitude=longitude,
                reference_latitude=(reference_latitude),
                reference_longitude=(reference_longitude),
                reference_altitude=(reference_altitude),
            )

            target_distance = math.hypot(
                east_offset,
                north_offset,
            )

            if target_distance > self.max_target_distance_m:
                raise ValueError(
                    f"Marking point {index} is "
                    f"{target_distance:.1f} m from "
                    "the local reference"
                )

            point = (
                reference_east + east_offset,
                reference_north + north_offset,
            )

            local_points.append(point)

            self.get_logger().warn(
                f"MARKING {index}: " f"E={point[0]:.3f}, " f"N={point[1]:.3f}"
            )

        return local_points

    def _convert_markings_px4_origin(
        self,
    ) -> list[tuple[float, float]]:
        if self.latest_gp_origin is None:
            raise ValueError("PX4 global origin unavailable")

        origin_position = self.latest_gp_origin.position

        origin = GeographicOrigin(
            latitude_deg=float(origin_position.latitude),
            longitude_deg=float(origin_position.longitude),
            altitude_m=float(origin_position.altitude),
        )

        candidate_points: list[tuple[float, float]] = []

        for index, (
            latitude,
            longitude,
        ) in enumerate(
            self.raw_marking_points,
            start=1,
        ):
            ned = project_geodetic_to_px4_ned(
                origin,
                latitude,
                longitude,
            )
            enu = transform_ned_to_enu(ned)

            target_distance = math.hypot(
                enu.east_m,
                enu.north_m,
            )

            if target_distance > self.max_target_distance_m:
                raise ValueError(
                    f"PX4 candidate marking {index} is "
                    f"{target_distance:.1f} m from the estimator origin"
                )

            candidate_points.append(
                (
                    enu.east_m,
                    enu.north_m,
                )
            )

            self.get_logger().warn(
                f"PX4 NED CANDIDATE P{index}: "
                f"NED(N={ned.north_m:.6f}, E={ned.east_m:.6f}) -> "
                f"ENU(E={enu.east_m:.6f}, N={enu.north_m:.6f})"
            )

        return candidate_points

    def _log_localization_shadow(
        self,
        legacy_points: list[tuple[float, float]],
        candidate_points: list[tuple[float, float]],
    ) -> None:
        assert self.latest_gp_origin is not None

        origin = self.latest_gp_origin.position

        radial_deltas = [
            math.hypot(
                candidate[0] - legacy[0],
                candidate[1] - legacy[1],
            )
            for legacy, candidate in zip(
                legacy_points,
                candidate_points,
            )
        ]

        self.localization_shadow_summary = {
            "mode": self.localization_mode,
            "candidate_available": True,
            "reason": None,
            "origin_latitude_deg": float(origin.latitude),
            "origin_longitude_deg": float(origin.longitude),
            "point_count": len(candidate_points),
            "maximum_legacy_candidate_delta_m": max(radial_deltas),
            "mean_legacy_candidate_delta_m": (
                sum(radial_deltas) / len(radial_deltas)
            ),
            "frame_residual_m": None,
            "frame_receive_skew_sec": None,
        }

        self.get_logger().warn(
            "LOCALIZATION SHADOW: legacy remains authoritative; "
            f"gp_origin=({origin.latitude:.7f}, {origin.longitude:.7f})"
        )

        for index, (legacy, candidate) in enumerate(
            zip(legacy_points, candidate_points),
            start=1,
        ):
            delta_east = candidate[0] - legacy[0]
            delta_north = candidate[1] - legacy[1]
            radial_delta = math.hypot(delta_east, delta_north)

            self.get_logger().warn(
                f"LOCALIZATION SHADOW P{index}: "
                f"legacy=({legacy[0]:.6f}, {legacy[1]:.6f}) m, "
                f"candidate=({candidate[0]:.6f}, "
                f"{candidate[1]:.6f}) m, "
                f"delta=({delta_east:+.6f}, "
                f"{delta_north:+.6f}) m, "
                f"radial={radial_delta:.6f} m"
            )

        self._log_frame_consistency_shadow()

    def _log_frame_consistency_shadow(
        self,
    ) -> None:
        if (
            self.latest_gp_origin is None
            or self.latest_fused_global_fix is None
            or self.latest_local_odom is None
            or self.latest_fused_global_time is None
            or self.latest_local_time is None
        ):
            self.localization_shadow_summary["frame_reason"] = (
                "fused-global/local sample unavailable"
            )

            self.get_logger().warn(
                "LOCALIZATION SHADOW: fused-global/local residual unavailable"
            )

            return

        origin_position = self.latest_gp_origin.position
        fused_position = self.latest_fused_global_fix

        try:
            ned = project_geodetic_to_px4_ned(
                GeographicOrigin(
                    latitude_deg=float(origin_position.latitude),
                    longitude_deg=float(origin_position.longitude),
                    altitude_m=float(origin_position.altitude),
                ),
                float(fused_position.latitude),
                float(fused_position.longitude),
            )
            enu = transform_ned_to_enu(ned)
        except ValueError as error:
            self.localization_shadow_summary["frame_reason"] = str(error)

            self.get_logger().warn(
                "LOCALIZATION SHADOW: fused-global projection rejected: "
                f"{error}"
            )

            return

        local_east = float(self.latest_local_odom.pose.pose.position.x)
        local_north = float(self.latest_local_odom.pose.pose.position.y)
        delta_east = local_east - enu.east_m
        delta_north = local_north - enu.north_m
        radial_delta = math.hypot(delta_east, delta_north)
        receive_skew = abs(
            (
                self.latest_fused_global_time
                - self.latest_local_time
            ).nanoseconds
        ) / 1e9

        self.localization_shadow_summary.update(
            {
                "frame_reason": None,
                "frame_delta_east_m": delta_east,
                "frame_delta_north_m": delta_north,
                "frame_residual_m": radial_delta,
                "frame_receive_skew_sec": receive_skew,
            }
        )

        self.get_logger().warn(
            "LOCALIZATION SHADOW FRAME: "
            f"NED=({ned.north_m:.6f}, {ned.east_m:.6f}) m -> "
            f"ENU=({enu.east_m:.6f}, {enu.north_m:.6f}) m, "
            f"odom=({local_east:.6f}, {local_north:.6f}) m, "
            f"delta=({delta_east:+.6f}, "
            f"{delta_north:+.6f}) m, "
            f"radial={radial_delta:.6f} m, "
            f"receive_skew={receive_skew:.3f} s"
        )

    def _convert_markings_to_local(
        self,
    ) -> list[tuple[float, float]]:
        if self.raw_coordinate_mode == "local":
            self.localization_shadow_summary = {
                "mode": self.localization_mode,
                "candidate_available": False,
                "reason": "mission already uses local coordinates",
            }

            return list(self.raw_marking_points)

        legacy_points = self._convert_markings_legacy()

        if self.localization_mode == "legacy":
            self.localization_shadow_summary = {
                "mode": self.localization_mode,
                "candidate_available": False,
                "reason": "shadow calculation disabled",
            }

            return legacy_points

        if self.latest_gp_origin is None:
            self.localization_shadow_summary = {
                "mode": self.localization_mode,
                "candidate_available": False,
                "reason": "PX4 global origin unavailable",
            }

            self.get_logger().warn(
                "LOCALIZATION SHADOW SKIPPED: PX4 global origin unavailable; "
                "legacy coordinates remain authoritative"
            )

            return legacy_points

        try:
            candidate_points = self._convert_markings_px4_origin()
        except ValueError as error:
            self.localization_shadow_summary = {
                "mode": self.localization_mode,
                "candidate_available": False,
                "reason": str(error),
            }

            self.get_logger().warn(
                "LOCALIZATION SHADOW SKIPPED: PX4 candidate rejected: "
                f"{error}; legacy coordinates remain authoritative"
            )

            return legacy_points

        try:
            self._log_localization_shadow(
                legacy_points,
                candidate_points,
            )
        except Exception as error:  # noqa: BLE001 - diagnostics must never veto legacy authority
            self.get_logger().error(
                "LOCALIZATION SHADOW DIAGNOSTICS FAILED: "
                f"{error}; legacy coordinates remain authoritative"
            )

        return legacy_points

    # ==========================================================
    # Path generation
    # ==========================================================

    @staticmethod
    def _distance(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return math.hypot(
            second[0] - first[0],
            second[1] - first[1],
        )

    def _append_point(
        self,
        points: list[tuple[float, float]],
        point_types: list[int],
        marking_indices: list[int],
        point: tuple[float, float],
        point_type: int,
        marking_index: int = -1,
    ) -> None:
        if points:
            separation = self._distance(
                points[-1],
                point,
            )

            if separation < self.minimum_segment_length_m:
                raise ValueError("Generated consecutive path " "points are too close")

        points.append(point)

        point_types.append(int(point_type))

        marking_indices.append(int(marking_index))

        if len(points) > self.maximum_navigation_points:
            raise ValueError("Generated path exceeds " "maximum_navigation_points")

    def _append_interpolated_segment(
        self,
        points: list[tuple[float, float]],
        point_types: list[int],
        marking_indices: list[int],
        start: tuple[float, float],
        end: tuple[float, float],
        final_type: int,
        final_marking_index: int = -1,
    ) -> None:
        delta_x = end[0] - start[0]

        delta_y = end[1] - start[1]

        segment_length = math.hypot(
            delta_x,
            delta_y,
        )

        if (
            not math.isfinite(segment_length)
            or segment_length < self.minimum_segment_length_m
        ):
            raise ValueError("Cannot generate a zero-length " "navigation segment")

        divisions = max(
            1,
            int(math.ceil(segment_length / self.interpolation_spacing_m)),
        )

        for division in range(
            1,
            divisions + 1,
        ):
            ratio = division / divisions

            generated_point = (
                start[0] + ratio * delta_x,
                start[1] + ratio * delta_y,
            )

            is_final = division == divisions

            self._append_point(
                points=points,
                point_types=point_types,
                marking_indices=(marking_indices),
                point=generated_point,
                point_type=(final_type if is_final else (self.POINT_TYPE_PASS_THROUGH)),
                marking_index=(final_marking_index if is_final else -1),
            )

    def _calculate_dummy_point(
        self,
        *,
        new_row_first: tuple[
            float,
            float,
        ],
        new_row_second: tuple[
            float,
            float,
        ],
    ) -> tuple[float, float]:
        if self.dummy_point_distance_m is None:
            raise ValueError("Dummy-point distance unavailable")

        direction_x = new_row_second[0] - new_row_first[0]

        direction_y = new_row_second[1] - new_row_first[1]

        direction_length = math.hypot(
            direction_x,
            direction_y,
        )

        if direction_length < self.minimum_segment_length_m:
            raise ValueError(
                "Cannot determine new-row " "direction from duplicate points"
            )

        unit_x = direction_x / direction_length

        unit_y = direction_y / direction_length

        return (
            new_row_first[0] - unit_x * self.dummy_point_distance_m,
            new_row_first[1] - unit_y * self.dummy_point_distance_m,
        )

    def _generate_navigation_path(
        self,
        marking_points: list[tuple[float, float]],
        rover_start: tuple[
            float,
            float,
        ],
    ) -> tuple[
        list[tuple[float, float]],
        list[int],
        list[int],
        int,
    ]:
        navigation_points: list[tuple[float, float]] = []

        point_types: list[int] = []
        marking_indices: list[int] = []

        dummy_count = 0

        # The rover's latest local-odometry position becomes the
        # first navigation-only point.
        #
        # It is not a marking point, so it does not affect:
        # - total marking-point count
        # - completed-point count
        # - skipped-point count
        # - mission report
        first_marking = marking_points[0]

        approach_distance = self._distance(
            rover_start,
            first_marking,
        )

        if approach_distance >= self.minimum_segment_length_m:
            # First point is the rover's current position.
            self._append_point(
                points=navigation_points,
                point_types=point_types,
                marking_indices=marking_indices,
                point=rover_start,
                point_type=(self.POINT_TYPE_PASS_THROUGH),
                marking_index=-1,
            )

            # Generate interpolation from rover start to P1.
            # The final point remains the real marking point P1.
            self._append_interpolated_segment(
                points=navigation_points,
                point_types=point_types,
                marking_indices=marking_indices,
                start=rover_start,
                end=first_marking,
                final_type=(self.POINT_TYPE_MARKING),
                final_marking_index=0,
            )

        else:
            # Rover is already effectively at P1.
            # Avoid creating duplicate points.
            self._append_point(
                points=navigation_points,
                point_types=point_types,
                marking_indices=marking_indices,
                point=first_marking,
                point_type=(self.POINT_TYPE_MARKING),
                marking_index=0,
            )

        self.get_logger().warn(
            "ROVER START -> P1: "
            f"start=({rover_start[0]:.3f}, "
            f"{rover_start[1]:.3f}) | "
            f"P1=({first_marking[0]:.3f}, "
            f"{first_marking[1]:.3f}) | "
            f"distance={approach_distance:.3f} m"
        )

        for index in range(len(marking_points) - 1):
            current_marking = marking_points[index]

            next_marking = marking_points[index + 1]

            transition_distance = self._distance(
                current_marking,
                next_marking,
            )

            if transition_distance < self.minimum_segment_length_m:
                raise ValueError(
                    "Consecutive marking points "
                    f"{index + 1} and "
                    f"{index + 2} are too close"
                )

            use_dummy = (
                self.extension_mode == "ENABLE"
                and self.row_transition_threshold_m is not None
                and transition_distance < self.row_transition_threshold_m
            )

            if use_dummy:
                following_index = index + 2

                if following_index >= len(marking_points):
                    raise ValueError(
                        "A short transition was "
                        "detected before the final "
                        "marking point, but there is "
                        "no following point to determine "
                        "the next-row direction"
                    )

                following_marking = marking_points[following_index]

                dummy_point = self._calculate_dummy_point(
                    new_row_first=(next_marking),
                    new_row_second=(following_marking),
                )

                clearance_from_current = self._distance(
                    current_marking,
                    dummy_point,
                )

                if clearance_from_current < self.minimum_dummy_clearance_m:
                    raise ValueError(
                        "Calculated dummy point is "
                        "too close to the previous-row "
                        "endpoint. Change the frontend "
                        "dummy-point distance."
                    )

                # Previous row endpoint -> Dummy
                self._append_interpolated_segment(
                    points=navigation_points,
                    point_types=point_types,
                    marking_indices=(marking_indices),
                    start=current_marking,
                    end=dummy_point,
                    final_type=(self.POINT_TYPE_DUMMY_ALIGNMENT),
                    final_marking_index=-1,
                )

                # Dummy -> first point of new row
                self._append_interpolated_segment(
                    points=navigation_points,
                    point_types=point_types,
                    marking_indices=(marking_indices),
                    start=dummy_point,
                    end=next_marking,
                    final_type=(self.POINT_TYPE_MARKING),
                    final_marking_index=(index + 1),
                )

                dummy_count += 1

                self.get_logger().warn(
                    "ROW TRANSITION: "
                    f"{index + 1} -> "
                    f"{index + 2} | "
                    f"gap="
                    f"{transition_distance:.3f} m | "
                    f"Dummy="
                    f"({dummy_point[0]:.3f}, "
                    f"{dummy_point[1]:.3f}) | "
                    f"direction="
                    f"{index + 2} -> "
                    f"{index + 3}"
                )

            else:
                self._append_interpolated_segment(
                    points=navigation_points,
                    point_types=point_types,
                    marking_indices=(marking_indices),
                    start=current_marking,
                    end=next_marking,
                    final_type=(self.POINT_TYPE_MARKING),
                    final_marking_index=(index + 1),
                )

        if not (len(navigation_points) == len(point_types) == len(marking_indices)):
            raise ValueError("Generated path metadata length " "mismatch")

        published_markings = [
            marking_index for marking_index in marking_indices if marking_index >= 0
        ]

        expected_markings = list(range(len(marking_points)))

        if published_markings != expected_markings:
            raise ValueError(
                "Generated navigation path does "
                "not preserve every original "
                "marking point exactly once"
            )

        return (
            navigation_points,
            point_types,
            marking_indices,
            dummy_count,
        )

    # ==========================================================
    # ROS publication
    # ==========================================================

    def _build_path(
        self,
        points: list[tuple[float, float]],
        stamp: Any,
    ) -> NavPath:
        path = NavPath()

        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        for x, y in points:
            pose = PoseStamped()

            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = float(x)

            pose.pose.position.y = float(y)

            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        return path

    def _publish_ready(
        self,
        ready: bool,
    ) -> None:
        message = Bool()
        message.data = bool(ready)

        self.ready_pub.publish(message)

    def _publish_path_metadata(
        self,
        *,
        point_types: list[int],
        marking_indices: list[int],
    ) -> None:
        type_message = UInt8MultiArray()

        type_message.data = [int(value) for value in point_types]

        self.path_types_pub.publish(type_message)

        index_message = Int32MultiArray()

        index_message.data = [int(value) for value in marking_indices]

        self.marking_indices_pub.publish(index_message)

    def _publish_path_signature(
        self,
        signature: str | None,
    ) -> None:
        message = String()
        message.data = signature or ""
        self.path_signature_pub.publish(message)

    def _publish_empty_outputs(
        self,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        empty_path = self._build_path(
            [],
            stamp,
        )

        self.mission_waypoints_pub.publish(empty_path)

        self.nav_path_pub.publish(empty_path)

        self._publish_path_metadata(
            point_types=[],
            marking_indices=[],
        )

        self._publish_path_signature(None)

    def _publish_status(
        self,
        *,
        state: str,
        message: str,
        dummy_count: int = 0,
    ) -> None:
        status = {
            "state": str(state).upper(),
            "message": message,
            "ready": self.ready,
            "mission_id": (self.mission_id),
            "mission_checksum": (self.mission_checksum),
            "path_signature": self.prepared_path_signature,
            "coordinate_mode": (self.raw_coordinate_mode),
            "extension_mode": (self.extension_mode),
            "dummy_point_distance_m": (self.dummy_point_distance_m),
            "row_transition_threshold_m": (self.row_transition_threshold_m),
            "marking_point_count": len(
                self.prepared_marking_points or self.raw_marking_points
            ),
            "navigation_point_count": len(self.prepared_navigation_points),
            "dummy_point_count": int(dummy_count),
            "interpolation_spacing_m": (self.interpolation_spacing_m),
            "localization": dict(self.localization_shadow_summary),
            "error": self.last_error,
        }

        status_message = String()

        status_message.data = json.dumps(
            status,
            separators=(",", ":"),
            sort_keys=True,
        )

        self.status_pub.publish(status_message)

    @staticmethod
    def _make_signature(
        navigation_points: list[tuple[float, float]],
        marking_points: list[tuple[float, float]],
        point_types: list[int],
        marking_indices: list[int],
    ) -> str:
        digest = hashlib.sha256()

        for label, points in (
            (
                b"NAVIGATION",
                navigation_points,
            ),
            (
                b"MARKINGS",
                marking_points,
            ),
        ):
            digest.update(label)

            for x, y in points:
                digest.update(
                    struct.pack(
                        "!dd",
                        x,
                        y,
                    )
                )

        digest.update(b"TYPES")

        digest.update(bytes(point_types))

        digest.update(b"INDICES")

        for value in marking_indices:
            digest.update(
                struct.pack(
                    "!i",
                    value,
                )
            )

        return digest.hexdigest()

    # ==========================================================
    # Runtime state
    # ==========================================================

    def _clear_prepared_state(
        self,
        *,
        publish_empty: bool,
    ) -> None:
        self.prepare_requested = False
        self.preparing = False
        self.ready = False
        self.rtk_ready_since = None

        self.prepared_marking_points = []
        self.prepared_navigation_points = []
        self.prepared_path_types = []
        self.prepared_marking_indices = []
        self.prepared_path_signature = None

        self.localization_shadow_summary = {
            "mode": self.localization_mode,
            "candidate_available": False,
            "reason": "not evaluated",
        }

        self._publish_ready(False)

        if publish_empty:
            self._publish_empty_outputs()

    def _reset_all_runtime(
        self,
        *,
        publish_empty: bool,
    ) -> None:
        self._clear_prepared_state(publish_empty=publish_empty)

        self.raw_coordinate_mode = None
        self.raw_marking_points = []

        self.extension_mode = None
        self.dummy_point_distance_m = None
        self.row_transition_threshold_m = None

        self.mission_id = None
        self.mission_checksum = None
        self.last_error = None

    def _set_error(
        self,
        message: str,
    ) -> None:
        self._clear_prepared_state(publish_empty=True)

        self.last_error = str(message)

        self._publish_status(
            state="ERROR",
            message=self.last_error,
        )

        self.get_logger().error("Trajectory preparation failed: " f"{self.last_error}")

    def _log_waiting(
        self,
        reason: str,
    ) -> None:
        now = self.get_clock().now()

        if (now - self.last_wait_log_time).nanoseconds < 1_000_000_000:
            return

        self.last_wait_log_time = now

        self.get_logger().info("TRAJECTORY PREPARING: " f"{reason}")

        self._publish_status(
            state="PREPARING",
            message=reason,
        )

    # ==========================================================
    # Control loop
    # ==========================================================

    def _control_loop(
        self,
    ) -> None:
        with self._lock:
            if not self.prepare_requested:
                return

            if self.raw_coordinate_mode is None or not self.raw_marking_points:
                self._set_error("No valid mission is loaded")

                return

            if self.raw_coordinate_mode == "gps":
                (
                    reference_ready,
                    reason,
                ) = self._reference_is_ready()

                now = self.get_clock().now()

                if not reference_ready:
                    self.rtk_ready_since = None

                    self._publish_ready(False)

                    self._log_waiting(reason)

                    return

                if self.rtk_ready_since is None:
                    self.rtk_ready_since = now

                    self._log_waiting("RTK FIXED detected; " "stabilizing reference")

                    return

                stable_age = (now - self.rtk_ready_since).nanoseconds / 1e9

                if stable_age < self.rtk_stable_sec:
                    self._log_waiting(
                        "RTK stable " f"{stable_age:.1f}/" f"{self.rtk_stable_sec:.1f}s"
                    )

                    return

            (
                rover_start_ready,
                rover_start_reason,
            ) = self._rover_start_is_ready()

            if not rover_start_ready:
                self._publish_ready(False)

                self._log_waiting(rover_start_reason)

                return

            try:
                # This is the rover position at trajectory preparation.
                rover_start = self._current_rover_start_point()

                marking_points = self._convert_markings_to_local()

                (
                    navigation_points,
                    point_types,
                    marking_indices,
                    dummy_count,
                ) = self._generate_navigation_path(
                    marking_points,
                    rover_start,
                )

                stamp = self.get_clock().now().to_msg()

                mission_path = self._build_path(
                    marking_points,
                    stamp,
                )

                navigation_path = self._build_path(
                    navigation_points,
                    stamp,
                )

                signature = self._make_signature(
                    navigation_points=(navigation_points),
                    marking_points=(marking_points),
                    point_types=(point_types),
                    marking_indices=(marking_indices),
                )

                self.prepared_marking_points = marking_points

                self.prepared_navigation_points = navigation_points

                self.prepared_path_types = point_types

                self.prepared_marking_indices = marking_indices

                self.prepared_path_signature = signature

                self.mission_waypoints_pub.publish(mission_path)

                self.nav_path_pub.publish(navigation_path)

                self._publish_path_metadata(
                    point_types=point_types,
                    marking_indices=(marking_indices),
                )

                # Commit marker: subscribers install the separately retained
                # path components only after this matching signature arrives.
                self._publish_path_signature(signature)

                self.prepare_requested = False
                self.preparing = False
                self.ready = True
                self.last_error = None

                self._publish_ready(True)

                self._publish_status(
                    state="READY",
                    message=("Mission trajectory prepared"),
                    dummy_count=dummy_count,
                )

                pass_through_count = sum(
                    1
                    for value in point_types
                    if (value == self.POINT_TYPE_PASS_THROUGH)
                )

                self.get_logger().warn("===== TRAJECTORY READY =====")

                self.get_logger().warn(f"Mission ID        : " f"{self.mission_id}")

                self.get_logger().warn(f"Signature         : " f"{signature[:12]}")

                self.get_logger().warn(f"Marking points    : " f"{len(marking_points)}")

                self.get_logger().warn(f"Dummy points      : " f"{dummy_count}")

                self.get_logger().warn(f"Pass-through pts  : " f"{pass_through_count}")

                self.get_logger().warn(
                    f"Navigation points : " f"{len(navigation_points)}"
                )

                self.get_logger().warn("Ready topic        : true")

            except ValueError as error:
                self._set_error(str(error))


def main(
    args: list[str] | None = None,
) -> None:
    rclpy.init(args=args)

    node = TrajectoryGenerator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node._publish_ready(False)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
