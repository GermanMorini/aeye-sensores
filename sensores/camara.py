#!/usr/bin/env python3

from __future__ import annotations

import os
import math
from pathlib import Path
from typing import Dict

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import CameraPan, CameraStatus

try:
    from onvif import ONVIFCamera
except Exception:  # pragma: no cover - dependency/import failure path
    ONVIFCamera = None


def _parse_env_file(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except Exception:
        return values

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        if key:
            values[key] = value
    return values


def _resolve_default_env_file() -> Path:
    try:
        share_dir = Path(get_package_share_directory("sensores"))
        candidates = [share_dir / ".env"]
        try:
            workspace_root = share_dir.parents[3]
            candidates.append(workspace_root / "src" / "sensores" / ".env")
        except IndexError:
            pass

        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]
    except Exception:
        # Fallback when ament index is unavailable; keep this non-fatal.
        return Path(__file__).resolve().parents[2] / ".env"


def _onvif_default_wsdl_dir() -> Path | None:
    if ONVIFCamera is None:
        return None
    try:
        defaults = getattr(ONVIFCamera.__init__, "__defaults__", None)
        if defaults and isinstance(defaults[0], str):
            return Path(defaults[0])
    except Exception:
        return None
    return None


def _resolve_default_wsdl_dir() -> Path:
    candidates = []
    try:
        share_dir = Path(get_package_share_directory("sensores"))
        candidates.append(share_dir / "wsdl")
        try:
            workspace_root = share_dir.parents[3]
            candidates.append(workspace_root / "src" / "sensores" / "wsdl")
        except IndexError:
            pass
    except Exception:
        pass

    candidates.append(Path(__file__).resolve().parents[2] / "wsdl")
    onvif_default = _onvif_default_wsdl_dir()
    if onvif_default is not None:
        candidates.append(onvif_default)

    for candidate in candidates:
        if candidate.is_dir() and (candidate / "devicemgmt.wsdl").exists():
            return candidate
    return candidates[0] if candidates else Path("wsdl")


class CamaraNode(Node):
    def __init__(self) -> None:
        super().__init__("camara")

        default_env_file = str(_resolve_default_env_file())
        self.declare_parameter("env_file", default_env_file)
        self._env_file = Path(str(self.get_parameter("env_file").value))
        self._env_data = _parse_env_file(self._env_file)

        self.declare_parameter("camera_host", self._env_cfg("CAMERA_HOST", "192.168.1.64"))
        self.declare_parameter("camera_port", int(self._env_cfg("CAMERA_PORT", "80")))
        self.declare_parameter("camera_user", self._env_cfg("CAMERA_USER", "admin"))
        self.declare_parameter("camera_pass", self._env_cfg("CAMERA_PASS", "CHANGE_ME"))

        self.declare_parameter("camera_wsdl_dir", str(_resolve_default_wsdl_dir()))
        self.declare_parameter("camera_profile_token", "")
        self.declare_parameter("camera_zoom_fixed_level", 0.5)
        self.declare_parameter("camera_zoom_zero_level", 0.0)
        self.declare_parameter("camera_zoom_initial_in", False)
        self.declare_parameter("camera_pan_deg_min", 0.0)
        self.declare_parameter("camera_pan_deg_max", 355.0)
        self.declare_parameter("camera_onvif_pan_min", -1.0)
        self.declare_parameter("camera_onvif_pan_max", 1.0)

        self._host = str(self.get_parameter("camera_host").value)
        self._port = int(self.get_parameter("camera_port").value)
        self._user = str(self.get_parameter("camera_user").value)
        self._password = str(self.get_parameter("camera_pass").value)
        self._wsdl_dir = str(self.get_parameter("camera_wsdl_dir").value).strip()
        self._profile_token_cfg = str(self.get_parameter("camera_profile_token").value)
        self._zoom_fixed_level = self._clamp_zoom(float(self.get_parameter("camera_zoom_fixed_level").value))
        self._zoom_zero_level = self._clamp_zoom(float(self.get_parameter("camera_zoom_zero_level").value))
        self._zoom_in = bool(self.get_parameter("camera_zoom_initial_in").value)
        self._pan_deg_min = float(self.get_parameter("camera_pan_deg_min").value)
        self._pan_deg_max = float(self.get_parameter("camera_pan_deg_max").value)
        self._onvif_pan_min = float(self.get_parameter("camera_onvif_pan_min").value)
        self._onvif_pan_max = float(self.get_parameter("camera_onvif_pan_max").value)

        self._camera = None
        self._media_service = None
        self._ptz_service = None
        self._profile_token = ""
        self._ready = False
        self._ready_error = ""
        self._last_command = "none"

        self._connect_onvif()

        self.create_service(CameraPan, "/camara/camera_pan", self._on_camera_pan)
        self.create_service(
            Trigger,
            "/camara/camera_zoom_toggle",
            self._on_camera_zoom_toggle,
        )
        self.create_service(CameraStatus, "/camara/camera_status", self._on_camera_status)

        self.get_logger().info(
            "camara node ready "
            f"(env_file={self._env_file}, host={self._host}:{self._port}, wsdl_dir={self._wsdl_dir}, onvif_ready={self._ready})"
        )

    def _env_cfg(self, key: str, default: str) -> str:
        value = self._env_data.get(key, "")
        if value:
            return value
        env_value = os.environ.get(key, "")
        if env_value:
            return env_value
        return default

    def _connect_onvif(self) -> None:
        if ONVIFCamera is None:
            self._ready = False
            self._ready_error = "onvif-zeep is not installed"
            self.get_logger().error(self._ready_error)
            return

        if not self._host or not self._user or not self._password:
            self._ready = False
            self._ready_error = (
                "missing CAMERA_HOST/CAMERA_USER/CAMERA_PASS in env config"
            )
            self.get_logger().error(self._ready_error)
            return

        wsdl_dir = Path(self._wsdl_dir) if self._wsdl_dir else _resolve_default_wsdl_dir()
        wsdl_probe = wsdl_dir / "devicemgmt.wsdl"
        if not wsdl_probe.exists():
            self._ready = False
            self._ready_error = (
                f"ONVIF WSDL not found: {wsdl_probe}. "
                "Set ROS param camera_wsdl_dir to a valid wsdl directory."
            )
            self.get_logger().error(self._ready_error)
            return

        try:
            self.get_logger().info(
                "Attempting ONVIF connection "
                f"(host={self._host}, port={self._port}, user={self._user}, wsdl_dir={wsdl_dir})"
            )
            self._camera = ONVIFCamera(
                self._host,
                self._port,
                self._user,
                self._password,
                str(wsdl_dir),
            )
            self._media_service = self._camera.create_media_service()
            self._ptz_service = self._camera.create_ptz_service()

            if self._profile_token_cfg:
                self._profile_token = self._profile_token_cfg
            else:
                profiles = self._media_service.GetProfiles()
                if not profiles:
                    raise RuntimeError("camera media service returned no profiles")
                self._profile_token = str(profiles[0].token)

            self._ready = True
            self._ready_error = ""
        except Exception as exc:
            self._ready = False
            self._ready_error = f"ONVIF init failed: {exc}"
            self.get_logger().error(self._ready_error)

    def _clamp_zoom(self, value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _clamp(self, value: float, lo: float, hi: float) -> float:
        return max(float(lo), min(float(hi), float(value)))

    def _normalize_angle_deg(self, angle_deg: float) -> float:
        angle = math.fmod(float(angle_deg), 360.0)
        if angle < 0.0:
            angle += 360.0
        return angle

    def _angle_deg_to_onvif_pan(self, angle_deg: float) -> float:
        deg_lo = min(self._pan_deg_min, self._pan_deg_max)
        deg_hi = max(self._pan_deg_min, self._pan_deg_max)
        clamped_deg = self._clamp(angle_deg, deg_lo, deg_hi)

        deg_span = self._pan_deg_max - self._pan_deg_min
        if abs(deg_span) < 1e-6:
            raise RuntimeError("invalid pan mapping: camera_pan_deg_min == camera_pan_deg_max")

        ratio = (clamped_deg - self._pan_deg_min) / deg_span
        pan = self._onvif_pan_min + ratio * (self._onvif_pan_max - self._onvif_pan_min)
        pan_lo = min(self._onvif_pan_min, self._onvif_pan_max)
        pan_hi = max(self._onvif_pan_min, self._onvif_pan_max)
        return self._clamp(pan, pan_lo, pan_hi)

    def _get_status(self):
        req = self._ptz_service.create_type("GetStatus")
        req.ProfileToken = self._profile_token
        return self._ptz_service.GetStatus(req)

    def _set_pan_absolute(self, angle_deg: float) -> tuple[bool, str]:
        try:
            status = self._get_status()
            position = getattr(status, "Position", None)
            pan_tilt = getattr(position, "PanTilt", None) if position is not None else None
            zoom = getattr(position, "Zoom", None) if position is not None else None

            target_pan = self._angle_deg_to_onvif_pan(angle_deg)
            current_tilt = 0.0
            if pan_tilt is not None and hasattr(pan_tilt, "y"):
                current_tilt = float(pan_tilt.y)

            req = self._ptz_service.create_type("AbsoluteMove")
            req.ProfileToken = self._profile_token

            out_pos = {"PanTilt": {"x": float(target_pan), "y": float(current_tilt)}}
            if zoom is not None and hasattr(zoom, "x"):
                out_pos["Zoom"] = {"x": self._clamp_zoom(float(zoom.x))}
            req.Position = out_pos

            self._ptz_service.AbsoluteMove(req)
            return True, ""
        except Exception as exc:
            return False, f"AbsoluteMove pan failed: {exc}"

    def _set_zoom_absolute(self, target_zoom: float) -> tuple[bool, str]:
        try:
            status = self._get_status()
            position = getattr(status, "Position", None)
            pan_tilt = None
            if position is not None:
                pan_tilt = getattr(position, "PanTilt", None)

            req = self._ptz_service.create_type("AbsoluteMove")
            req.ProfileToken = self._profile_token

            out_pos = {"Zoom": {"x": self._clamp_zoom(target_zoom)}}
            if pan_tilt is not None:
                out_pos["PanTilt"] = {"x": float(pan_tilt.x), "y": float(pan_tilt.y)}
            req.Position = out_pos

            self._ptz_service.AbsoluteMove(req)
            return True, ""
        except Exception as exc:
            return False, f"AbsoluteMove zoom failed: {exc}"

    def _zoom_toggle(self) -> tuple[bool, str]:
        epsilon = 0.02
        current_zoom = None
        try:
            status = self._get_status()
            position = getattr(status, "Position", None)
            zoom = getattr(position, "Zoom", None) if position is not None else None
            if zoom is not None:
                current_zoom = float(zoom.x)
        except Exception:
            current_zoom = None

        if current_zoom is not None:
            is_zero = abs(current_zoom - self._zoom_zero_level) <= epsilon
            target = self._zoom_fixed_level if is_zero else self._zoom_zero_level
        else:
            target = self._zoom_zero_level if self._zoom_in else self._zoom_fixed_level

        ok, err = self._set_zoom_absolute(target)
        if ok:
            self._zoom_in = abs(target - self._zoom_zero_level) > epsilon
        return ok, err

    def _on_camera_pan(
        self, request: CameraPan.Request, response: CameraPan.Response
    ) -> CameraPan.Response:
        if not self._ready:
            response.ok = False
            response.error = self._ready_error or "camera is not ready"
            response.applied_angle_deg = 0.0
            return response

        input_angle = float(request.angle_deg)
        if not math.isfinite(input_angle):
            response.ok = False
            response.error = "angle_deg must be finite"
            response.applied_angle_deg = 0.0
            return response

        applied_angle = self._normalize_angle_deg(input_angle)
        ok, err = self._set_pan_absolute(applied_angle)

        response.ok = bool(ok)
        response.error = "" if ok else err
        response.applied_angle_deg = float(applied_angle)
        if ok:
            self._last_command = f"angle:{applied_angle:.1f}"
        return response

    def _on_camera_zoom_toggle(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._ready:
            response.success = False
            response.message = self._ready_error or "camera is not ready"
            return response

        ok, err = self._zoom_toggle()
        response.success = bool(ok)
        response.message = "" if ok else err
        if ok:
            self._last_command = "zoom_toggle"
        return response

    def _on_camera_status(
        self, _request: CameraStatus.Request, response: CameraStatus.Response
    ) -> CameraStatus.Response:
        if not self._ready:
            response.ok = False
            response.error = self._ready_error or "camera is not ready"
            response.last_command = self._last_command
            response.zoom_in = bool(self._zoom_in)
            return response

        response.ok = True
        response.error = ""

        response.last_command = self._last_command
        response.zoom_in = bool(self._zoom_in)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CamaraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
