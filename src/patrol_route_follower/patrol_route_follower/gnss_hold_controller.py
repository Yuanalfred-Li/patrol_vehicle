#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional


NORMAL = "GNSS_NORMAL"
WEAK_PENDING = "GNSS_WEAK_PENDING"
HEADING_HOLD = "GNSS_HEADING_HOLD"
RECOVERING = "GNSS_RECOVERING"
STOPPED = "GNSS_STOPPED"

HOLD = "HOLD"
RESUME = "RESUME"
STOP = "STOP"


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


@dataclass(frozen=True)
class HoldConfig:
    enabled: bool = False

    weak_debounce_sec: float = 0.3
    maximum_hold_sec: float = 3.0
    hold_speed_rpm: float = 8.0

    maximum_heading_error_deg: float = 8.0
    heading_control_kp: float = 8.0
    maximum_steering_request: float = 80.0
    maximum_entry_steering_request: float = 80.0

    minimum_straight_segment_length_m: float = 4.0

    recovery_stable_sec: float = 1.0
    recovery_max_path_error_m: float = 1.0
    recovery_max_heading_error_deg: float = 15.0
    recovery_max_progress_jump_m: float = 2.0

    auto_resume_after_stop: bool = False


@dataclass(frozen=True)
class HoldDecision:
    action: str
    state: str

    speed_rpm: float = 0.0
    steering_request: float = 0.0
    heading_error_deg: float = 0.0

    elapsed_sec: float = 0.0
    remaining_sec: float = 0.0

    reason: str = ""
    recovered_progress_s: Optional[float] = None


class GnssHoldController:

    def __init__(self, config: HoldConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.state = NORMAL
        self.stop_reason = ""

        self.bad_since: Optional[float] = None
        self.recovery_since: Optional[float] = None

        self.target_heading_rad = 0.0
        self.imu_to_ros_offset_rad = 0.0
        self.frozen_progress_s = 0.0

    def begin(
        self,
        *,
        now: float,
        target_heading_rad: float,
        pose_yaw_rad: float,
        imu_heading_deg: float,
        straight_remaining_m: float,
        frozen_progress_s: float,
    ) -> HoldDecision:
        if not self.config.enabled:
            return self._stop(
                now,
                "GNSS heading hold is disabled",
            )

        values = (
            now,
            target_heading_rad,
            pose_yaw_rad,
            imu_heading_deg,
            straight_remaining_m,
            frozen_progress_s,
        )

        if not all(math.isfinite(value) for value in values):
            return self._stop(
                now,
                "heading hold input contains non-finite value",
            )

        if (
            straight_remaining_m
            < self.config.minimum_straight_segment_length_m
        ):
            return self._stop(
                now,
                (
                    "remaining straight distance too short: "
                    f"{straight_remaining_m:.2f}m < "
                    f"{self.config.minimum_straight_segment_length_m:.2f}m"
                ),
            )

        self.target_heading_rad = normalize_angle(
            target_heading_rad
        )

        # 定位正常时：
        # ROS航向 = 固定转换偏移 - MINS原始航向。
        # 由最后一个有效Pose和同时刻原始IMU航向反推出转换偏移，
        # 避免在本模块重复配置yaw_offset。
        self.imu_to_ros_offset_rad = normalize_angle(
            pose_yaw_rad
            + math.radians(imu_heading_deg)
        )

        self.frozen_progress_s = float(frozen_progress_s)
        self.bad_since = float(now)
        self.recovery_since = None
        self.stop_reason = ""
        self.state = WEAK_PENDING

        return self._hold_decision(
            now=now,
            imu_heading_deg=imu_heading_deg,
            entry=True,
        )

    def update_bad(
        self,
        *,
        now: float,
        imu_heading_deg: float,
    ) -> HoldDecision:
        if self.state == NORMAL:
            return self._stop(
                now,
                "heading hold was not initialized",
            )

        if self.state == STOPPED:
            return self._stopped_decision(now)

        elapsed = self._elapsed(now)

        if self.state == RECOVERING:
            # 单帧恢复不能重置总计时器。
            self.recovery_since = None
            self.state = HEADING_HOLD

        if elapsed < self.config.weak_debounce_sec:
            self.state = WEAK_PENDING
        else:
            self.state = HEADING_HOLD

        return self._hold_decision(
            now=now,
            imu_heading_deg=imu_heading_deg,
            entry=False,
        )

    def update_good(
        self,
        *,
        now: float,
        imu_heading_deg: float,
        recovery_path_error_m: float,
        recovery_heading_error_deg: float,
        recovered_progress_s: float,
    ) -> HoldDecision:
        if self.state == NORMAL:
            return HoldDecision(
                action=RESUME,
                state=NORMAL,
                reason="GNSS already normal",
                recovered_progress_s=recovered_progress_s,
            )

        if self.state == STOPPED:
            return self._stopped_decision(now)

        elapsed = self._elapsed(now)

        if elapsed > self.config.maximum_hold_sec:
            return self._stop(
                now,
                "GNSS hold timeout during recovery",
            )

        values = (
            imu_heading_deg,
            recovery_path_error_m,
            recovery_heading_error_deg,
            recovered_progress_s,
        )

        if not all(math.isfinite(value) for value in values):
            return self._stop(
                now,
                "GNSS recovery contains non-finite value",
            )

        if (
            abs(recovery_path_error_m)
            > self.config.recovery_max_path_error_m
        ):
            return self._stop(
                now,
                (
                    "GNSS recovery path error too large: "
                    f"{recovery_path_error_m:.2f}m"
                ),
            )

        if (
            abs(recovery_heading_error_deg)
            > self.config.recovery_max_heading_error_deg
        ):
            return self._stop(
                now,
                (
                    "GNSS recovery heading error too large: "
                    f"{recovery_heading_error_deg:.1f}deg"
                ),
            )

        progress_jump = abs(
            recovered_progress_s
            - self.frozen_progress_s
        )

        if (
            progress_jump
            > self.config.recovery_max_progress_jump_m
        ):
            return self._stop(
                now,
                (
                    "GNSS recovery progress jump too large: "
                    f"{progress_jump:.2f}m"
                ),
            )

        if self.state != RECOVERING:
            self.state = RECOVERING
            self.recovery_since = float(now)

        assert self.recovery_since is not None

        stable_time = now - self.recovery_since

        if stable_time >= self.config.recovery_stable_sec:
            self.state = NORMAL
            self.bad_since = None
            self.recovery_since = None

            return HoldDecision(
                action=RESUME,
                state=NORMAL,
                elapsed_sec=elapsed,
                remaining_sec=max(
                    0.0,
                    self.config.maximum_hold_sec - elapsed,
                ),
                reason="GNSS recovery validated",
                recovered_progress_s=float(
                    recovered_progress_s
                ),
            )

        return self._hold_decision(
            now=now,
            imu_heading_deg=imu_heading_deg,
            entry=False,
        )

    def _hold_decision(
        self,
        *,
        now: float,
        imu_heading_deg: float,
        entry: bool,
    ) -> HoldDecision:
        elapsed = self._elapsed(now)

        if elapsed > self.config.maximum_hold_sec:
            return self._stop(
                now,
                (
                    "GNSS hold timeout: "
                    f"{elapsed:.2f}s > "
                    f"{self.config.maximum_hold_sec:.2f}s"
                ),
            )

        if not math.isfinite(imu_heading_deg):
            return self._stop(
                now,
                "IMU heading is not finite",
            )

        imu_ros_yaw = normalize_angle(
            self.imu_to_ros_offset_rad
            - math.radians(imu_heading_deg)
        )

        heading_error_rad = normalize_angle(
            self.target_heading_rad - imu_ros_yaw
        )
        heading_error_deg = math.degrees(
            heading_error_rad
        )

        if (
            abs(heading_error_deg)
            > self.config.maximum_heading_error_deg
        ):
            return self._stop(
                now,
                (
                    "heading divergence: "
                    f"{heading_error_deg:.1f}deg > "
                    f"{self.config.maximum_heading_error_deg:.1f}deg"
                ),
            )

        steering_request = (
            self.config.heading_control_kp
            * heading_error_deg
        )

        steering_limit = abs(
            self.config.maximum_steering_request
        )

        steering_request = max(
            -steering_limit,
            min(steering_limit, steering_request),
        )

        if (
            entry
            and abs(steering_request)
            > abs(
                self.config.maximum_entry_steering_request
            )
        ):
            return self._stop(
                now,
                (
                    "initial steering correction too large: "
                    f"{steering_request:.1f}"
                ),
            )

        return HoldDecision(
            action=HOLD,
            state=self.state,
            speed_rpm=float(
                self.config.hold_speed_rpm
            ),
            steering_request=float(steering_request),
            heading_error_deg=float(heading_error_deg),
            elapsed_sec=elapsed,
            remaining_sec=max(
                0.0,
                self.config.maximum_hold_sec - elapsed,
            ),
            reason="IMU heading hold active",
        )

    def _elapsed(self, now: float) -> float:
        if self.bad_since is None:
            return 0.0

        return max(0.0, float(now) - self.bad_since)

    def _stop(
        self,
        now: float,
        reason: str,
    ) -> HoldDecision:
        self.state = STOPPED
        self.stop_reason = str(reason)

        return self._stopped_decision(now)

    def _stopped_decision(
        self,
        now: float,
    ) -> HoldDecision:
        elapsed = self._elapsed(now)

        return HoldDecision(
            action=STOP,
            state=STOPPED,
            elapsed_sec=elapsed,
            remaining_sec=0.0,
            reason=self.stop_reason or "GNSS hold stopped",
        )
