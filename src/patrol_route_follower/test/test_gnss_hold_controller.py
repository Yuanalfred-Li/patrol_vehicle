import math
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from patrol_route_follower.gnss_hold_controller import (  # noqa: E402
    GnssHoldController,
    HoldConfig,
    HEADING_HOLD,
    HOLD,
    NORMAL,
    RECOVERING,
    RESUME,
    STOP,
    STOPPED,
    WEAK_PENDING,
)


def make_controller() -> GnssHoldController:
    return GnssHoldController(
        HoldConfig(
            enabled=True,
            weak_debounce_sec=0.3,
            maximum_hold_sec=3.0,
            hold_speed_rpm=8.0,
            maximum_heading_error_deg=8.0,
            heading_control_kp=8.0,
            maximum_steering_request=80.0,
            maximum_entry_steering_request=80.0,
            minimum_straight_segment_length_m=4.0,
            recovery_stable_sec=1.0,
            recovery_max_path_error_m=1.0,
            recovery_max_heading_error_deg=15.0,
            recovery_max_progress_jump_m=2.0,
            auto_resume_after_stop=False,
        )
    )


def begin(controller: GnssHoldController):
    return controller.begin(
        now=0.0,
        target_heading_rad=0.0,
        pose_yaw_rad=0.0,
        imu_heading_deg=90.0,
        straight_remaining_m=10.0,
        frozen_progress_s=5.0,
    )


def test_pending_hold_recovery_and_resume():
    controller = make_controller()

    decision = begin(controller)

    assert decision.action == HOLD
    assert decision.state == WEAK_PENDING
    assert decision.speed_rpm == 8.0

    decision = controller.update_bad(
        now=0.31,
        imu_heading_deg=90.0,
    )

    assert decision.action == HOLD
    assert decision.state == HEADING_HOLD

    decision = controller.update_good(
        now=0.50,
        imu_heading_deg=90.0,
        recovery_path_error_m=0.20,
        recovery_heading_error_deg=2.0,
        recovered_progress_s=5.20,
    )

    assert decision.action == HOLD
    assert decision.state == RECOVERING

    # 恢复阶段再次出现坏帧，不重置首次异常总计时。
    decision = controller.update_bad(
        now=0.70,
        imu_heading_deg=90.0,
    )

    assert decision.action == HOLD
    assert decision.state == HEADING_HOLD
    assert math.isclose(
        decision.elapsed_sec,
        0.70,
        abs_tol=1.0e-6,
    )

    decision = controller.update_good(
        now=0.80,
        imu_heading_deg=90.0,
        recovery_path_error_m=0.20,
        recovery_heading_error_deg=2.0,
        recovered_progress_s=5.20,
    )

    assert decision.state == RECOVERING

    decision = controller.update_good(
        now=1.81,
        imu_heading_deg=90.0,
        recovery_path_error_m=0.20,
        recovery_heading_error_deg=2.0,
        recovered_progress_s=5.20,
    )

    assert decision.action == RESUME
    assert decision.state == NORMAL
    assert decision.recovered_progress_s == 5.20


def test_total_timeout_includes_debounce():
    controller = make_controller()
    begin(controller)

    decision = controller.update_bad(
        now=3.01,
        imu_heading_deg=90.0,
    )

    assert decision.action == STOP
    assert decision.state == STOPPED
    assert "timeout" in decision.reason.lower()


def test_heading_divergence_stops():
    controller = make_controller()
    begin(controller)

    # 初始转换关系下，80°原始航向对应ROS航向+10°，
    # 相对0°目标产生-10°误差，超过8°限制。
    decision = controller.update_bad(
        now=0.40,
        imu_heading_deg=80.0,
    )

    assert decision.action == STOP
    assert "divergence" in decision.reason


def test_short_straight_segment_is_rejected():
    controller = make_controller()

    decision = controller.begin(
        now=0.0,
        target_heading_rad=0.0,
        pose_yaw_rad=0.0,
        imu_heading_deg=90.0,
        straight_remaining_m=3.9,
        frozen_progress_s=0.0,
    )

    assert decision.action == STOP
    assert "straight distance" in decision.reason


def test_implausible_recovery_stops():
    controller = make_controller()
    begin(controller)

    decision = controller.update_good(
        now=0.50,
        imu_heading_deg=90.0,
        recovery_path_error_m=1.20,
        recovery_heading_error_deg=2.0,
        recovered_progress_s=5.10,
    )

    assert decision.action == STOP
    assert "path error" in decision.reason


def test_stopped_state_never_auto_resumes():
    controller = make_controller()
    begin(controller)

    controller.update_bad(
        now=3.10,
        imu_heading_deg=90.0,
    )

    decision = controller.update_good(
        now=3.20,
        imu_heading_deg=90.0,
        recovery_path_error_m=0.0,
        recovery_heading_error_deg=0.0,
        recovered_progress_s=5.0,
    )

    assert decision.action == STOP
    assert decision.state == STOPPED
