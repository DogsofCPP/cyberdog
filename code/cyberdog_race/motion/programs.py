"""
Pre-built motion programs using the TOML-based sequential executor pattern.
These are reusable motion sequences for common race tasks.
"""
import time
import toml
import os
from ..lcm_types import (
    GAIT_TROT_SLOW, GAIT_BOUND, GAIT_TROT_FAST,
    MODE_LOCOMOTION, MODE_RECOVERY_STAND, MODE_PURE_DAMPER,
    MODE_POSITION_INTERP, MODE_ACTION_TRIGGER,
    CONTACT_ALL,
)


def trot_slow_forward(ctrl, duration_s=2.0, speed=0.20):
    """Walk forward slowly for a set duration."""
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=speed, vy=0.0, wz=0.0,
        step_h_max=0.06, step_h_min=0.06,
        duration_ms=0,
    )
    time.sleep(duration_s)
    ctrl.stop_moving()


def trot_fast_forward(ctrl, duration_s=2.0, speed=0.30):
    """Walk forward at medium-fast speed."""
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,  # use TROT_SLOW for stability
        vx=speed, vy=0.0, wz=0.0,
        step_h_max=0.08, step_h_min=0.06,
        duration_ms=0,
    )
    time.sleep(duration_s)
    ctrl.stop_moving()


def bound_jump(ctrl, n_jumps=1, speed=0.20):
    """Execute bound/pronk jumps (for segment 5 or overcoming obstacles)."""
    for _ in range(n_jumps):
        ctrl.locomotion(
            gait_id=GAIT_BOUND,
            vx=speed, vy=0.0, wz=0.0,
            step_h_max=0.20, step_h_min=0.05,
            duration_ms=0,
        )
        time.sleep(1.0)


def recover_stand(ctrl, height=0.22):
    """Standard recovery stand sequence."""
    ctrl.recovery_stand()
    time.sleep(2.0)
    ctrl.set_height(height, duration_ms=500)
    time.sleep(0.6)


def pure_damper(ctrl):
    """Lie down."""
    ctrl.pure_damper()
    time.sleep(1.0)


def forward_and_adjust_height(ctrl, height=0.15, duration_s=2.0):
    """Walk forward with body lowered (e.g., for passing under height bar)."""
    ctrl.set_height(height, duration_ms=300)
    time.sleep(0.3)
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.15, vy=0.0, wz=0.0,
        step_h_max=0.04, step_h_min=0.02,
        duration_ms=0,
    )
    time.sleep(duration_s)
    ctrl.set_height(0.22, duration_ms=300)
    ctrl.stop_moving()


def turn_in_place(ctrl, angle_rad=1.57, speed=0.3):
    """Turn in place by a given angle (approximate)."""
    direction = 1.0 if angle_rad > 0 else -1.0
    wz = direction * abs(speed)
    # time to turn: angle / abs(wz)
    duration_s = abs(angle_rad) / abs(speed) + 0.5
    ctrl.locomotion(gait_id=GAIT_TROT_SLOW, vx=0.0, vy=0.0, wz=wz)
    time.sleep(duration_s)
    ctrl.stop_moving()


def strafe_sideways(ctrl, direction='left', duration_s=1.0, speed=0.15):
    """Strafe sideways (left or right)."""
    vy = -speed if direction == 'left' else speed
    ctrl.locomotion(
        gait_id=GAIT_TROT_SLOW,
        vx=0.0, vy=vy, wz=0.0,
        duration_ms=0,
    )
    time.sleep(duration_s)
    ctrl.stop_moving()
