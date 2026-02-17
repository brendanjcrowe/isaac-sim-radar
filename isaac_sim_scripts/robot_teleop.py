"""Keyboard teleop for the robot via OmniGraph.

Run inside Isaac Sim's Script Editor after the scene is loaded.
Uses WASD keys for driving a differential-drive robot.

Controls:
    W / Up    — forward
    S / Down  — backward
    A / Left  — turn left
    D / Right — turn right
    Space     — stop
    Q         — quit teleop
"""

import carb
import carb.input
import omni.kit.app
import omni.usd
from pxr import PhysxSchema

# Teleop parameters
LINEAR_SPEED = 1.0    # m/s
ANGULAR_SPEED = 1.5   # rad/s
WHEEL_BASE = 0.5      # meters (distance between left/right wheels)
WHEEL_RADIUS = 0.1    # meters

# Joint paths (adjust for your robot)
LEFT_WHEEL_JOINT = "/World/Robot/left_wheel_joint"
RIGHT_WHEEL_JOINT = "/World/Robot/right_wheel_joint"


class KeyboardTeleop:
    """Simple keyboard teleop controller for differential drive."""

    def __init__(self):
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.kit.app.get_app().get_keyboard()
        self._appwindow = omni.appwindow.get_default_app_window()
        self._keyboard = self._appwindow.get_keyboard()

        self._forward = False
        self._backward = False
        self._left = False
        self._right = False
        self._active = True

        self._sub = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_key_event
        )

        self._stage = omni.usd.get_context().get_stage()
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )

        carb.log_info("Keyboard teleop active. WASD to drive, Q to quit.")

    def _on_key_event(self, event, *args, **kwargs):
        """Handle keyboard press/release events."""
        pressed = event.type == carb.input.KeyboardEventType.KEY_PRESS
        released = event.type == carb.input.KeyboardEventType.KEY_RELEASE

        if not (pressed or released):
            return True

        is_down = pressed
        key = event.input

        if key in (carb.input.KeyboardInput.W, carb.input.KeyboardInput.UP):
            self._forward = is_down
        elif key in (carb.input.KeyboardInput.S, carb.input.KeyboardInput.DOWN):
            self._backward = is_down
        elif key in (carb.input.KeyboardInput.A, carb.input.KeyboardInput.LEFT):
            self._left = is_down
        elif key in (carb.input.KeyboardInput.D, carb.input.KeyboardInput.RIGHT):
            self._right = is_down
        elif key == carb.input.KeyboardInput.SPACE:
            self._forward = self._backward = self._left = self._right = False
        elif key == carb.input.KeyboardInput.Q and pressed:
            self._active = False
            carb.log_info("Teleop deactivated.")

        return True

    def _on_update(self, event):
        """Apply velocity commands each frame."""
        if not self._active:
            return

        # Compute linear and angular velocity
        linear = 0.0
        angular = 0.0
        if self._forward:
            linear += LINEAR_SPEED
        if self._backward:
            linear -= LINEAR_SPEED
        if self._left:
            angular += ANGULAR_SPEED
        if self._right:
            angular -= ANGULAR_SPEED

        # Differential drive: convert (v, omega) to wheel velocities
        v_left = (linear - angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        v_right = (linear + angular * WHEEL_BASE / 2.0) / WHEEL_RADIUS

        # Apply to joints
        self._set_joint_velocity(LEFT_WHEEL_JOINT, v_left)
        self._set_joint_velocity(RIGHT_WHEEL_JOINT, v_right)

    def _set_joint_velocity(self, joint_path: str, velocity: float):
        """Set target velocity on a drive joint."""
        prim = self._stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            return

        drive = PhysxSchema.PhysxJointAPI(prim)
        if drive:
            # Set velocity target via the drive API
            vel_attr = prim.GetAttribute("drive:angular:physics:targetVelocity")
            if vel_attr:
                vel_attr.Set(velocity)

    def shutdown(self):
        """Clean up subscriptions."""
        if self._sub:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)
            self._sub = None
        self._update_sub = None


# Global instance
_teleop = None


def start():
    """Start the keyboard teleop controller."""
    global _teleop
    if _teleop is not None:
        _teleop.shutdown()
    _teleop = KeyboardTeleop()


def stop():
    """Stop the keyboard teleop controller."""
    global _teleop
    if _teleop is not None:
        _teleop.shutdown()
        _teleop = None


if __name__ == "__main__":
    start()
