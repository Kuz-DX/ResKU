#!/usr/bin/env python3
"""Independent joint position targets without MoveIt Servo or IK.

Shares feedback freshness, focus, hold, following-error and speed/limit guards
with manual_controller.py. No robot model is required in this mode.
"""
import numpy as np

from manual_controller import ManualController, main


class SeparateManualController(ManualController):
    node_name = 'manual_controller_sep'
    requires_model = False
    extra_parameters = {'shoulder_axis': 4, 'elbow_axis': 1, 'wrist_pitch_axis': 7,
                        'wrist_yaw_axis': 6}
    # One joint at its boundary must not block another joint.
    limit_groups = tuple(slice(i, i+1) for i in range(6))

    def command_velocity(self, measured):
        yaw = self.axis('wrist_yaw_axis')
        yaw *= -1 if self.params['invert_wrist_yaw'] else 1
        return self.speeds * np.array([
            self.button('base_ccw_button')-self.button('base_cw_button'),
            self.axis('shoulder_axis'),
            -self.axis('elbow_axis'),
            self.axis('wrist_pitch_axis'),
            0.,
            yaw,
        ])


if __name__ == '__main__':
    main(SeparateManualController)
