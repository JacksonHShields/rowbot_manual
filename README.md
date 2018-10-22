# Rowbot Manual

This is a package for using manual control for WAM-V.

The modes available are:
- tank-drive: Each joystick axes is mapped directly to a motor. This is open loop.
- diff-drive: The left joystick controls the thrust, the right joystick controls the heading. These are both open-loop, and designed to be operated when the controllers aren't working.
- vel-drive: The joysticks are sent to the velocity controller. Needs a working low level controller.
