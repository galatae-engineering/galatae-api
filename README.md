# Robot

A Python interface for controlling the Galatae Robot arm over a serial connection through GCODE.

## Installation

Requires `pyserial` and optionally `opencv-python` (for `show_video`):

```bash
pip install pyserial opencv-python
```

## Usage

```python
from robot import Robot

r = Robot()
r.set_joint_speed(50)
r.enable_motors()
r.calibrate()
r.go_to_pose([450, 0, 300, 180, 0])
print(r.get_tool_pose())
r.go_to_foetus_pos()
r.disable_motors()
```

## API Reference

### `Robot(debug=False)`
Creates a new `Robot` instance. Automatically searches connected serial ports for a device named `"Giga"` and opens a serial connection to it at 115200 baud. Prints an error if no matching device is found. Set `debug=True` to print all messages sent to and received from the board.

### `calibrate()`
Sends the homing command (`$H`), moving the arm to its reference/home position.

### `set_joint_speed(speed)`
Sets the joint movement speed in degrees per second. A value around `100` is a reasonable default.

### `go_to_pose(pose)`
Moves the arm to an absolute `pose` (`[x, y, z, pitch, roll, gripper_angle (optional)]`).

### `linear_move_to_pose(pose)`
Moves the arm in a straight line from its current pose to target pose `pose`, by breaking the path into small incremental steps and calling `go_to_pose` for each one.

### `go_to_foetus_pos()`
Moves the arm to its compact "foetus"/folded rest position (`G28`).

### `get_tool_pose()`
Queries the board and returns the current tool-center-point pose as reported by the firmware.

### `get_angles()`
Queries the board and returns the current joint angles as reported by the firmware.

### `enable_motors()`
Energies the motors (`M17`).

### `disable_motors()`
Disables/de-energizes the motors (`M18`), allowing the arm to be moved freely by hand.

### `set_tool(coord)`
Sets the tool offset coordinates (`[x, y, z]`) on the board (`G10L2P1`).

### `probe(pose)`
Moves toward the given `pose` using a probing G-code command (`G38.2`), typically used to detect contact with a surface. `pose = [x, y, z, pitch, roll, gripper_angle]`.

### `update_absolute_distance_mode(expected_value)`
Switches the board between absolute (`G90`) and relative (`G91`) positioning modes if it isn't already in the requested mode. `expected_value=True` for absolute, `False` for relative.

### `jog(pose)`
Moves the arm by a relative offset (`pose`) from its current position.

### `linear_probe(pose)`
Moves in a straight line from the current pose toward `pose`, probing at each intermediate step, and stops as soon as contact is detected (or the target is reached). Returns `True` if contact was detected.

### `open_gripper(extra_degrees=0)`
Opens the gripper. An optional `extra_degrees` value can be added to open it further than the default position.

### `close_gripper(extra_degrees=0)`
Closes the gripper. An optional `extra_degrees` value can be added to the closing angle.

### `calibrate_gripper()`
Homes/calibrates the gripper (sends `$H`).





### `reset_angles(angles)`
Overrides the current joint angle values without moving the arm (`G92`), effectively re-zeroing the reported position.



### `show_video()`
Opens the default camera (via OpenCV) and displays a live video feed in a window. Press `q` to close it. Does nothing but print a message if no camera is available.

### `send_message(message)`
Sends a raw string message to the Arduino over serial, terminated with a newline. Does not wait for a response.

### `send_message_and_wait_conf(message)`
Sends a raw message and blocks until the board replies with `"ok"` or `"error"`. Returns `True` if the command succeeded, `False` otherwise.

### `reset()`
Sends a full reset command (`$RST=*`) to the board.