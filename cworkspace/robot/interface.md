# cbott G-Code
Robot interface language loosely based on G-Code structure and implemented in robotlib for robot control.

This document describes the available commands.

---

## J - Set Raw Joint Values
### Description
Sets the 4 robot joint values in AX12 "steps" 0-1023

All joints move to the commanded position at the last-commanded speed for the joint, independent of other joints. That is, movement of the end effector will not be linear and joints will complete motion at different times.
### Usage
`J <j0> <j1> <j2> <j3>`

j0-j3: (uint) Value to set joint 0-3
### Examples
Set robot approximately to home position\
`J 500 400 800 800`

---

## H - Home
### Description
Moves the robot to the home (default) pose

This command takes no parameters
### Usage
`H`

---

## A - Set Joint Angles
### Description
Sets the 4 joints of the robot to the specified angles (radians) with speed controlled so that all joints complete motion at the same time.

Joint speed is set by the optional feedrate parameter.
### Usage
`A <a0> <a1> <a2> <a3> [<f>]`

a0-a3: (float) Radian angle to set joint 0-3\
f: (uint) Optional feedrate which dictates the maximum speed for any joint, in AX12 speed units with range 0-1023. Defaults to 100.
### Examples
Set the robot to approximately home position at three times the default speed\
`A 1.57 2.0 -1.57 -1.57 300`

---

## E - Print Errors
### Description
Read out servo error messages for each joint.

This command takes no parameters.
### Usage
`E`

Response
```
e0
e1
e2
e3
```
e0-3: Error status byte from joints 0-3, in order

---

## G - Set Position
### Description
Moves the end effector to a desired position in cartesian space, with a specified ground-relative angle.

Maximum joint speed is controlled by the optional feedrate parameter, with individual joint speeds set so that all joints complete motion at the same time.
### Usage
`G <x> <y> <z> <a> [<f>]`

x: (float) Target X coordinate in milimeters\
y: (float) Target Y coordinate in milimeters\
z: (float) Target Z coordinate in milimeters\
a: (float) Target ground-relative end effector angle in radians\
f: (uint) Optional feedrate which dictates the maximum speed for any joint, in AX12 speed units with range 0-1023. Defaults to 100.
### Examples
Move to 100mm in all axes, with the end effector pointing up, at a slow speed\
`G 100 100 100 1.57 50`
