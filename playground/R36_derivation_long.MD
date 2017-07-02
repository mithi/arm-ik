# R03 PROOF


- `R0u` - the rotation of gripper_URDF frame wrt to base frame
```
R0u = rotz(yaw) * roty(pitch) * rotx(roll) 
```
- Note: The yaw, pitch, and roll are given wrt to the URDF frame, these are extrinsic rotations, not intrinsic
- #`R0u != rotx(roll) * roty(pitch) * rotz(yaw)`


- `Rgu` the rotation gripper_URDF frame wrt to the gripper frame (DH)
```
Rgu = rotz(pi) * roty(-pi/2)
```
- We must convert `R0u` to gripper frame by performing:
- - a rotation of `180` degrees ccw about the z axis 
- - and then a rotation of `90` degrees cw about the new y axis. 


- `R6g` - the rotation of gripper wrt to joint 6 frame
- `R6g` is actually an identity matrix I, or no rotation
- The gripper frame wrt to the base frame is the same orientation as joint 6 with respect to the 
base frame, only translated by `0.303` units in its z axis.


- Get `R36` in terms of `R36`, `Rgu`, and `R6g`

- We know:
```
R0u = R06 * R6g * Rgu 
R0u = rotz(yaw) * roty(pitch) * rotx(roll) 
```

- We also know:
```
R06 = R03 * R36
```

- We can get this by subtitution:
```
(R03 * R36) * I * (Rgu) = rotz(yaw) * roty(pitch) * rotx(roll)
R03 * R36 * Rgu = rotz(yaw) * roty(pitch) * rotx(roll)
```

- Note that: `R * R.inverse() = I` and `R * I = I * R = R`
- Multiply `Rgu.inverse()` both sides
``` 
R03 * R36 * Rgu * Rgu.inverse() = (rotz(yaw) * roty(pitch) * rotx(roll)) * (Rgu.inverse())
R03 * R36 = rotz(yaw) * roty(pitch) * rotx(roll) * Rgu.inverse()
```

- Note that for rotation matrices: inverse of rotations is equal to rotation's transpose  `r.inverse() = r.T`
- And also `(rotz(pi) * roty(-pi/2)).T = rotz(pi) * roty(-pi/2)`

```
R03 * R36 = rotz(yaw) * roty(pitch) * rotx(roll) * Rgu.T
R03 * R36 = rotz(yaw) * roty(pitch) * rotx(roll) * (rotz(pi) * roty(-pi/2).T
```
- Strangely: `(rotz(pi) * roty(-pi/2)).T = rotz(pi) * roty(-pi/2)`

```
R03 * R36 = rotz(yaw) * roty(pitch) * rotx(roll) * (rotz(pi) * roty(-pi/2)
R03 * R36 = rotz(yaw) * roty(pitch) * rotx(roll) * rotz(pi) * roty(-pi/2)
```

- Multiplying `R03.inverse() = R03.T` to both sides of the equation we get
```
R36 = R03.T * rotz(yaw) * roty(pitch) * rotx(roll) * rotz(pi) * roty(-pi/2)
```
