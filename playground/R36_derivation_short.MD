# R36 DERIVATION

**We know the following from our geometry:**
```
1. R0u = R06 * R6g * Rgu 
2. R03 = R03 * R36
3. R6g = I
4. R0u = rotz(yaw) * roty(pitch) * rotx(roll)
5. Rgu = rotz(pi) * roty(-pi/2)
```

**For any matrix recall the following properties:**
```
1. R = I * R = R * I
2. (A * B) * C = A * (B * C)
3. if A = B then A * C = B * C and vice versa
4. if A = B then C * A = C * C and vice versa
5. R * R.inverse() = R.inverse() * R = I 
```

**Note that for rotation matrices**
```
R.inverse() = R.T 
Therefore: R.T * R = R * R.T = I 
```

**From the discussion above, you can do the following:** 
- _Subsitution_  
- - `R0u = (R03 * R36) * I * Rgu`
- _Multiply both sides by `Rgu.inverse() = Rgu.T` at the right_
- - `R0u * Rgu.T = R03 * R36 `
- _Multiply both sides by `R03.inverse() = R03.T` at the left_ 
- - `R03.T * R0u * Rgu.T = R36 `

**We conclude then that:**
```
R36 = R03.T * R0u * Rgu.T
where:
R0u = rotz(yaw) * roty(pitch) * rotx(roll)
Rgu = rotz(pi) * roty(-pi/2)
```

**Because `rotz(pi) * roty(-pi/2) = (rotz(pi) * roty(-pi/2)).T`, it is simply:**
```
R036 = R03.T * rotz(yaw) * roty(pitch) * rotx(roll) * rotz(pi) * roty(-pi/2)
```
