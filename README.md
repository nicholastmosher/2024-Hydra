# 4H-Alarm Hydra 2024 Codebase

## Hybrid control

We would like to be able to combine inputs from many sources and blend them
into a single output. We'd like to be able to choose weights for each input,
and to be able to change the weights for different control "modes"

If we have N sources of inputs, we can represent them with P values like 
the following:

```
- P0
- P1
- ...
- PN
```

Each of these input source is represented as a Vector of input components.
For swerve drive, we control it with (x, y , rotation), where x and y
are translations in field space and rotation is an amount by which to rotate
the robot around its center. This means that a single P value is actually
a 3D vector, such as `P1 = (x, y, rot)`, where this 3-tuple is going to be
a Vector in code.

To control the weight of each input, we need to choose a number between
0 and 1, where 0 indicates 0 influence and 1 indicates full influence.
If we have 5 inputs, i.e. 5 P-values, we would need 5 weights, i.e. 5
t-values. The sum of all our chosen weights should be 1.

```
P = (P1, P2, P3, P4, P5)
t = (t1, t2, t3, t4, t5)
```

Remembering that each P-value is a 3D vector, we can further expand our
representation like this:

```
P = (
  (P1x, P1y, P1rot), // SwerveVector
  (P2x, P2y, P2rot),
  (P3x, P3y, P3rot),
  (P4x, P4y, P4rot),
  (P5x, P5y, P5rot),
)
```

or, equivalently, 

```
P = (
  [P1x, P2x, P3x, P4x, P5x], // N=5 components
  [P1y, P2y, P3y, P4y, P5y], // N=5 components
  [P1rot, P2rot, P3rot, P4rot, P5rot], // N=5 components
)
```

If we want to be able to assign weights for our inputs component-wise,
we can also describe our weights as vectors with the same number of components
as our inputs, i.e. `t1 = (t1x, t1y, t1rot)`. Doing this to all weights
gives us a weight vector:

```
t = (
  (t1x, t1y, t1rot),
  (t2x, t2y, t2rot),
  (t3x, t3y, t3rot),
  (t4x, t4y, t4rot),
  (t5x, t5y, t5rot),
)
```

or, equivalently,

```
t = (
  [t1x, t2x, t3x, t4x, t5x], // Sums to 1, N=5 components
  [t1y, t2y, t3y, t4y, t5y], // Sums to 1, N=5 components
  [t1rot, t2rot, t3rot, t4rot, t5rot], // Sums to 1, N=5 components
)
```

Generalizing to `N` inputs, we want to calculate a single output vector,
which may be represented as follows:

```
P = (Px, Py, Prot)
Px = [P1x, P2x, ... PNx]
Py = [P1y, P2y, ... PNy]
Prot = [P1rot, P2rot, ... PNrot]
```

Our weights may similarly be represented by a set of vectors, as follows:

```
t = (tx, ty, trot)
tx = [t1x, t2x, ..., tNx] // tx = Vector<N>
ty = [t1y, t2y, ..., tNy]
trot = [t1rot, t2rot, ..., tNrot]
```

To calculate our total output as a weighted blend of our inputs, we need to
multiply our input components by their corresponding weights:

```
output(P, t) = (Px * tx, Py * ty, Prot * trot) = (
   [P1x * t1x, P2x * t2x, ..., PNx * tNx],
   [P1y * t1y, P2y * t2y, ..., PNy * tNy],
   [P1rot * t1rot, P2rot * t2rot, ..., PNrot * tNrot],
)
```

### Constraints and Rules


