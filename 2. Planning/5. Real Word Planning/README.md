# Real-World Planning

## Extended Reading

> At this point, you have the knowledge to read through a recent paper on path planning. The following paper, Path Planning for [Non-Circular Micro Aerial Vehicles in Constrained Environments](https://www.cs.cmu.edu/~maxim/files/pathplanforMAV_icra13.pdf), addresses the problem of path planning for a quadrotor.

> It is an enjoyable read that culminates these past few sections of path planning, as it references a number of planning methods that you have learned, and introduces a present-day application of path planning. Reading the paper will help you gain an appreciation for this area of research, as well as help you gain confidence in the subject.

> Some additional definitions that you may find helpful while reading the paper:

> - **Anytime algorithm:** an anytime algorithm is an algorithm that will return a solution even if it's computation is halted before it finishes searching the entire space. The longer the algorithm plans, the more optimal the solution will be.

> - **RRT\*:** RRT* is a variant of RRT that tries to smooth the tree branches at every step. It does so by looking to see whether a child node can be swapped with it's parent (or it's parent's parent, etc) to produce a more direct path. The result is a less zig-zaggy and more optimal path.

## Papers

### Path Planning for Non-Circular Micro Aerial Vehicles in Constrained Environments

Added [here](papers/pathplanforMAV_icra13.pdf) for brevity.

> Operating micro aerial vehicles (MAVs) outside of
the bounds of a rigidly controlled lab environment, specifically
one that is unstructured and contains unknown obstacles, poses
a number of challenges. One of these challenges is that of
quickly determining an optimal (or nearly so) path from the
MAVs current position to a designated goal state. Past work
in this area using full-size unmanned aerial vehicles (UAVs)
has predominantly been performed in benign environments.
>
> However, due to their small size, MAVs are capable of operating
in indoor environments which are more cluttered. This requires
planners to account for the vehicle heading in addition to its
spatial position in order to successfully navigate. In addition,
due to the short flight times of MAVs along with the inherent
hazards of operating in close proximity to obstacles, we desire
the trajectories to be as cost-optimal as possible. Our approach
uses an anytime planner based on A\*
that performs a graph
search on a four-dimensional (4-D) (x,y,z,heading) lattice.
>
> This allows for the generation of close-to-optimal trajectories based
on a set of precomputed motion primitives along with the
capability to provide trajectories in real-time allowing for onthe-fly re-planning as new sensor data is received. We also account for arbitrary vehicle shapes, permitting the use of a noncircular footprint during the planning process. By not using the
overly conservative circumscribed circle for collision checking,
we are capable of successfully finding optimal paths through
cluttered environments including those with narrow hallways.
Analytically, we show that our planner provides bounds on the
sub-optimality of the solution it finds. Experimentally, we show
that the planner can operate in real-time
