# Obstacle Detection Design Decisions

## Goal

Determine whether a 3D path intersects a convex obstacle.

- The path is a clockwise circular arc in the XY plane.
- The path height changes linearly from `start_height` to `end_height`.
- The obstacle is a convex polygon in the XY plane with a fixed height.
- A point intersects the obstacle if:
  - its XY position lies within the polygon bounds, and
  - its height is less than or equal to the obstacle height.

## Main Approach

Used a sampling-based solution instead of an exact analytical geometry solution.

Why:

- The problem statement explicitly allows an approximate solution.
- Sampling is simple to implement and explain.
- It keeps the logic readable for an interview-style exercise.

High-level algorithm:

1. Compute the clockwise angular sweep of the arc.
2. Estimate arc length from radius and sweep.
3. Choose the number of samples from `arc_length / sample_distance`.
4. For each sampled point:
   - compute XY position on the arc,
   - compute interpolated height,
   - test XY containment in the polygon,
   - compare point height with obstacle height.
5. Return `true` as soon as one sampled point intersects.

## Angle Convention

The problem defines angles as clockwise from the `+Y` axis.

That is why sampled points are computed as:

```cpp
x = center.x + radius * sin(angle)
y = center.y + radius * cos(angle)
```

This differs from the more common math convention of counterclockwise from `+X`.

## Sweep Normalization

Raw subtraction of `end_angle - start_angle` can fail when the arc crosses the `0/360` boundary.

Example:

- `start = 350`
- `end = 10`

The intended clockwise arc is `20` degrees, but raw subtraction gives `-340`.

Decision:

- Normalize the angular sweep into the equivalent clockwise sweep within one revolution.

Implementation idea:

```cpp
float sweep_deg = std::fmod(end_angle - start_angle, 360.0f);
if (sweep_deg < 0.0f) {
    sweep_deg += 360.0f;
}
```

This makes cases like `350 -> 10` behave as a short wrapped arc instead of incorrectly sampling the long way around.

## Polygon Containment Test

Used a cross-product based point-in-convex-polygon test.

Why:

- The obstacle is guaranteed to be convex.
- The vertices are guaranteed to be provided in clockwise order.
- This makes the test short, efficient, and easy to reason about.

Decision:

- Treat boundary points as inside the polygon.

Implementation detail:

- For a clockwise polygon, an interior point should satisfy `cross <= 0` for each edge.
- The code only rejects when `cross > 0`.
- This means points on an edge or vertex are counted as inside.

## Invalid Polygon Guard

Added a guard for polygons with fewer than 3 vertices.

Why:

- Fewer than 3 points do not define a polygon.
- Without this guard, the function could incorrectly return `true` for invalid inputs.

Decision:

```cpp
if (no_of_vertices < 3) return false;
```

## Sampling Resolution

Used `sample_distance` to control the density of sampled points along the arc.

Why:

- Smaller values improve detection accuracy.
- Larger values reduce computation.

Decision:

- Compute:

```cpp
no_of_samples = ceil(arc_length / sample_distance)
```

- Always use at least one sample interval so start and end points are still evaluated.

## Height Handling

The height of the path is linearly interpolated using sampling progress:

```cpp
pt_height = start_height + progress * (end_height - start_height)
```

Decision:

- A sampled point is considered blocked when:

```cpp
pt_height <= obstacle.height
```

This follows the problem statement directly.

## Numeric Type Decisions

The provided `Path` and `Obstacle` interfaces use `float` for scalar values.

Decision:

- Kept those scalar values as provided instead of redesigning the interfaces.
- Used `double` for computed sampled XY coordinates because:
  - `Eigen::Vector2d` stores doubles,
  - `getCenter()` returns double-based coordinates,
  - it avoids narrowing the final sampled point coordinates.

This is a constrained mixed-precision design caused by the fixed class definitions.

## Test Coverage Decisions

Kept the original provided-style tests and added one wraparound path:

- `path_4({0, 0}, 10, 350, 10, 5, 5)`

Why:

- It exercises angle normalization across the `360 -> 0` boundary.

Additional checks:

- one positive case against an existing obstacle,
- one negative case against an existing obstacle,
- one edge case where the path touches a polygon vertex.

## Known Limitation

This is a sampling-based method, so it is approximate.

Important consequence:

- Very small intersections or single-point touches can be missed if no sample lands exactly on that location.

Example in the tests:

- `path_4` can touch `obstacle_3` at the vertex `(0, 10)`.
- With a coarse `sample_distance`, the exact touching point may not be sampled.
- Reducing `sample_distance` improves the chance of detecting that case.

This limitation is acceptable for this solution because the prompt explicitly states that perfect precision is not required.
