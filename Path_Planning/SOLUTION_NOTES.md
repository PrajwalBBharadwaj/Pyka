# Obstacle Detection Solution Notes

## Overview

This document summarizes:

- the key implementation decisions in `obstacle_detect.cpp`,
- the reasoning behind those decisions,
- the test cases used to validate the solution,
- and the generated graphics for each test case.

Related files:

- [obstacle_detect.cpp](/home/prajwal/Documents/Job_apps/Interviews/Pyka/obstacle_detect.cpp)
- [DECISIONS.md](/home/prajwal/Documents/Job_apps/Interviews/Pyka/DECISIONS.md)
- [generate_test_case_graphics.py](/home/prajwal/Documents/Job_apps/Interviews/Pyka/generate_test_case_graphics.py)

## Key Decisions

### 1. Sampling-Based Intersection Check

Used a sampling-based approach instead of an exact analytical intersection method.

Why:

- The prompt explicitly allows an approximate solution.
- It keeps the implementation straightforward and readable.
- It is easy to tune using `sample_distance`.

Implication:

- The algorithm may miss very small or single-point contacts if no sample lands exactly on the intersection.

### 2. Clockwise Arc Sampling

The problem defines angles clockwise from the `+Y` axis.

So the sampled arc point is computed as:

```cpp
x = center.x + radius * sin(angle)
y = center.y + radius * cos(angle)
```

This matches the stated coordinate convention instead of the standard math convention.

### 3. Angle Sweep Normalization

Normalized the angular sweep so arcs that cross the `360 -> 0` boundary are handled correctly.

Example:

- `350 -> 10` should be treated as a short `20` degree clockwise arc.
- Raw subtraction gives `-340`, which would sample the wrong path.

Implementation:

```cpp
float sweep_deg = std::fmod(path.getEndAngle() - path.getStartAngle(), 360.0f);
if (sweep_deg < 0.0f) {
    sweep_deg += 360.0f;
}
```

### 4. Convex Polygon Containment Check

Used a cross-product based point-in-convex-polygon test.

Why:

- The obstacle is guaranteed to be convex.
- The vertices are guaranteed to be in clockwise order.

Boundary handling:

- Points on an edge or vertex are treated as inside the polygon.
- This is implemented by rejecting only when `cross > 0`.

### 5. Invalid Polygon Guard

Added a guard for polygons with fewer than 3 vertices:

```cpp
if (no_of_vertices < 3) return false;
```

This prevents invalid polygon inputs from being treated as intersecting.

### 6. Mixed Precision Choice

Kept the provided `float`-based scalar fields in `Path` and `Obstacle`, since those definitions were treated as fixed.

Used `double` for sampled `x` and `y` coordinates because:

- `Eigen::Vector2d` stores doubles,
- the path center is already `double`-based,
- this avoids narrowing the final sampled XY coordinates.

## Test Results

All current runtime results:

- `path_1` vs `obstacle_1`: expected `false`, actual `false`, `PASS`
- `path_1` vs `obstacle_2`: expected `true`, actual `true`, `PASS`
- `path_1` vs `obstacle_3`: expected `false`, actual `false`, `PASS`
- `path_2` vs `obstacle_1`: expected `true`, actual `true`, `PASS`
- `path_2` vs `obstacle_2`: expected `false`, actual `false`, `PASS`
- `path_2` vs `obstacle_3`: expected `true`, actual `true`, `PASS`
- `path_3` vs `obstacle_1`: expected `false`, actual `false`, `PASS`
- `path_3` vs `obstacle_2`: expected `true`, actual `true`, `PASS`
- `path_3` vs `obstacle_3`: expected `false`, actual `false`, `PASS`
- `path_4` vs `obstacle_1`: expected `true`, actual `true`, `PASS`
- `path_4` vs `obstacle_2`: expected `false`, actual `false`, `PASS`
- `path_4` vs `obstacle_3`: expected `true`, actual `false`, `FAIL`

Summary image:

- [summary.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/summary.png)

## Graphics By Test Case

### Base Test Cases

- `path_1` vs `obstacle_1`: [path_1_vs_obstacle_1.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_1_vs_obstacle_1.png)
- `path_1` vs `obstacle_2`: [path_1_vs_obstacle_2.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_1_vs_obstacle_2.png)
- `path_1` vs `obstacle_3`: [path_1_vs_obstacle_3.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_1_vs_obstacle_3.png)
- `path_2` vs `obstacle_1`: [path_2_vs_obstacle_1.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_2_vs_obstacle_1.png)
- `path_2` vs `obstacle_2`: [path_2_vs_obstacle_2.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_2_vs_obstacle_2.png)
- `path_2` vs `obstacle_3`: [path_2_vs_obstacle_3.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_2_vs_obstacle_3.png)
- `path_3` vs `obstacle_1`: [path_3_vs_obstacle_1.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_3_vs_obstacle_1.png)
- `path_3` vs `obstacle_2`: [path_3_vs_obstacle_2.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_3_vs_obstacle_2.png)
- `path_3` vs `obstacle_3`: [path_3_vs_obstacle_3.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_3_vs_obstacle_3.png)

### Additional Wraparound Test Cases

- `path_4` vs `obstacle_1`: [path_4_vs_obstacle_1.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_4_vs_obstacle_1.png)
- `path_4` vs `obstacle_2`: [path_4_vs_obstacle_2.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_4_vs_obstacle_2.png)
- `path_4` vs `obstacle_3`: [path_4_vs_obstacle_3.png](/home/prajwal/Documents/Job_apps/Interviews/Pyka/test_case_graphics/path_4_vs_obstacle_3.png)

## Important Edge Case

The `path_4` vs `obstacle_3` case is intentionally useful because it exposes the main tradeoff of the chosen method.

Why it fails:

- The path touches the obstacle only at a very small boundary location.
- The algorithm checks only discrete sampled points.
- With `sample_distance = 0.1f`, no sample lands exactly on the touching point.

Result:

- expected `true`
- actual `false`

This is consistent with the design choice to use an approximate method.

If higher fidelity is needed:

- reduce `sample_distance`, or
- replace the sampling approach with a more exact geometric intersection method.

## Recommendation

For an interview submission, this solution is still defendable because:

- it matches the prompt’s approximate-solution allowance,
- it correctly handles clockwise angle wraparound,
- it clearly documents the known limitation,
- and the graphics make the behavior and tradeoffs easy to explain.
