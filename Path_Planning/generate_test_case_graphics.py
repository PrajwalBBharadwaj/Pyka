import math
from pathlib import Path as FilePath

import matplotlib.pyplot as plt
import numpy as np


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


class Path:
    def __init__(self, center, radius, start_angle, end_angle, start_height, end_height):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)
        self.start_angle = float(start_angle)
        self.end_angle = float(end_angle)
        self.start_height = float(start_height)
        self.end_height = float(end_height)


class Obstacle:
    def __init__(self, vertices, height):
        self.vertices = np.array(vertices, dtype=float)
        self.height = float(height)


def point_in_convex_polygon(point, polygon) -> bool:
    if len(polygon) < 3:
        return False

    for i in range(len(polygon)):
        vertex_a = polygon[i]
        vertex_b = polygon[(i + 1) % len(polygon)]
        edge = vertex_b - vertex_a
        rel = point - vertex_a
        cross = edge[0] * rel[1] - edge[1] * rel[0]
        if cross > 0:
            return False
    return True


def sample_path(path: Path, sample_distance: float):
    sweep_deg = math.fmod(path.end_angle - path.start_angle, 360.0)
    if sweep_deg < 0.0:
        sweep_deg += 360.0

    sweep_rad = deg2rad(sweep_deg)
    arc_length = path.radius * sweep_rad
    sample_count = max(1, math.ceil(arc_length / sample_distance))

    samples = []
    for i in range(sample_count + 1):
        progress = i / sample_count
        angle_deg = path.start_angle + progress * sweep_deg
        angle_rad = deg2rad(angle_deg)
        x = path.center[0] + path.radius * math.sin(angle_rad)
        y = path.center[1] + path.radius * math.cos(angle_rad)
        z = path.start_height + progress * (path.end_height - path.start_height)
        samples.append(
            {
                "progress": progress,
                "angle_deg": angle_deg,
                "point": np.array([x, y], dtype=float),
                "height": z,
            }
        )
    return samples, sweep_deg


def intersects(path: Path, obstacle: Obstacle, sample_distance: float):
    samples, sweep_deg = sample_path(path, sample_distance)
    hit_indices = []
    for idx, sample in enumerate(samples):
        if sample["height"] <= obstacle.height and point_in_convex_polygon(sample["point"], obstacle.vertices):
            hit_indices.append(idx)
    return bool(hit_indices), hit_indices, samples, sweep_deg


def plot_test_case(name, path, obstacle, expected, sample_distance, output_dir):
    actual, hit_indices, samples, sweep_deg = intersects(path, obstacle, sample_distance)

    fig, ax = plt.subplots(figsize=(6, 6))

    polygon = np.vstack([obstacle.vertices, obstacle.vertices[0]])
    ax.fill(polygon[:, 0], polygon[:, 1], color="#f4a261", alpha=0.35, label="Obstacle")
    ax.plot(polygon[:, 0], polygon[:, 1], color="#bc6c25", linewidth=2)

    dense_progress = np.linspace(0.0, 1.0, 300)
    dense_points = []
    for progress in dense_progress:
        angle_deg = path.start_angle + progress * sweep_deg
        angle_rad = deg2rad(angle_deg)
        dense_points.append(
            [
                path.center[0] + path.radius * math.sin(angle_rad),
                path.center[1] + path.radius * math.cos(angle_rad),
            ]
        )
    dense_points = np.array(dense_points)
    ax.plot(dense_points[:, 0], dense_points[:, 1], color="#1d3557", linewidth=2.5, label="Arc")

    sample_points = np.array([sample["point"] for sample in samples])
    ax.scatter(sample_points[:, 0], sample_points[:, 1], color="#457b9d", s=18, label="Samples", zorder=3)

    if hit_indices:
        hits = sample_points[hit_indices]
        ax.scatter(hits[:, 0], hits[:, 1], color="#d62828", s=36, label="Intersecting samples", zorder=4)

    ax.scatter(path.center[0], path.center[1], color="#2a9d8f", s=28, label="Center", zorder=4)

    start = samples[0]["point"]
    end = samples[-1]["point"]
    ax.scatter(start[0], start[1], color="#2a9d8f", marker="^", s=70, zorder=4, label="Start / End")
    ax.scatter(end[0], end[1], color="#2a9d8f", marker="s", s=60, zorder=4)

    status = "PASS" if actual == expected else "FAIL"
    ax.set_title(
        f"{name}\nexpected={expected}, actual={actual}, result={status}",
        fontsize=11,
    )
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=8)

    all_x = np.concatenate([polygon[:, 0], dense_points[:, 0], [path.center[0]]])
    all_y = np.concatenate([polygon[:, 1], dense_points[:, 1], [path.center[1]]])
    padding = 2.0
    ax.set_xlim(all_x.min() - padding, all_x.max() + padding)
    ax.set_ylim(all_y.min() - padding, all_y.max() + padding)

    fig.tight_layout()
    fig.savefig(output_dir / f"{name}.png", dpi=180)
    plt.close(fig)

    return {
        "name": name,
        "expected": expected,
        "actual": actual,
        "status": status,
    }


def plot_summary(results, output_dir):
    fig, ax = plt.subplots(figsize=(10, max(4, len(results) * 0.45)))
    ax.axis("off")

    lines = ["Obstacle Detection Test Summary", ""]
    for result in results:
        lines.append(
            f"{result['name']}: expected={result['expected']}, actual={result['actual']} -> {result['status']}"
        )

    ax.text(0.01, 0.99, "\n".join(lines), va="top", ha="left", fontsize=11, family="monospace")
    fig.tight_layout()
    fig.savefig(output_dir / "summary.png", dpi=180)
    plt.close(fig)


def main():
    sample_distance = 0.1

    paths = {
        "path_1": Path((0, 0), 15, 0, 90, 25, 15),
        "path_2": Path((10, 10), 5, -45, 225, 18, 30),
        "path_3": Path((10, -10), 15, -15, 15, 60, 10),
        "path_4": Path((0, 0), 10, 350, 10, 5, 5),
    }

    obstacles = {
        "obstacle_1": Obstacle(((10, 10), (10, 0), (0, 10)), 30),
        "obstacle_2": Obstacle(((10, 0), (10, 10), (20, 10), (20, 0)), 20),
        "obstacle_3": Obstacle(((0, 10), (0, 20), (10, 20), (10, 10)), 20),
    }

    tests = [
        ("path_1_vs_obstacle_1", paths["path_1"], obstacles["obstacle_1"], False),
        ("path_1_vs_obstacle_2", paths["path_1"], obstacles["obstacle_2"], True),
        ("path_1_vs_obstacle_3", paths["path_1"], obstacles["obstacle_3"], False),
        ("path_2_vs_obstacle_1", paths["path_2"], obstacles["obstacle_1"], True),
        ("path_2_vs_obstacle_2", paths["path_2"], obstacles["obstacle_2"], False),
        ("path_2_vs_obstacle_3", paths["path_2"], obstacles["obstacle_3"], True),
        ("path_3_vs_obstacle_1", paths["path_3"], obstacles["obstacle_1"], False),
        ("path_3_vs_obstacle_2", paths["path_3"], obstacles["obstacle_2"], True),
        ("path_3_vs_obstacle_3", paths["path_3"], obstacles["obstacle_3"], False),
        ("path_4_vs_obstacle_1", paths["path_4"], obstacles["obstacle_1"], True),
        ("path_4_vs_obstacle_2", paths["path_4"], obstacles["obstacle_2"], False),
        ("path_4_vs_obstacle_3", paths["path_4"], obstacles["obstacle_3"], True),
    ]

    output_dir = FilePath("test_case_graphics")
    output_dir.mkdir(exist_ok=True)

    results = []
    for name, path, obstacle, expected in tests:
        results.append(plot_test_case(name, path, obstacle, expected, sample_distance, output_dir))

    plot_summary(results, output_dir)


if __name__ == "__main__":
    main()
