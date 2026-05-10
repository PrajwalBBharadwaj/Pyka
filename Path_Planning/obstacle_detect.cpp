#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

constexpr float deg2rad(float deg) {
    return deg * M_PI / 180.0;
}

class Path {
private:
    const Eigen::Vector2d _center;
    const float _radius;
    const float _start_angle;
    const float _end_angle;
    const float _start_height;
    const float _end_height;

public:
    Path(const Eigen::Vector2d& center,
         float radius,
         float start_angle,
         float end_angle,
         float start_height,
         float end_height)
        : _center(center),
          _radius(radius),
          _start_angle(start_angle),
          _end_angle(end_angle),
          _start_height(start_height),
          _end_height(end_height) {}

    const Eigen::Vector2d& getCenter() const { return _center; }
    float getRadius() const { return _radius; }
    float getStartAngle() const { return _start_angle; }
    float getEndAngle() const { return _end_angle; }
    float getStartHeight() const { return _start_height; }
    float getEndHeight() const { return _end_height; }
};

struct Obstacle {
    const std::vector<Eigen::Vector2d> vertices;
    const float height;
    Obstacle(const std::vector<Eigen::Vector2d>& vertices,
             float height)
        : vertices(vertices),
          height(height) {}
};

bool pointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {

    int no_of_vertices = polygon.size();
    if (no_of_vertices < 3)
        return false;

    for (int i = 0; i < no_of_vertices; i++) {
        const Eigen::Vector2d& vertex_a = polygon[i];
        const Eigen::Vector2d& vertex_b = polygon[(i + 1) % no_of_vertices];

        Eigen::Vector2d edge = vertex_b - vertex_a;
        Eigen::Vector2d rel  = point - vertex_a;

        double cross = edge.x() * rel.y() - edge.y() * rel.x();

        // clockwise polygon assumption
        if (cross > 0) return false;
    }

    return true;
}


/*----- Intersection check -----*/
bool intersects(const Path& path, const Obstacle& obs, float sample_distance) { 

    if (sample_distance <= 0) return false;

    // Wrapping sweep to be within a circle since problem statement mentions "circular arc" path
    float sweep_deg = std::fmod(path.getEndAngle() - path.getStartAngle(), 360.0f);

    // If end angle is numerically less than start angle, wrap across 360 degrees
    // so the sweep still represents the intended clockwise arc. 
    if (sweep_deg < 0.0f) {
        sweep_deg += 360.0f;
    }

    float sweep_rad = deg2rad(sweep_deg);

    float arc_length = path.getRadius() * sweep_rad;

    int no_of_samples = std::max(1,
        static_cast<int>(std::ceil(arc_length / sample_distance))
    );

    for (int i = 0; i <= no_of_samples; ++i) {

        float progress = static_cast<float>(i) / no_of_samples;

        float angle_deg = path.getStartAngle() + progress * sweep_deg;
        float angle_rad = deg2rad(angle_deg);

        // Clockwise from +Y axis
        double x = path.getCenter().x() + path.getRadius() * std::sin(angle_rad);
        double y = path.getCenter().y() + path.getRadius() * std::cos(angle_rad);

        Eigen::Vector2d pt(x, y);

        float pt_height = path.getStartHeight() +
                   progress * (path.getEndHeight() - path.getStartHeight());

        if (pt_height <= obs.height && pointInConvexPolygon(pt, obs.vertices)) {
            return true;
        }
    }

    return false;
}

/*------- Test function --------*/
void runTest(const std::string& name, bool actual, bool expected) {

    std::cout << name << ":";

    std::cout << " (expected " << std::boolalpha << expected
              << ", got " << std::boolalpha << actual << ")";
    
    if (actual == expected) {
        std::cout << " PASS" << std::endl;
    } else {
        std::cout << " FAIL" << std::endl;
    }

}

int main() {

    // Given Test Paths
    Path path_1({0, 0}, 15, 0, 90, 25, 15);
    Path path_2({10, 10}, 5, -45, 225, 18, 30);
    Path path_3({10, -10}, 15, -15, 15, 60, 10);

    // Additional Test Paths
    Path path_4({0, 0}, 10, 350, 10, 5, 5);

    // Given Test Obstacles
    Obstacle obstacle_1(
        std::vector<Eigen::Vector2d>({{10, 10},{10, 0},{0, 10}}), 30);

    Obstacle obstacle_2(
        std::vector<Eigen::Vector2d>({{10, 0}, {10, 10}, {20, 10}, {20, 0}}), 20);

    Obstacle obstacle_3(
        std::vector<Eigen::Vector2d>({{0, 10}, {0, 20}, {10, 20}, {10, 10}}), 20);


    // Configurable sample distance parameter for sampling points along arc
    // Change sample_distance to 0.01f to PASS the last test case
    const float sample_distance = 0.1f;

    // Expected results

    runTest(
        "path_1 intersects obstacle_1:",
        intersects(path_1, obstacle_1, sample_distance),
        false);

    runTest(
        "path_1 intersects obstacle_2:",
        intersects(path_1, obstacle_2, sample_distance),
        true);

    runTest(
        "path_1 intersects obstacle_3:",
        intersects(path_1, obstacle_3, sample_distance),
        false);
    
    std::cout << "--------------------" << std::endl;

    runTest(
        "path_2 intersects obstacle_1",
        intersects(path_2, obstacle_1, sample_distance),
        true);

    runTest(
        "path_2 intersects obstacle_2",
        intersects(path_2, obstacle_2, sample_distance),
        false);

    runTest(
        "path_2 intersects obstacle_3",
        intersects(path_2, obstacle_3, sample_distance),
        true);
    
    std::cout << "--------------------" << std::endl;

    runTest(
        "path_3 intersects obstacle_1",
        intersects(path_3, obstacle_1, sample_distance),
        false);

    runTest(
        "path_3 intersects obstacle_2",
        intersects(path_3, obstacle_2, sample_distance),
        true);

    runTest(
        "path_3 intersects obstacle_3",
        intersects(path_3, obstacle_3, sample_distance),
        false);

    std::cout << "--------------------" << std::endl;

    runTest(
        "path_4 intersects obstacle_1",
        intersects(path_4, obstacle_1, sample_distance),
        true);

    runTest(
        "path_4 intersects obstacle_2",
        intersects(path_4, obstacle_2, sample_distance),
        false);

    // This test case shows the edge case for sampling-based method
    runTest(
        "path_4 intersects obstacle_3",
        intersects(path_4, obstacle_3, sample_distance),
        true);

    return 0;
}
