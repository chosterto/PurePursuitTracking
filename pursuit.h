#ifndef PURSUIT_H_INCLUDED
#define PURSUIT_H_INCLUDED

#include <vector>
#include <cmath>


struct Vec2D {
    double x = 0;
    double y = 0;
    double speed;
    double distance;
};

class PurePursuitController {
public:
    PurePursuitController(double maxSpeed, double maxAcceleration);

    std::vector<Vec2D> GetPath();
    void InjectWaypoint(double x, double y);
    void SetOrigin(double x, double y);
    void Interpolate(double spacing);
    void SmoothPath(double a, double b, double tolerance);

private:
    double Curvature(Vec2D P, Vec2D Q, Vec2D R);
    double FindMaxVelocity(int index, double maxVelo, double k);
    void CalculateDistances();
    void CalculateTargetVelocities();

    std::vector<Vec2D> path;
    double l_maxSpeed;
    double l_maxAcceleration;
};

#endif
