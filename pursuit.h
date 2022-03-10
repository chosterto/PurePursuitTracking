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
    PurePursuitController(double maxSpeed, double maxAcceleration, double lookAheadDistance);

    std::vector<Vec2D> GetPath();
    void InjectWaypoint(double x, double y);
    void SetOrigin(double x, double y);
    void Interpolate(double spacing);
    void SmoothPath(double a, double b, double tolerance);

private:
    double Curvature(Vec2D P, Vec2D Q, Vec2D R);
    double FindMaxVelocity(int index, double k);
    void CalculateDistances();
    void CalculateTargetVelocities();
    Vec2D ClosestPoint(Vec2D pos);
    Vec2D LookAheadPoint(Vec2D start, Vec2D end, Vec2D pos);
    double CurvatureOfArc(Vec2D pos, double heading);

    std::vector<Vec2D> path;
    Vec2D lastLookAheadPoint;
    int lastClosestPointIdx = 0;
    double l_maxSpeed;
    double l_maxAcceleration;
    double l_lookAheadDistance;
};

#endif
