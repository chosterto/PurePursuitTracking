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

    /**
     * @brief Gets all points of the path
     * 
     * @return path of the robot
     */
    std::vector<Vec2D> GetPath();
    /**
     * @brief Adds a waypoint to the path
     * 
     * @param x x coordinate of point
     * @param y y coordinate of point
     */
    void InjectWaypoint(double x, double y);
    /**
     * @brief sets the origin point of path
     * 
     * @param x x coordinate of point
     * @param y y coordinate of point
     */
    void SetOrigin(double x, double y);
    /**
     * @brief Calculates points in between a given set of waypoints
     * 
     * @param spacing distance between each point
     */
    void Interpolate(double spacing);
    /**
     * @brief Smooths the robot path
     * 
     * @param a weight data
     * @param b weight smooth
     * @param tolerance tolerance of smoothed path
     */
    void SmoothPath(double a, double b, double tolerance);
    /**
     * @brief translates waypoints by an offset x and y value
     * 
     * @param x offset of x
     * @param y offset of y
     */
    void Translate(double x, double y);
    /**
     * @brief Clears and resets robot path
     * 
     */
    void ClearPath();

protected:
    double Curvature(Vec2D P, Vec2D Q, Vec2D R);
    double FindMaxVelocity(int index, double k);
    void CalculateDistances();
    void CalculateTargetVelocities();
    Vec2D ClosestPoint(Vec2D pos);
    Vec2D LookAheadPoint(Vec2D start, Vec2D end, Vec2D pos);
    double CurvatureOfArc(Vec2D pos, double heading);

    std::vector<Vec2D> path;
    Vec2D lastLookAheadPoint;
    Vec2D offset{0, 0};
    int lastClosestPointIdx = 0;
    double l_maxSpeed;
    double l_maxAcceleration;
    double l_lookAheadDistance;
};

#endif
